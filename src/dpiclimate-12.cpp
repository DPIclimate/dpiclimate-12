// Serial1 works with standby mode.
#define serial Serial

#define USE_SERIAL

#include <cmath>
#include <limits>

#include <dpiclimate-12.h>

// The ESP32 requires these variables to be in RTC memory so they
// keep their value over deep sleep reboots. Other architectures
// don't need that so make the RTC_DATA_ATTR macro empty when not
// compiling for the ESP32.
#if not defined RTC_DATA_ATTR
#define RTC_DATA_ATTR
#endif

RTC_DATA_ATTR char measure_cmd[] = { 0, 'M', '!', 0 };
RTC_DATA_ATTR char measure_crc_cmd[] = { 0, 'M', 'C', '!', 0 };
RTC_DATA_ATTR char additional_measure_cmd[] = { 0, 'M', 0, '!', 0 };
RTC_DATA_ATTR char additional_measure_crc_cmd[] = { 0, 'M', 'C', 0, '!', 0 };
RTC_DATA_ATTR char data_cmd[] = { 0, 'D', '0', '!', 0 };

RTC_DATA_ATTR char c_measure_cmd[] = { 0, 'C', '!', 0 };
RTC_DATA_ATTR char c_measure_crc_cmd[] = { 0, 'C', 'C', '!', 0 };
RTC_DATA_ATTR char c_additional_measure_cmd[] = { 0, 'C', 0, '!', 0 };
RTC_DATA_ATTR char c_additional_measure_crc_cmd[] = { 0, 'C', 'C', 0, '!', 0 };

RTC_DATA_ATTR static sensor_info sensors[DPIClimate12::MAX_SENSORS];

// When initialising a union you provide one value, which is assinged to the
// first member of the union.
FLOAT DPIClimate12::NaN = { std::numeric_limits<float>::signaling_NaN() };

int DPIClimate12::get_response() {
    #ifdef USE_SERIAL
        serial.print("Waiting for response");
        serial.flush();
    #endif

    int a = 0;
    uint8_t tries = 0;
    while (a < 1) {
        delay(10);
        a = m_sdi12.available();
        tries++;

        // Wait up to 500ms, some sensors are not ready to send even
        // if they have sent the attention command after a measure.
        if (a < 1 && tries > 50) {
            #ifdef USE_SERIAL
                serial.println(" - timeout");
                serial.flush();
            #endif
            return -1;
        }
    }

    int i = 0;
    int ch;
    tries = 0;
    memset(response_buffer, 0, sizeof(response_buffer));
    while (i < BUF_LEN && tries < 10) {
        if (m_sdi12.available() < 1) {
            delay(10);
            tries++;
            continue;
        }

        ch = m_sdi12.read();
        if (ch == 0x0a || ch == 0x0d || (ch >= 32 && ch <= 126)) {
            // Reset the tries counter whenever a valid character is received.
            tries = 0;
            if (ch >= 32) {
                response_buffer[i++] = ch;
                response_buffer[i] = 0;
            }

            if (ch == '\n') {
                break;
            }
        } else {
            // Assume any invalid character means the response is useless
            // and return an error.
            #ifdef USE_SERIAL
                serial.print(" - invalid character ");
                serial.print(ch, HEX);
                serial.print(" ");
                serial.flush();
            #endif
            return -1;
        }
    }

    #ifdef USE_SERIAL
        serial.print(", received: "); serial.write(response_buffer, i); serial.print(", length: "); serial.print(i); serial.println();
    #endif

    return i;
}

int DPIClimate12::get_response(char *buffer, int buffer_len) {
    int i = get_response();
    if (i > 0 && buffer != 0 && buffer_len > 0) {
        int len = i < buffer_len ? i : buffer_len;
        memcpy(buffer, response_buffer, len);
    }

    return i;
}

/**
 * Parse up to MAX_VALUES floats from response_buffer.
 *
 * response_buffer is assumed to contain a valid response from D command, so
 * an SDI-12 address, then a series of floats delimited by + and -. response_buffer
 * is assumed to be null-terminated, not terminated with <CR><LF>.
 *
 * This function returns any successfully parsed floats when an invalid
 * character is encountered.
 *
 * Returns the number of floats parsed. The values are found in values[0 .. n-1].
 */
int DPIClimate12::parse_values(int value_idx) {
    // Assume the first character is an SDI-12 address.
    if (response_buffer[1] != '+' && response_buffer[1] != '-') {
        #ifdef USE_SERIAL
        serial.println("Invalid response, expected first value to start with + or -");
        #endif
        return -1;
    }

    int v_count = 0;
    str_val[0] = response_buffer[1];
    int sv_idx = 1;

    // Start looking at the 3rd character of the reponse because the first character
    // is the sensor address, and the second character has already been checked and
    // put into str_val above.
    int i = 2;

    // Adding 1 to the length of the response so the loop sees the null terminator.
    int max = strlen(response_buffer) + 1;
    while (i < max) {
        char ch = response_buffer[i++];
        if (ch != '+' && ch != '-' && ch != 0) {
            if (ch == '.' || (ch >= '0' && ch <= '9')) {
                str_val[sv_idx++] = ch;
                str_val[sv_idx] = 0;
            }

            // The response may contain a 3 character CRC after the values, this
            // code will soak them up without adding them into str_val, and when
            // the response_buffer null terminator is reached the last value will
            // be decoded, below.
            continue;
        }

        m_values[value_idx].value = atof(str_val);
        value_idx++;
        v_count++;
        str_val[0] = ch;
        sv_idx = 1;

        if (value_idx >= MAX_VALUES) {
            break;
        }
    }

    return v_count;
}

int DPIClimate12::do_any_measure(char *cmd, bool crc) {
    #ifdef USE_SERIAL
        serial.print("Sending command: "); serial.write(cmd, strlen(cmd)); serial.println();
    #endif

    // EnviroPros seem to need a pause before they're ready to respond after they do anything.
    delay(500);

    m_sdi12.clearBuffer();
    m_sdi12.sendCommand(cmd);
    int len = get_response();
    if (len != 5) {
        if (len < 1) {
            #ifdef USE_SERIAL
                serial.println("timeout");
            #endif
        } else {
            #ifdef USE_SERIAL
                serial.print("Invalid response to measure command: "); serial.write(response_buffer, len); serial.println();
            #endif
        }
        return -1;
    }

    uint8_t num_values = (uint8_t)response_buffer[4] - '0';
    response_buffer[4] = 0;
    long delay_seconds = atol(&response_buffer[1]);

    #ifdef USE_SERIAL
    serial.print("Wait "); serial.print(delay_seconds); serial.print(", for "); serial.print(num_values); serial.println(" values");
    #endif

    if (delay_seconds > 0) {
        uint32_t start_ms = millis();
        uint32_t end_ms = start_ms + (delay_seconds * 1000);
        uint32_t now_ms = start_ms;
        len = 0;
        while (now_ms < end_ms && len < 1) {
            delay(200);
            now_ms = millis();

            // Wait for the service request from the sensor. It must send a service request
            // if it specifies a wait period > 0 (SDI-12 spec v1.4, pg 13, ss 4.4.6).
            len = get_response();
        }

        if (len != 1) {
            #ifdef USE_SERIAL
            serial.println("Did not get expected service request from sensor");
            #endif
            return -1;
        }
    }

    // EnviroPros seem to need a pause before they're ready to respond after they do anything.
    delay(500);

    return do_data_commands(cmd[0], num_values, crc);
}

int DPIClimate12::do_measure(uint8_t address, bool crc) {
    char *cmd = crc ? measure_crc_cmd : measure_cmd;
    cmd[0] = address;
    return do_any_measure(cmd, crc);
}

int DPIClimate12::do_additional_measure(uint8_t address, uint8_t additional, bool crc) {
    char *cmd = crc ? additional_measure_crc_cmd : additional_measure_cmd;
    cmd[0] = address;
    cmd[2] = additional;
    return do_any_measure(cmd, crc);
}

int DPIClimate12::do_data_commands(char addr, uint8_t num_values, bool crc) {
    data_cmd[0] = addr;

    int value_count = 0;
    int d_cmd_count = 0;

    while (value_count < num_values && d_cmd_count < 10) {
        data_cmd[2] = '0' + d_cmd_count++;

        #ifdef USE_SERIAL
        serial.print("Sending command: "); serial.write(data_cmd, strlen(data_cmd)); serial.println();
        #endif
        m_sdi12.sendCommand(data_cmd);
        int len = get_response();
        if (len < 3) {
            #ifdef USE_SERIAL
            serial.println("Did not get expected readings from sensor");
            #endif
            return -1;
        }

        if (crc && ! check_crc()) {
            #ifdef USE_SERIAL
            serial.println("Bad CRC");
            #endif
            return -1;
        }

        len = parse_values(value_count);
        if (len < 1) {
            return -1;
        }

        value_count += len;
        delay(10);
    }

    return value_count;
}

int DPIClimate12::do_concurrent_measures(uint8_t *addresses, int num_addresses, uint8_t measure_id, result_info *results_info, bool crc) {
    uint8_t num_values;

    if (addresses == nullptr || results_info == nullptr || num_addresses < 1 || measure_id > 9) {
        return -1;
    }

    char *cmd;
    if (measure_id < 1) {
        cmd = crc ? c_measure_crc_cmd : c_measure_cmd;
    } else {
        if (crc) {
            cmd = c_additional_measure_crc_cmd;
            cmd[3] = '0' + measure_id;
        } else {
            cmd = c_additional_measure_cmd;
            cmd[2] = '0' + measure_id;
        }
    }

    if (num_addresses > MAX_SENSORS) {
        num_addresses = MAX_SENSORS;
    }

    for (int i = 0; i < num_addresses; i++) {
        cmd[0] = addresses[i];

        #ifdef USE_SERIAL
            serial.print("Sending command: "); serial.write(cmd, strlen(cmd)); serial.println();
        #endif
        m_sdi12.clearBuffer();
        m_sdi12.sendCommand(cmd);
        int len = get_response();
        if (len != 6) {
            if (len < 1) {
                #ifdef USE_SERIAL
                    serial.println("timeout");
                #endif
            } else {
                #ifdef USE_SERIAL
                    serial.print("Invalid response to measure command: "); serial.write(response_buffer, len); serial.println();
                #endif
            }
            return -1;
        }

        num_values = (((uint8_t)response_buffer[4] - '0') * 10) + (uint8_t)response_buffer[5] - '0';
        response_buffer[4] = 0;
        long delay_seconds = atol(&response_buffer[1]);

        results_info[i].delay = delay_seconds;
        results_info[i].num_values = num_values;
        #ifdef USE_SERIAL
        serial.print("Wait "); serial.print(delay_seconds); serial.print(", for "); serial.print(num_values); serial.println(" values");
        #endif
    }

    return 0;
}

int DPIClimate12::do_verification(char addr) {
    char cmd[] = { addr, 'V', '!', 0 };
    return do_any_measure(cmd, false);
}

FLOAT* DPIClimate12::get_values() {
    return m_values;
}

FLOAT DPIClimate12::get_value(int i) {
  if (i >= 0 && i < MAX_VALUES) {
    return m_values[i];
  }

  return NaN;
}

void DPIClimate12::scan_bus(sensor_list &sensor_list) {
    static char attn_cmd[] = { 0, '!', 0};
    static char info_cmd[] = { 0, 'I', '!', 0};

    int s_count = 0;
    size_t info_size = sizeof(sensor_info);
    memset((void *)&sensors, 0, sizeof(sensor_info) * MAX_SENSORS);
    for (char c = '0'; c <= '9'; c++) {
        attn_cmd[0] = c;
        m_sdi12.clearBuffer();
        m_sdi12.sendCommand(attn_cmd);
        int len = get_response();
        if (len == 1) {
            // EnviroPros & C4E salinity sensors seem to need a pause here.
            delay(500);
            info_cmd[0] = c;
            m_sdi12.clearBuffer();
            m_sdi12.sendCommand(info_cmd);
            len = get_response();
            if (len > 0) {
                size_t max = len < info_size ? len : info_size;
                memcpy(&sensors[s_count], response_buffer, max);
                s_count++;
            }
        }

        // C4E salinity sensor needs a delay here.
        delay(500);
    }

    sensor_list.count = s_count;
    sensor_list.sensors = sensors;
}

bool DPIClimate12::change_address(uint8_t from, uint8_t to) {
    char change_addr_cmd[] = { from, 'A', to, '!', 0 };
    m_sdi12.clearBuffer();
    m_sdi12.sendCommand(change_addr_cmd);
    int len = get_response();
    return len == 1 && response_buffer[0] == to;
}

bool DPIClimate12::check_crc() {
    // Assume the response buffer has the CRC at the end.
    size_t resp_len = strlen(response_buffer) - 3;

    uint16_t crc = 0;
    for (int i = 0; i < resp_len; i++) {
        crc ^= ((uint16_t)response_buffer[i] & 0xff);
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    char ascii_crc[4] = { 0, 0, 0, 0 };
    ascii_crc[0] = 0x40 | (crc >> 12);
    ascii_crc[1] = 0x40 | ((crc >> 6) & 0x3f);
    ascii_crc[2] = 0x40 | (crc & 0x3f);

    return ascii_crc[0] == response_buffer[resp_len++] &&
           ascii_crc[1] == response_buffer[resp_len++] &&
           ascii_crc[2] == response_buffer[resp_len] ;
}
