// Serial1 works with standby mode.
#define serial Serial

#define USE_SERIAL

#include <cmath>
#include <limits>

#include <dpiclimate-12.h>

#ifdef ESP32
#include "esp_log.h"
#define TAG "DPI12"
#define LOG(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#else
#define LOG log_msg

// A buffer for printing log messages.
static constexpr int MAX_MSG = 256;
static char msg[MAX_MSG];

// A printf-like function to print log messages prefixed by the current
// LMIC tick value. Don't call it before os_init();
void log_msg(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, MAX_MSG, fmt, args);
    va_end(args);
    serial.write(msg, strlen(msg));
    serial.println();
}
}
#endif

// The ESP32 requires these variables to be in RTC memory so they
// keep their value over deep sleep reboots. Other architectures
// don't need that so make the RTC_DATA_ATTR macro empty when not
// compiling for the ESP32.
#if not defined RTC_DATA_ATTR
#define RTC_DATA_ATTR
#endif

char measure_cmd[] = { 0, 'M', '!', 0 };
char measure_crc_cmd[] = { 0, 'M', 'C', '!', 0 };
char additional_measure_cmd[] = { 0, 'M', 0, '!', 0 };
char additional_measure_crc_cmd[] = { 0, 'M', 'C', 0, '!', 0 };
char data_cmd[] = { 0, 'D', '0', '!', 0 };

char c_measure_cmd[] = { 0, 'C', '!', 0 };
char c_measure_crc_cmd[] = { 0, 'C', 'C', '!', 0 };
char c_additional_measure_cmd[] = { 0, 'C', 0, '!', 0 };
char c_additional_measure_crc_cmd[] = { 0, 'C', 'C', 0, '!', 0 };

RTC_DATA_ATTR static sensor_info sensors[DPIClimate12::MAX_SENSORS];

// When initialising a union you provide one value, which is assigned to the
// first member of the union.
FLOAT DPIClimate12::NaN = { std::numeric_limits<float>::signaling_NaN() };

int DPIClimate12::waitForChar(uint32_t timeout) {
    uint32_t start = millis();
    int a = m_sdi12.available();
    while ((millis() - start) < timeout && a < 1) {
        delay(10); // Let the MCU do something else.
        a = m_sdi12.available();
    }

    uint32_t end = millis();
    LOG("Delta from start of read: %lu ms, UART available = %d", (end - start), a);

    return a > 0 ? a : -1;
}

int DPIClimate12::get_response(uint32_t timeout) {
    int a = waitForChar(timeout);
    if (a < 1) {
        LOG("Timeout");
        return -1;
    }

    memset(response_buffer, 0, sizeof(response_buffer));
    size_t len = m_sdi12.readBytesUntil('\n', response_buffer, BUF_LEN);

    // Strip trailing whitespace. response_buffer + len would point to the trailing null
    // so subtract one from that to get to the last character.
    char *p = response_buffer + len - 1;
    while (p > response_buffer && *p != 0 && *p < ' ') {
        *p = 0;
        p--;
    }

    // Why 1 is added here: imagine the string is "X"; this means p == msg_buf but the string
    // length is 1, not 0.
    len = p - response_buffer + 1;

    LOG("Received: [%s] [%d]", response_buffer, len);

    return len;
}

int DPIClimate12::get_response(char *buffer, int buffer_len, uint32_t timeout) {
    int i = get_response(timeout);
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
        LOG("Invalid response, expected first value to start with + or -");
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

int DPIClimate12::do_any_measure(char *cmd, bool wait_full_time, bool crc) {
    LOG("Sending command: [%s]", cmd);

    // EnviroPros seem to need a pause before they're ready to respond after they do anything.
    //delay(500);

    m_sdi12.clearBuffer();
    m_sdi12.sendCommand(cmd);
    int len = get_response();

    bool is_concurrent = false;
    int expected_len = 5; // For an M command.
    if (cmd[1] == 'C') {
        is_concurrent = true;
        expected_len = 6; // Concurrent measures use 2 digits for the number of values.
        wait_full_time = true; // Concurrent measures have no service request so must wait the full time.
    }

    // NOTE: This code doesn't work for concurrent measurements. They use 2 digits to
    // specify how many values will be returned.
    if (len != expected_len) {
        if (len < 1) {
            LOG("Timeout");
        } else {
            LOG("Invalid response to measure command: [%s]", response_buffer);
        }
        return -1;
    }

    // NOTE: This is 'tricky' code. First it uses response_buffer[4] to determine how many values
    // are expected, then it sets response_buffer[4] to zero, null-terminating the
    // time-to-wait string starting at response_buffer[1]. So the number of values must be read
    // first, then the time-to-wait parsed after that.
    int num_values = atoi(&response_buffer[4]);
    response_buffer[4] = 0;
    long delay_seconds = atol(&response_buffer[1]);

    LOG("Wait %lds for %d values", delay_seconds, num_values);

    if (delay_seconds > 0) {
        // Most sensors send a service request when they're ready to deliver their data. However, EnviroPros are not
        // ready to deliver their data after their service request. Waiting the entire delay seems to work.
        if (wait_full_time) {
            delay(delay_seconds * 1000); // ENVPRO & C4E
        }

        if ( ! is_concurrent) {
            len = get_response(delay_seconds * 1000);
            if (len < 1) {
                LOG("Did not get service request from sensor");
                return -1;
            }
        }
    }

    // EnviroPros seem to need a pause before they're ready to respond after they do anything.
    //delay(500);

    return do_data_commands(cmd[0], num_values, crc);
}

int DPIClimate12::do_measure(uint8_t address, bool wait_full_time , bool crc) {
    char *cmd = crc ? measure_crc_cmd : measure_cmd;
    cmd[0] = address;
    return do_any_measure(cmd, wait_full_time, crc);
}

int DPIClimate12::do_concurrent(uint8_t address, bool crc) {
    char *cmd = crc ? c_measure_crc_cmd : c_measure_cmd;
    cmd[0] = address;
    return do_any_measure(cmd, true, crc);
}

// TODO: Does this need the wait_full_time option to be passed in?
int DPIClimate12::do_additional_measure(uint8_t address, uint8_t additional, bool crc) {
    char *cmd = crc ? additional_measure_crc_cmd : additional_measure_cmd;
    cmd[0] = address;
    cmd[2] = additional;

    return do_any_measure(cmd, false, crc);
}

// TODO: Does this need the wait_full_time option to be passed in?
int DPIClimate12::do_additional_concurrent(uint8_t address, uint8_t additional, bool crc) {
    char *cmd = crc ? c_additional_measure_crc_cmd : c_additional_measure_cmd;
    cmd[0] = address;
    cmd[2] = additional;

    return do_any_measure(cmd, true, crc);
}

int DPIClimate12::do_data_commands(char addr, uint8_t num_values, bool crc) {
    data_cmd[0] = addr;

    int value_count = 0;
    int d_cmd_count = 0;
    data_cmd[2] = '0';

    while (value_count < num_values && d_cmd_count < 10) {
        LOG("Sending command: [%s]", data_cmd);

        m_sdi12.sendCommand(data_cmd);
        int len = get_response();
        if (len < 3) {
            LOG("Did not get expected readings from sensor");
            return -1;
        }

        if (crc && ! check_crc()) {
            LOG("Bad CRC");
            return -1;
        }

        len = parse_values(value_count);
        if (len < 1) {
            return -1;
        }

        value_count += len;
        data_cmd[2]++;

        delay(10); // ENVPRO!
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
    return do_any_measure(cmd, false, false);
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

    sensor_list.count = 0;
    size_t info_size = sizeof(sensor_info);
    memset((void *)&sensors, 0, sizeof(sensor_info) * MAX_SENSORS);
    for (char c = '0'; c <= '9'; c++) {
        info_cmd[0] = c;
        m_sdi12.clearBuffer();
        LOG("Sending command [%s]", info_cmd);
        m_sdi12.sendCommand(info_cmd);
        int len = get_response(150);
        if (len > 0) {
            size_t max = len < info_size ? len : info_size;
            memcpy(&sensors[sensor_list.count], response_buffer, max);
            sensors[sensor_list.count].null = 0;
            sensor_list.count++;
        }
    }

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
