// Serial1 works with standby mode.
#define serial Serial1

//#define USE_SERIAL

#include <cmath>
#include <dpiclimate-12.h>

char measure_cmd[] = { 0, 'M', '!', 0 };
char data_cmd[] = { 0, 'D', '0', '!', 0 };

int DPIClimate12::get_response() {
    #ifdef USE_SERIAL
        serial.print("Waiting for response");
    #endif

    int i = 0;
    while (m_sdi12.available() < 1) {
        delay(100);
        i++;

        if (i > 10) {
            #ifdef USE_SERIAL
                serial.println(" - timeout");
            #endif
            return -1;
        }
    }

    i = 0;
    int ch;
    memset(response_buffer, 0, sizeof(response_buffer));
    while (i < BUF_LEN) {
        if (m_sdi12.available() < 1) {
            delay(10);
            continue;
        }

        ch = m_sdi12.read();
        if (ch >= ' ') {
            response_buffer[i++] = ch;
            response_buffer[i] = 0;
        }

        if (ch == '\n') {
            break;
        }
    }

    #ifdef USE_SERIAL
        serial.print(", received: "); serial.write(response_buffer, i); serial.print(", length: "); serial.print(i); serial.println();
    #endif

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
int DPIClimate12::parse_values() {
  // Assume the first character is an SDI-12 address.
  int max = strlen(response_buffer) + 1;
  int val_count = 0;

  if (response_buffer[1] != '+' && response_buffer[1] != '-') {
    return 0;
  }

  str_val[0] = response_buffer[1];
  int i = 2;
  int sv_idx = 1;
  while (i < max) {
    char ch = response_buffer[i++];
    if (ch != '+' && ch != '-' && ch  != 0) {
      if (ch == '.' || (ch >= '0' && ch <= '9')) {
        str_val[sv_idx++] = ch;
        str_val[sv_idx] = 0;
        continue;
      } else {
        #ifdef USE_SERIAL
            serial.println("Invalid float value.");
        #endif
        return val_count;
      }
    }

    m_values[val_count].value = atof(str_val);
    val_count++;
    str_val[0] = ch;
    sv_idx = 1;
  }

  return val_count;
}

int DPIClimate12::do_measure(uint8_t address) {
    measure_cmd[0] = address;

    #ifdef USE_SERIAL
        serial.print("Sending command: "); serial.write(measure_cmd, strlen(measure_cmd)); serial.println();
    #endif
    m_sdi12.clearBuffer();
    m_sdi12.sendCommand(measure_cmd);
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
        // Wait for the service request from the sensor. It must send a service request
        // if it specifies a wait period > 0 (SDI-12 spec v1.4, pg 13, ss 4.4.6).
        len = get_response();
        if (len != 1) {
            #ifdef USE_SERIAL
            serial.println("Did not get expected service request from sensor");
            #endif
            return -1;
        }
    }

    data_cmd[0] = address;
    data_cmd[2] = '0';
    #ifdef USE_SERIAL
    serial.print("Sending command: "); serial.write(data_cmd, strlen(data_cmd)); serial.println();
    #endif
    m_sdi12.sendCommand(data_cmd);
    len = get_response();
    if (len < 2) {
        #ifdef USE_SERIAL
        serial.println("Did not get expected readings from sensor");
        #endif
        return -1;
    }

    len = parse_values();
    return len;
}

FLOAT* DPIClimate12::get_values() {
  return &m_values[0];
}

FLOAT DPIClimate12::get_value(int i) {
  if (i < MAX_VALUES) {
    return m_values[i];
  }

  return m_values[0];
}
