#include <dpiclimate-12.h>

// Initialise EnviroDIY/SDI12 object
static SDI12 m_sdi12;

// Command definitions where a represents the device address (integer between
// 0 - 9) and d represents a nested dataset (an integer between 0 - 9).
// Note CRC means carriage return check.

// Measure aM!
static char measure_cmd[] = { 0, 'M', '!', 0 };
// Measure with CRC aMC!
static char measure_crc_cmd[] = { 0, 'M', 'C', '!', 0 };
// Additional measurements aMd!
static char additional_measure_cmd[] = { 0, 'M', 0, '!', 0 };
// Additional measurements with CRC aMCd!
static char additional_measure_crc_cmd[] = { 0, 'M', 'C', 0, '!', 0 };
// Get data aD0!
static char data_cmd[] = { 0, 'D', '0', '!', 0 };
// Attention command a! (not supported on all devices)
static char attn_cmd[] = { 0, '!', 0};
// Get device information aI!
static char info_cmd[] = { 0, 'I', '!', 0};
// Concurrent measurement aC!
static char c_measure_cmd[] = { 0, 'C', '!', 0 };
// Concurrent measurement with CRC aCC!
static char c_measure_crc_cmd[] = { 0, 'C', 'C', '!', 0 };
// Verification command aV!
static char verify_cmd[] = { 0, 'V', '!', 0 };
// Change address of device aAb! (changing from address a -> b)
static char change_address_cmd[] = { 0, 'A', 1, '!', 0 };

/**
 * Initialise DPI12 library and underlying SDI12 library.
 *
 * @breif Takes the pin number of the SDI-12 data line and a list of sensors to
 * populate upon initialisation.
 *
 * @note This must be called in void setup()
 *
 * @todo Reassign devices to different addresses so they can be plugged in one
 * at a time
 *
 * @param sdi12_data_pin The SDI-12 data line input/output pin.
 * @param sensor_list Sensors object to populate with detected sensors.
 */
void DPI12::begin(int8_t sdi12_data_pin, DPI12_sensor_list &sensor_list){
    m_sdi12.setDataPin(sdi12_data_pin);
    m_sdi12.begin();
    scan_bus(sensor_list);
}

/**
 * Identifies devices with different addresses on the bus.
 *
 * @breif Used to populate a list of connected sensors as they are identified
 * on the bus. This function uses the attention command (see attn_cmd).
 *
 * @warn SDI12 devices (should) default to address 0. If multiple sensors are
 * connected at a single point in time they may have conflicting addresses.
 * Users should reassign the address of each SDI12 device using
 * change_address(a, b)
 *
 * @param sensor_list Sensors object to populate.
 */
void DPI12::scan_bus(DPI12_sensor_list &sensor_list) {

    #ifdef DPI12_USE_SERIAL
        serial.println("Getting sensor list.");
    #endif

    int s_count = 0;
    uint32_t info_size = sizeof(DPI12_sensor_info);
    memset((void *)&DPI12_sensors, 0,
           sizeof(DPI12_sensor_info) * MAX_SENSORS);
    for (char c = '0'; c <= '9'; c++) { // TODO change back to '9'
    #ifdef DPI12_USE_SERIAL
            serial.print("Checking address: "); serial.println(c);
    #endif
        attn_cmd[0] = c;
        m_sdi12.clearBuffer();
        m_sdi12.sendCommand(attn_cmd);
        int8_t len = get_response();
        if (len == 1) {
            do_delay();
            info_cmd[0] = c;
            m_sdi12.clearBuffer();
            m_sdi12.sendCommand(info_cmd);
            len = get_response();
            if (len > 0) {
                uint32_t max = (uint32_t)len < info_size ? len : info_size;
                memcpy(&DPI12_sensors[s_count], res_buf, max);
                #ifdef DPI12_USE_SERIAL
                    device_information(&DPI12_sensors[s_count]);
                #endif
                s_count++;
            }
        }

        do_delay();
    }

    sensor_list.count = s_count;
    sensor_list.sensors = DPI12_sensors;
}


/**
 * Preform SDI-12 command.
 *
 * @breif Preforms a provided SDI-12 command. Requires the command to have a
 * valid device address (mostly handled by this library). If the command is a
 * measure command this function will also preform the necessary delay before
 * preforming a get data command.
 *
 * @param cmd
 * @param crc
 * @return
 */
int8_t DPI12::do_command(const char* cmd, const bool crc) {

    #ifdef DPI12_USE_SERIAL
        serial.print("Sending command: ");
        serial.write(cmd, strlen(cmd)); serial.println();
    #endif

    do_delay();

    m_sdi12.clearBuffer();
    m_sdi12.sendCommand(cmd);
    int8_t len = get_response();
    if (len != 5) {
        #ifdef DPI12_USE_SERIAL
            if (len < 1) {
                serial.println("timeout");
            } else {
                serial.print("Invalid response to measure command: ");
                serial.write(res_buf, len); serial.println();
            }
        #endif
        return DPI12_ERR;
    }

    uint8_t num_values = res_buf[4] - '0';
    res_buf[4] = 0;
    char* ptr;
    uint32_t delay_seconds = strtol(&res_buf[1], &ptr, 10);

    #ifdef DPI12_USE_SERIAL
        serial.print("Wait ");
        serial.print(delay_seconds); serial.print(" sec, for ");
        serial.print(num_values); serial.println(" values");
    #endif

    if (delay_seconds > 0) {
        uint32_t start_ms = millis();
        uint32_t end_ms = start_ms + (delay_seconds * 1000);
        uint32_t now_ms = start_ms;
        len = 0;
        while (now_ms < end_ms && len < 1) {
            delay(200);
            now_ms = millis();

            // Wait for the service request from the sensor. It must send a
            // service request if it specifies a wait period > 0
            // (SDI-12 spec v1.4, pg 13, ss 4.4.6).
            len = get_response();
        }

        if (len != 1) {
            #ifdef DPI12_USE_SERIAL
                serial.println("Invalid service request from sensor");
            #endif
            return DPI12_ERR;
        }
    }

    do_delay();

    return do_data_command(cmd[0], (int8_t)num_values, crc);
}

int8_t DPI12::get_response() {

    #ifdef DPI12_USE_SERIAL
        serial.print("Waiting for response");
        serial.flush();
    #endif

    int8_t a = 0; // Characters in buffer (-1 if error or overflow)
    int8_t n_tries = 0;
    const int8_t MAX_RETRIES = 50;
    while (a < 1) {
        delay(10);
        a = (int8_t)m_sdi12.available(); // Get number of characters in buffer
        n_tries++;

        // Wait up to 500ms ~(MAX_TRIES * delay(10)). Some DPI12_sensors are
        // not ready to send even if they have sent the attention command after
        // a measure.
        if (a < 1 && n_tries > MAX_RETRIES) {
            #ifdef DPI12_USE_SERIAL
                serial.println(" - timeout");
                serial.flush();
            #endif
            return DPI12_ERR;
        }
    }

    int8_t i = 0;
    n_tries = 0;
    memset(res_buf, 0, sizeof(res_buf));
    while ((uint8_t)i < sizeof(res_buf) && n_tries < 10) {

        // Check if there are characters in the buffer. If not add slight delay
        // and retry.
        if (m_sdi12.available() < 1) {
            delay(10);
            n_tries++;
            continue;
        }

        int ch = m_sdi12.read();
        if (ch == 0x0a || ch == 0x0d || (ch >= 32 && ch <= 126)) {
            // Reset the n_tries counter whenever a valid character is received.
            n_tries = 0;
            if(ch == '\n'){
                break;
            } else if (ch >= 32) {
                res_buf[i++] = (char)ch;
                res_buf[i] = 0;
            }
        } else {
            // Assume any invalid character means the response is malformed 
            // and return an error.
            #ifdef DPI12_USE_SERIAL
                serial.print(" - invalid character ");
                serial.print(ch, HEX);
                serial.print(" ");
                serial.flush();
            #endif
            return DPI12_ERR;
        }
    }

    #ifdef DPI12_USE_SERIAL
        serial.print(", received: ");
        serial.write(res_buf, i);
        serial.print(", length: ");
        serial.print(i); serial.println();
    #endif

    return i;
}


int8_t DPI12::do_measure(uint8_t address, const bool crc){
    char *cmd = crc ? measure_crc_cmd : measure_cmd;
    cmd[0] = (char)address;
    return do_command(cmd, crc);
}


/**
 * Parse up to MAX_VALUES floats from res_buf.
 *
 * res_buf is assumed to contain a valid response from D command, so
 * an SDI-12 address, then a series of floats delimited by + and -.
 * res_buf is assumed to be null-terminated, 
 * not terminated with <CR><LF>.
 *
 * This function returns any successfully parsed floats when an invalid
 * character is encountered.
 *
 * Returns the number of floats parsed. The values are found in 
 * values[0 .. n-1].
 */
int8_t DPI12::parse_values() {

    // Assume the first character is an SDI-12 address.
    if (res_buf[1] != '+' && res_buf[1] != '-') {
        #ifdef DPI12_USE_SERIAL
            serial.println("Error, expected first value to start with + or -");
        #endif
        return DPI12_ERR;
    }

    // Start looking at the 2nd character of the response because the first
    // character is the sensor address.
    int8_t i = 1;

    char str_val[10]; // String value buffer (max length of 9)
    int8_t val_count = 0; // Number of values parsed
    int8_t sv_idx = 1;
    while (i < (strlen(res_buf) + 1)) {
        char ch = res_buf[i++];
        if (ch != '+' && ch != '-' && ch != 0) {
            if (ch == '.' || (ch >= '0' && ch <= '9')) {
                str_val[sv_idx++] = ch;
                str_val[sv_idx] = 0;
            }
            // The response may contain a 3 character CRC after the values,
            // this code will soak them up without adding them into str_val,
            // and when the res_buf null terminator is reached the last value
            // will be decoded, below.
            continue;
        }

        char* ptr;
        m_values[val_count++].value = (float)strtod(str_val, &ptr);
        str_val[0] = ch;
        sv_idx = 1;
    }

    return val_count;
}


int8_t DPI12::do_additional_measure(const uint8_t address,
                                    const uint8_t additional,
                                    const bool crc) {
    char *cmd = crc ? additional_measure_crc_cmd : additional_measure_cmd;
    cmd[0] = (const char)address;
    cmd[2] = (const char)additional;
    return do_command(cmd, crc);
}


int8_t DPI12::do_data_command(const uint8_t address,
                              const int8_t num_values,
                              const bool crc) {
    data_cmd[0] = (char)address;

    int8_t n_values = 0; // Number of values returned
    int8_t d_cmd_count = 0; // Index (d) in dataset commands (e.g. 0Dd!)

    while (n_values < num_values && d_cmd_count < 10) {

        // Add index to data command
        data_cmd[2] = (char)('0' + d_cmd_count++);

        #ifdef DPI12_USE_SERIAL
            serial.print("Sending command: ");
            serial.write(data_cmd, strlen(data_cmd)); serial.println();
        #endif

        m_sdi12.sendCommand(data_cmd);
        int8_t len = get_response();

        if (len < 3) {
            #ifdef DPI12_USE_SERIAL
                serial.println("Unexpected values from sensor.");
            #endif
            return DPI12_ERR;
        }

        if (crc && !check_crc()) {
            #ifdef DPI12_USE_SERIAL
                serial.println("Bad CRC");
            #endif
            return DPI12_CRC_ERR;
        }

        n_values += parse_values();
    }

    return n_values;
}


int8_t DPI12::do_concurrent_measures(const uint8_t *addresses,
                                     const int num_addresses,
                                     const bool crc) {

    char *cmd = crc ? c_measure_crc_cmd : c_measure_cmd;

    // Determine the maximum length of measurement delay from a sensor
    uint32_t max_delay = 0;
    for (int i = 0; i < num_addresses; i++) {
        cmd[0] = (char)addresses[i];

        #ifdef DPI12_USE_SERIAL
            serial.print("Sending command: ");
            serial.write(cmd, strlen(cmd)); serial.println();
        #endif

        m_sdi12.clearBuffer();
        m_sdi12.sendCommand(cmd);
        int8_t len = get_response();
        if (len != 6) {
            #ifdef DPI12_USE_SERIAL
                if (len < 1) {
                    serial.println("timeout");
                } else {
                    serial.print("Invalid response to measure command: ");
                    serial.write(res_buf, len); serial.println();
                }
            #endif
            return DPI12_ERR;
        }

        res_buf[4] = 0;
        char* ptr;
        uint32_t delay_seconds = strtol(&res_buf[1], &ptr, 10);
        if (delay_seconds > max_delay) {
            max_delay = delay_seconds;
        }

        #ifdef DPI12_USE_SERIAL
            uint8_t num_values = (((uint8_t)res_buf[4] - '0') * 10) +
                                 (uint8_t)res_buf[5] - '0';
            serial.print("Wait "); serial.print(delay_seconds);
            serial.print(", for "); serial.print(num_values);
            serial.println(" values");
        #endif
    }

    // No attention responses for concurrent measurements, just wait for the
    // longest sensor.
    delay(max_delay * 1000);

    for (int i = 0; i < num_addresses; i++) {
        data_cmd[0] = (char)addresses[i];
        data_cmd[2] = '0';
        #ifdef DPI12_USE_SERIAL
            serial.print("Sending command: ");
            serial.write(data_cmd, strlen(data_cmd)); serial.println();
        #endif
        m_sdi12.sendCommand(data_cmd);
        int8_t len = get_response();
        if (len < 2) {
            #ifdef DPI12_USE_SERIAL
                serial.println("Did not get expected readings from sensor");
            #endif
            return DPI12_ERR;
        }

        if (crc && ! check_crc()) {
            #ifdef DPI12_USE_SERIAL
                serial.println("Bad CRC");
            #endif
            return DPI12_CRC_ERR;
        }
    }

    return DPI12_OK;
}


int8_t DPI12::do_verify(const uint8_t address) {
    verify_cmd[0] = (char)address;
    return do_command(verify_cmd, false);
}


DPI12_FLOAT* DPI12::get_values() {
    return &m_values[0];
}


DPI12_FLOAT DPI12::get_value(const int8_t i) {
    if (i < MAX_VALUES) {
        return m_values[i];
    }
    return m_values[0];
}


bool DPI12::change_address(const uint8_t from, const uint8_t to) {
    change_address_cmd[0] = (char)from;
    change_address_cmd[2] = (char)to;
    m_sdi12.clearBuffer();
    m_sdi12.sendCommand(change_address_cmd);
    int8_t len = get_response();
    return len == 1 && res_buf[0] == to;
}


bool DPI12::check_crc() {
    // Assume the response buffer has the CRC at the end.
    size_t resp_len = strlen(res_buf) - 3;

    uint16_t crc = 0;
    int8_t i = 0;
    while(res_buf[i] != '\0' && i < (int8_t)(sizeof(res_buf) - 3)){
        crc ^= (uint16_t)res_buf[i] & 0xff;
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
    ascii_crc[0] = (char)(0x40 | (crc >> 12));
    ascii_crc[1] = (char)(0x40 | ((crc >> 6) & 0x3f));
    ascii_crc[2] = (char)(0x40 | (crc & 0x3f));

    return ascii_crc[0] == res_buf[resp_len++] &&
           ascii_crc[1] == res_buf[resp_len++] &&
           ascii_crc[2] == res_buf[resp_len] ;
}


void DPI12::do_delay(){
    // Some SDI12 sensors seem to need a pause before they're ready to respond
    // after they do anything.
    delay(DPI12_DELAY);
}


#ifdef DPI12_USE_SERIAL
    void DPI12::device_information(DPI12_sensor_info* sensor){

        serial.println("Device found:");
        serial.print("Address:\t"); serial.println((char)sensor->address);

        serial.print("SDI12 Version:\t");
        serial.print((char)sensor->sdi_version_major); serial.print(".");
        serial.println((char)sensor->sdi_version_minor);

        serial.print("Vendor:\t\t");
        for(int8_t i = 0; i < LEN_VENDOR; i++){
            char c = (char)sensor->vendor[i];
            if(c == '0') break;
            serial.print(c);
        } serial.println();

        serial.print("Model:\t\t");
        for(int8_t i = 0; i < LEN_MODEL; i++){
            char c = (char)sensor->model[i];
            if(c == '0') break;
            serial.print(c);
        } serial.println();

        serial.print("Sensor Version:\t");
        for(int8_t i = 0; i < LEN_SENSOR_VERSION; i++){
            char c = (char)sensor->sensor_version[i];
            if(c == '0') break;
            serial.print(c);
        } serial.println("\n~~~~~~~~~~~~");
    }
#endif


