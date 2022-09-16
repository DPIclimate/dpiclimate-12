#pragma once
#include <Arduino.h>
#include <SDI12.h>

/// \brief A union to make it simple to access the bytes of a float
/// for encoding values into messages.
typedef union {
    float value;
    uint8_t bytes[4];
} FLOAT;

constexpr int LEN_VENDOR = 8;
constexpr int LEN_MODEL = 6;
constexpr int LEN_SENSOR_VERSION = 3;
constexpr int LEN_INFO = 13;

typedef struct {
  uint8_t address;
  uint8_t sdi_version_major;
  uint8_t sdi_version_minor;
  uint8_t vendor[LEN_VENDOR];
  uint8_t model[LEN_MODEL];
  uint8_t sensor_version[LEN_SENSOR_VERSION];
  uint8_t info[LEN_INFO];
} sensor_info;

typedef struct {
    uint8_t count;
    sensor_info *sensors;
} sensor_list;

typedef struct {
    long delay;
    uint8_t num_values;
} result_info;

class DPIClimate12 {
    public:
        constexpr static int MAX_SENSORS = 10;
        constexpr static int MAX_VALUES = 32;

        DPIClimate12(SDI12 &sdi12) : m_sdi12(sdi12) {}

        void scan_bus(sensor_list &sensor_list);

        /// \brief Take a measurement from the specified sensor.
        ///
        /// The Start Measurement (aM!) and Send Data (aDx!) commands are
        /// used to start and retrieve measurements from a sensor. The values can
        /// be retrieved using get_values() or get_value(int).
        ///
        /// \param address the SDI-12 address of the sensor to measure.
        /// \return the number of values read back from the sensor.
        int do_measure(uint8_t address, bool crc = false);

        int do_additional_measure(uint8_t address, uint8_t additional, bool crc = false);

        int do_concurrent_measures(uint8_t *addresses, int num_addresses, uint8_t measure_id, result_info *results_info, bool crc = false);

        int do_data_commands(char addr, uint8_t num_values, bool crc = false);

        int do_verification(char addr);

        bool change_address(uint8_t from, uint8_t to);

        int get_response();
        int get_response(char *buffer, int buffer_len);

        /// \brief Retrieve the most recent values returned from a sensor.
        ///
        /// \return a pointer to an array of FLOATs holding the most recently
        /// received values.
        FLOAT* get_values();
        FLOAT get_value(int i);

    private:
        SDI12 &m_sdi12;

        constexpr static int BUF_LEN = 80;
        char cmd_buffer[BUF_LEN+1];
        char response_buffer[BUF_LEN+1];

        FLOAT m_values[MAX_VALUES];

        static FLOAT NaN;

        char str_val[10]; // SDI-12 spec says value strings can have a max length of 9.

        int do_any_measure(char *cmd, bool crc);

        int parse_values(int value_idx = 0);

        bool check_crc();
};
