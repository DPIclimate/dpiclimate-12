#pragma once
#include <Arduino.h>
#include <SDI12.h>

/// \brief A union to make it simple to access the bytes of a float
/// for encoding values into messages.
typedef union {
    double value;
    uint8_t bytes[sizeof(double)];
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
  // Allows the struct to be printed as a string.
  uint8_t null;
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
        /// \param wait_full_time if true, wait the full time specified by the sensor
        ///            response even if the service request arrives early. Some sensors
        ///            send the service request but are not really ready to response yet.
        /// \param crc if true, ask for a CRC in the response.
        /// \return the number of values read back from the sensor.
        int do_measure(uint8_t address, bool wait_full_time = false, bool crc = false);

        /// \brief Take a concurrent measurement from the specified sensor.
        ///
        /// The Start Concurrent Measurement (aC!) and Send Data (aDx!) commands are
        /// used to start and retrieve measurements from a sensor. The values can
        /// be retrieved using get_values() or get_value(int).
        ///
        /// \param address the SDI-12 address of the sensor to measure.
        /// \param crc if true, ask for a CRC in the response.
        /// \return the number of values read back from the sensor.
        int do_concurrent(uint8_t address, bool crc = false);

        int do_additional_measure(uint8_t address, uint8_t additional, bool crc = false);

        int do_additional_concurrent(uint8_t address, uint8_t additional, bool crc = false);

        int do_concurrent_measures(uint8_t *addresses, int num_addresses, uint8_t measure_id, result_info *results_info, bool crc = false);

        int do_data_commands(char addr, uint8_t num_values, bool crc = false);

        int do_verification(char addr);

        bool change_address(uint8_t from, uint8_t to);

        int get_response(uint32_t timeout = 1000);
        int get_response(char *buffer, int buffer_len, uint32_t timeout = 1000);

        /// \brief Retrieve the most recent values returned from a sensor.
        ///
        /// \return a pointer to an array of FLOATs holding the most recently
        /// received values.
        FLOAT* get_values();
        FLOAT get_value(int i);

        /// \brief Copy the vendor id for the given sensor into buffer.
        ///
        /// buffer must be at least LEN_VENDOR+1 bytes long.

        /// \param buffer The buffer to copy the vendor id into. It will be null-terminated
        ///               and have trailing spaces stripped.
        /// \param sensor_idx The index of the sensor with sensor_list.sensors. This is not the
        ///                   SDI-12 address of the sensor.
        /// \param sensors A sensor_list structure as filled in by scan_bus.
        static void get_vendor(char *buffer, const size_t sensor_idx, const sensor_list &sensors);

        /// \brief Copy the model id for the given sensor into buffer.
        ///
        /// buffer must be at least LEN_MODEL+1 bytes long.

        /// \param buffer The buffer to copy the model id into. It will be null-terminated
        ///               and have trailing spaces stripped.
        /// \param sensor_idx The index of the sensor with sensor_list.sensors. This is not the
        ///                   SDI-12 address of the sensor.
        /// \param sensors A sensor_list structure as filled in by scan_bus.
        static void get_model(char *buffer, const size_t sensor_idx, const sensor_list &sensors);

    private:
        SDI12 &m_sdi12;

        constexpr static int BUF_LEN = 80;
        char response_buffer[BUF_LEN+1];

        FLOAT m_values[MAX_VALUES];

        static FLOAT NaN;

        char str_val[10]; // SDI-12 spec says value strings can have a max length of 9.

        int waitForChar(uint32_t timeout = 1000);
        int do_any_measure(char *cmd, bool wait_full_time, bool crc);

        int parse_values(int value_idx = 0);

        bool check_crc();
};
