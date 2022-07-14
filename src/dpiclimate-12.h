#pragma once
#include <Arduino.h>
#include <SDI12.h>

// Enables serial debugging (verbose output)
#define DPI12_USE_SERIAL

// Serial1 works with standby mode.
#ifdef DPI12_USE_SERIAL
    #define serial Serial
#endif

/// \brief A union to make it simple to access the bytes of a float
/// for encoding values into messages.
typedef union {
    float value;
    uint8_t bytes[4];
} DPI12_FLOAT;

typedef enum {
	DPI12_OK = 0,
	DPI12_ERR = -1,
	DPI12_CRC_ERR = -2
} DPI12_status;

constexpr static int8_t LEN_VENDOR = 8;
constexpr static int8_t LEN_MODEL = 6;
constexpr static int8_t LEN_SENSOR_VERSION = 3;
constexpr static int8_t LEN_INFO = 13;
constexpr static int16_t DPI12_DELAY = 500;

typedef struct {
  uint8_t address;
  uint8_t sdi_version_major;
  uint8_t sdi_version_minor;
  uint8_t vendor[LEN_VENDOR];
  uint8_t model[LEN_MODEL];
  uint8_t sensor_version[LEN_SENSOR_VERSION];
  uint8_t info[LEN_INFO];
} DPI12_sensor_info;

typedef struct {
    uint8_t count;
    DPI12_sensor_info *sensors;
} DPI12_sensor_list;

class DPI12 {
    public:

        void begin(int8_t sdi12_data_pin, DPI12_sensor_list &sensor_list);

        void scan_bus(DPI12_sensor_list &sensor_list);

        /// \brief Take a measurement from the specified sensor.
        ///
        /// The Start Measurement (aM!) and Send Data (aDx!) commands are
        /// used to start and retrieve measurements from a sensor.
        ///
        /// \param address the SDI-12 address of the sensor to measure.
        /// \return the number of values read back from the sensor.
        int8_t do_command(const char* cmd, bool crc = false);
        int8_t do_measure(uint8_t address, bool crc = false);

        int8_t do_additional_measure(uint8_t address,
                                     uint8_t additional,
                                     bool crc = false);

        int8_t do_concurrent_measures(const uint8_t *addresses,
                                      int num_addresses,
                                      bool crc = false);

        int8_t do_verify(uint8_t address);

        bool change_address(uint8_t from, uint8_t to);

        int8_t get_response();

        /// \brief Retrieve the most recent values returned from a sensor.
        ///
        /// \return a pointer to an array of FLOATs holding the most recently
        /// received values.
        DPI12_FLOAT* get_values();
        DPI12_FLOAT get_value(int8_t i);

        #ifdef DPI12_USE_SERIAL
            static void device_information(DPI12_sensor_info* sensor);
        #endif

    private:
        // Set delay based on experince with different sensors
        static void do_delay();

		// Max number of connected sensors
		constexpr static const int8_t MAX_SENSORS = 10;
		// List of connected sensors
        DPI12_sensor_info DPI12_sensors[MAX_SENSORS];

		// Read buffer length (max length of 35 or 75)
		constexpr static int8_t BUF_LEN = 80;
		// Response buffer
		char res_buf[BUF_LEN+1];

		// Max number of data values
        constexpr static int8_t MAX_VALUES = 32;
		// Array of enum FLOAT to hold data values
        DPI12_FLOAT m_values[MAX_VALUES];

        int8_t do_data_command(const uint8_t address,
                               const int8_t num_values,
                               const bool crc);

        int8_t parse_values();

        bool check_crc();
};

