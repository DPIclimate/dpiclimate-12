#pragma once
#include <Arduino.h>
#include <SDI12.h>

/// \brief A union to make it simple to access the bytes of a float
/// for encoding values into messages.
typedef union {
    float value;
    uint8_t bytes[4];
} FLOAT;

class DPIClimate12 {
    public:
        DPIClimate12(SDI12 &sdi12) : m_sdi12(sdi12) {}

        /// \brief Take a measurement from the specified sensor.
        ///
        /// The Start Measurement (aM!) and Send Data (aDx!) commands are
        /// used to start and retrieve measurements from a sensor.
        ///
        /// \param address the SDI-12 address of the sensor to measure.
        /// \return the number of values read back from the sensor.
        int do_measure(uint8_t address);

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

        constexpr static int MAX_VALUES = 32;
        FLOAT m_values[MAX_VALUES];
        char str_val[10]; // SDI-12 spec says value strings can have a max length of 9.

        int get_response();
        int parse_values();
};
