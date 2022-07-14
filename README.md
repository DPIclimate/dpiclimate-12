# dpiclimate-12

A high level wrapper for the SDI12 library.

## Installation

Until this is published as an Arduino library, this repo must be cloned
so it is sitting under the Arduino libraries directory.

## Example

```cpp
#include "dpiclimate-12.h"

#define SDI12_DATA_PIN 12 // SDI-12 input/output data pin

DPI12 dpi12;
DPI12_sensor_list sensor_list; // For device information

void setup(){
    Serial.begin(9600);
    // Initialise and get list of connected sensors
    dpi12.begin(SDI12_DATA_PIN, sensor_list);
}

void loop(){
    // Loop through connected sensors
    for(uint8_t i = 0; i < sensor_list.count; i++){
        // Print out sensor information
        DPI12::device_information(&sensor_list.sensors[i]);
        // Take a measurement
        int8_t n_values = dpi12.do_measure(sensor_list.sensors[i].address);
        // Check that data has been captured
        if(n_values > 1){
            // Get values from buffer
            DPI12_FLOAT* values = dpi12.get_values();
            // Print out the values 
            for(int8_t x = 1; x < n_values; x++){
                serial.println(values[x].value);
            }
        }
    }
    delay(15000);
}
```
