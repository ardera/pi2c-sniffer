## GPIO I2C Sniffer for Raspberry Pi
Does exactly what it sounds like. Uses 2 GPIO pins of your Raspberry Pi to sniff I2C bus traffic.

Requirements: python3, pigpiod, pigpio library for python3

## Running
    
    sudo apt install python3 pigpio python3-pigpio git
    sudo pigpiod -s 2
    git clone --depth 1 https://github.com/ardera/pi2c-sniffer
    python3 ./pi2c-sniffer/pi2c-sniffer.py

The program is currently set up to listen on pins 23 & 24 and interpret the traffic as communication between VideoCore and the touchscreen controller of the Offical 7 inch touchscreen (an FT5426). It measures the time between touchscreen polls by the VideoCore, and when the touchscreen reports back touch data, it also measures how often the FT5426 reports fresh vs old touch data (by calculating the average interval between 'successful', fresh touch data polls)
