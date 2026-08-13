// ======================================================================
// \title  Rfm69SubtopologyConfig.hpp
// \brief required header file containing the required definitions for the subtopology autocoder
//
// ======================================================================
#ifndef Rfm69_Rfm69SubtopologyConfig_hpp
#define Rfm69_Rfm69SubtopologyConfig_hpp

struct Rfm69Device {
    // SPI bus number and chip select for the radio. Defaults match the
    // reference Raspberry Pi deployment (SPI0, CE1).
    int device = 0;  // SPI bus number (e.g., 0 for SPI bus 0)
    int select = 1;  // SPI chip select pin (e.g., 1 for CS1)
    // Optional hardware reset line. Defaults match the reference wiring:
    // active-high RST on BCM GPIO26 (physical pin 37).
    const char* resetGpioChip = "/dev/gpiochip0";
    int resetGpioPin = 26;
};

#endif  // Rfm69_Rfm69SubtopologyConfig_hpp
