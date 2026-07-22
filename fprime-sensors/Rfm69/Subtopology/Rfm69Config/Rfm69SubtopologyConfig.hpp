// ======================================================================
// \title  Rfm69SubtopologyConfig.hpp
// \brief required header file containing the required definitions for the subtopology autocoder
//
// ======================================================================
#ifndef Rfm69_Rfm69SubtopologyConfig_hpp
#define Rfm69_Rfm69SubtopologyConfig_hpp

struct Rfm69Device {
    int device;  // SPI bus number (e.g., 0 for SPI bus 0)
    int select;  // SPI chip select pin (e.g., 0 for CS0)
};

#endif  // Rfm69_Rfm69SubtopologyConfig_hpp
