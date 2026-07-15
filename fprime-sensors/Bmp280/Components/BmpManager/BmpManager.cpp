// ======================================================================
// \title  BmpManager.cpp
// \author aborjigin & Generated
// \brief  cpp file for BmpManager component implementation class
// ======================================================================

#include "fprime-sensors/Bmp280/Components/BmpManager/BmpManager.hpp"
#include <cmath>

namespace Bmp280 {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

BmpManager ::BmpManager(const char* const compName) : BmpManagerComponentBase(compName), m_state(RESET) {}

BmpManager ::~BmpManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void BmpManager ::parameterUpdated(FwPrmIdType id) {
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    switch (id) {
        case PARAMID_PRESSURE_OVERSAMPLING: {
            const PressureOversampling oversampling = this->paramGet_PRESSURE_OVERSAMPLING(isValid);
            FW_ASSERT(isValid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(isValid));
            this->log_ACTIVITY_HI_PressureOversamplingUpdated(oversampling);
            this->m_state = CONFIGURE;
            break;
        }
        case PARAMID_TEMPERATURE_OVERSAMPLING: {
            const TemperatureOversampling oversampling = this->paramGet_TEMPERATURE_OVERSAMPLING(isValid);
            FW_ASSERT(isValid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(isValid));
            this->log_ACTIVITY_HI_TemperatureOversamplingUpdated(oversampling);
            this->m_state = CONFIGURE;
            break;
        }
        case PARAMID_SEA_LEVEL_PRESSURE:
            // Passive parameter, used in altitude calculation only
            break;
        default:
            FW_ASSERT(0, static_cast<FwAssertArgType>(id));
            break;
    }
}

void BmpManager ::run_handler(FwIndexType portNum, U32 context) {
    switch (this->m_state) {
        case RESET:
            // If reset is successful, move to STARTUP_DELAY state
            if (this->reset()) {
                this->m_state = STARTUP_DELAY;
                this->m_startupCounter = STARTUP_DELAY_CYCLES;
            } else {
                this->log_WARNING_HI_DeviceFailure();
            }
            break;

        case STARTUP_DELAY:
            this->m_startupCounter--;
            if (this->m_startupCounter <= 0) {
                this->m_state = CHIP_ID_CHECK;
            }
            break;

        case CHIP_ID_CHECK: {
            U8 chip_id = 0;
            bool success = this->read_chip_id(chip_id); 
            if (success && chip_id == CHIP_ID_VALUE) {
                this->m_state = CALIBRATION_READ;
            } else {
                this->m_state = RESET;
                this->log_WARNING_HI_ChipIdCheckFailure();
            }
            break;
        }
        case CALIBRATION_READ:
            if (this->read_calibration_data()) {
                this->m_state = CONFIGURE;
            } else {
                this->m_state = RESET;
                this->log_WARNING_HI_CalibrationFailure();
            }
            break;
        case CONFIGURE:
            if (this->configure_device()) {
                this->m_state = RUNNING;
            } else {
                this->m_state = RESET;
                this->log_WARNING_HI_DeviceConfigureFailure();
            }
            break;
        case RUNNING: {
            // Reset throttles for logged events
            this->log_WARNING_HI_DeviceFailure_ThrottleClear();       // Clear throttle for Device Failure event
            this->log_WARNING_HI_ChipIdCheckFailure_ThrottleClear();  // Clear throttle for Chip ID Check Failure event
            this->log_WARNING_HI_CalibrationFailure_ThrottleClear();  // Clear throttle for Data Calibration Failure
                                                                      // event
            this->log_WARNING_HI_DeviceConfigureFailure_ThrottleClear();  // Clear throttle for Device Configure Failure
                                                                          // event
            // Step 1: Check if measurement is ready
            U8 status = 0;
            if (!this->read_status(status)) {
                this->m_state = RESET;
                break;
            }

            // Check if measurement is in progress (bit 3) or if data is being updated (bit 0)
            if ((status & 0x08) || (status & 0x01)) {
                break;  // Wait for next cycle
            }

            // Step 2: Trigger a new measurement in forced mode
            if (!this->trigger_measurement()) {
                this->m_state = RESET;
                this->log_WARNING_HI_MeasurementTriggerFailure();
                break;
            }

            // Step 3: Read measurement data
            // BMP280 SPI protocol: read 6 bytes starting from PRESSURE_MSB_REGISTER
            U8 spiData[MEASUREMENT_DATA_LENGTH + 1] = {0};
            spiData[0] = PRESSURE_MSB_REGISTER | 0x80;  // Register address with MSB=1 for read

            Fw::Buffer writeBuffer(spiData, MEASUREMENT_DATA_LENGTH + 1);
            Fw::Buffer readBuffer(spiData, MEASUREMENT_DATA_LENGTH + 1);

            if (this->spi_transfer(writeBuffer, readBuffer)) {
                // Skip first byte (register echo) and deserialize measurement data directly
                U8* dataPtr = &readBuffer.getData()[1];
                Fw::Buffer dataBuffer(dataPtr, MEASUREMENT_DATA_LENGTH);
                RawBmpData raw = this->deserialize_raw_data(dataBuffer);

                // Get sea level pressure parameter
                Fw::ParamValid paramValid;
                F32 seaLevelPressure = this->paramGet_SEA_LEVEL_PRESSURE(paramValid);
                FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

                const Bmp280Data bmpData = this->convert_raw_data(raw, this->m_calibration, seaLevelPressure);

                // Publish telemetry and push data through output port when connected
                this->tlmWrite_Reading(bmpData);
                if (this->isConnected_bmpDataPush_OutputPort(0)) {
                    this->bmpDataPush_out(0, bmpData);
                }

                this->log_WARNING_HI_MeasurementTriggerFailure_ThrottleClear();  // Clear throttle for Measurement
                                                                                 // Trigger Failure event
                this->log_WARNING_HI_DeviceReadFailure_ThrottleClear();  // Clear throttle for read Failure event

            } else {
                this->m_state = RESET;
                this->log_WARNING_HI_DeviceReadFailure();
            }
            break;
        }
        default:
            FW_ASSERT(0, this->m_state);
            break;
    }
}

// ----------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------

bool BmpManager ::reset() {
    U8 reset_sequence[] = {RESET_REGISTER & 0x7F, RESET_VALUE};  // Clear MSB for write
    Fw::Buffer writeBuffer(reset_sequence, sizeof(reset_sequence));
    Fw::Buffer readBuffer(reset_sequence, sizeof(reset_sequence));
    bool success = this->spi_transfer(writeBuffer, readBuffer);
    return success;
}

bool BmpManager ::read_chip_id(U8& id) {
    U8 spiData[2] = {CHIP_ID_REGISTER | 0x80, 0x00};  // Ensure MSB is set for read

    Fw::Buffer writeBuffer(spiData, 2);
    Fw::Buffer readBuffer(spiData, 2);  // Reuse same buffer for read

    bool success = this->spi_transfer(writeBuffer, readBuffer);

    if (success) {
        id = readBuffer.getData()[1];
    } else {
        id = 0;
    }
    return success;
}

bool BmpManager ::read_status(U8& status) {
    U8 spiData[2] = {STATUS_REGISTER | 0x80, 0x00};  // Ensure MSB is set for read

    Fw::Buffer writeBuffer(spiData, 2);
    Fw::Buffer readBuffer(spiData, 2);

    bool success = this->spi_transfer(writeBuffer, readBuffer);

    if (success) {
        status = readBuffer.getData()[1];
    } else {
        status = 0;
    }

    return success;
}

bool BmpManager ::read_calibration_data() {
    U8 spiData[CALIB_DATA_LENGTH + 1];
    spiData[0] = CALIB_DATA_REGISTER | 0x80;  // Register address with MSB=1 for read
    for (U32 i = 1; i <= CALIB_DATA_LENGTH; i++) {
        spiData[i] = 0x00;  // Dummy bytes that will be filled with calibration data
    }

    Fw::Buffer writeBuffer(spiData, CALIB_DATA_LENGTH + 1);
    Fw::Buffer readBuffer(spiData, CALIB_DATA_LENGTH + 1);

    if (this->spi_transfer(writeBuffer, readBuffer)) {
        U8* data = &readBuffer.getData()[1];

        m_calibration.dig_T1 = (static_cast<U16>(data[1]) << 8) | data[0];
        m_calibration.dig_T2 = (static_cast<I16>(data[3]) << 8) | data[2];
        m_calibration.dig_T3 = (static_cast<I16>(data[5]) << 8) | data[4];

        m_calibration.dig_P1 = (static_cast<U16>(data[7]) << 8) | data[6];
        m_calibration.dig_P2 = (static_cast<I16>(data[9]) << 8) | data[8];
        m_calibration.dig_P3 = (static_cast<I16>(data[11]) << 8) | data[10];
        m_calibration.dig_P4 = (static_cast<I16>(data[13]) << 8) | data[12];
        m_calibration.dig_P5 = (static_cast<I16>(data[15]) << 8) | data[14];
        m_calibration.dig_P6 = (static_cast<I16>(data[17]) << 8) | data[16];
        m_calibration.dig_P7 = (static_cast<I16>(data[19]) << 8) | data[18];
        m_calibration.dig_P8 = (static_cast<I16>(data[21]) << 8) | data[20];
        m_calibration.dig_P9 = (static_cast<I16>(data[23]) << 8) | data[22];

        return true;
    }
    return false;
}

bool BmpManager ::configure_device() {
    Fw::ParamValid paramValid;
    const PressureOversampling pressureOversampling = this->paramGet_PRESSURE_OVERSAMPLING(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
    const TemperatureOversampling temperatureOversampling = this->paramGet_TEMPERATURE_OVERSAMPLING(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

    // CTRL_MEAS register (0xF4) format:
    // bits 7:5 = osrs_t (temperature oversampling)
    // bits 4:2 = osrs_p (pressure oversampling)
    // bits 1:0 = mode (00=sleep, 01=forced, 11=normal)
    U8 ctrl_meas_value = (static_cast<U8>(temperatureOversampling) << 5) |
                         (static_cast<U8>(pressureOversampling) << 2) | 0x00;  // Start in sleep mode first

    U8 config_sequence[] = {CTRL_MEAS_REGISTER & 0x7F, ctrl_meas_value};  // Clear MSB for write
    Fw::Buffer writeBuffer(config_sequence, sizeof(config_sequence));
    Fw::Buffer readBuffer(config_sequence, sizeof(config_sequence));

    bool success = this->spi_transfer(writeBuffer, readBuffer);

    if (success) {
        U8 config_value = 0x00;                                         // No IIR filter, 4-wire SPI
        U8 config_sequence[] = {CONFIG_REGISTER & 0x7F, config_value};  // Clear MSB for write
        Fw::Buffer configWriteBuffer(config_sequence, sizeof(config_sequence));
        Fw::Buffer configReadBuffer(config_sequence, sizeof(config_sequence));

        success = this->spi_transfer(configWriteBuffer, configReadBuffer);
    }

    return success;
}

bool BmpManager ::trigger_measurement() {
    Fw::ParamValid paramValid;
    const PressureOversampling pressureOversampling = this->paramGet_PRESSURE_OVERSAMPLING(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
    const TemperatureOversampling temperatureOversampling = this->paramGet_TEMPERATURE_OVERSAMPLING(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

    // Set device to forced mode to trigger a measurement
    U8 ctrl_meas_value =
        (static_cast<U8>(temperatureOversampling) << 5) | (static_cast<U8>(pressureOversampling) << 2) | FORCED_MODE;

    // For SPI writes, register address MSB must be 0
    U8 config_sequence[] = {CTRL_MEAS_REGISTER & 0x7F, ctrl_meas_value};  // Clear MSB for write
    Fw::Buffer writeBuffer(config_sequence, sizeof(config_sequence));
    Fw::Buffer readBuffer(config_sequence, sizeof(config_sequence));
    return this->spi_transfer(writeBuffer, readBuffer);
}

bool BmpManager ::spi_transfer(Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) {
    // Validate buffer sizes
    FW_ASSERT(writeBuffer.getSize() != 0);

    FW_ASSERT(readBuffer.getSize() != 0);

    FW_ASSERT(writeBuffer.getSize() == readBuffer.getSize());

    // Perform the SPI transfer
    // Note: spiReadWrite_out calls the underlying SPI driver
    this->spiReadWrite_out(0, writeBuffer, readBuffer);

    // Linux SPI driver logs warnings internally if ioctl fails
    return true;
}

BmpManager::RawBmpData BmpManager ::deserialize_raw_data(Fw::Buffer& buffer) {
    auto deserializer = buffer.getDeserializer();
    RawBmpData raw;
    U8 pressure_msb, pressure_lsb, pressure_xlsb;
    U8 temperature_msb, temperature_lsb, temperature_xlsb;

    deserializer.deserializeTo(pressure_msb);
    deserializer.deserializeTo(pressure_lsb);
    deserializer.deserializeTo(pressure_xlsb);
    deserializer.deserializeTo(temperature_msb);
    deserializer.deserializeTo(temperature_lsb);
    deserializer.deserializeTo(temperature_xlsb);

    raw.pressure = (static_cast<U32>(pressure_msb) << 12) | (static_cast<U32>(pressure_lsb) << 4) |
                   (static_cast<U32>(pressure_xlsb) >> 4);

    raw.temperature = (static_cast<U32>(temperature_msb) << 12) | (static_cast<U32>(temperature_lsb) << 4) |
                      (static_cast<U32>(temperature_xlsb) >> 4);

    return raw;
}

Bmp280Data BmpManager ::convert_raw_data(const RawBmpData& raw, const CalibrationData& calib, F32 seaLevelPressure) {
    Bmp280Data bmpData;

    // BMP280 Temperature Compensation (from datasheet)
    // Returns temperature in DegC, resolution is 0.01 DegC
    I32 adc_T = static_cast<I32>(raw.temperature);
    I32 var1, var2, t_fine;
    var1 = ((((adc_T >> 3) - (static_cast<I32>(calib.dig_T1) << 1))) * static_cast<I32>(calib.dig_T2)) >> 11;
    var2 =
        (((((adc_T >> 4) - static_cast<I32>(calib.dig_T1)) * ((adc_T >> 4) - static_cast<I32>(calib.dig_T1))) >> 12) *
         static_cast<I32>(calib.dig_T3)) >>
        14;
    t_fine = var1 + var2;
    F32 temperature = static_cast<F32>((t_fine * 5 + 128) >> 8) / 100.0f;

    // BMP280 Pressure Compensation (from datasheet)
    // Returns pressure in Pa as unsigned 32 bit integer
    I32 adc_P = static_cast<I32>(raw.pressure);
    I64 var1_64, var2_64, p_64;
    var1_64 = static_cast<I64>(t_fine) - 128000;
    var2_64 = var1_64 * var1_64 * static_cast<I64>(calib.dig_P6);
    var2_64 = var2_64 + ((var1_64 * static_cast<I64>(calib.dig_P5)) << 17);
    var2_64 = var2_64 + (static_cast<I64>(calib.dig_P4) << 35);
    var1_64 = ((var1_64 * var1_64 * static_cast<I64>(calib.dig_P3)) >> 8) +
              ((var1_64 * static_cast<I64>(calib.dig_P2)) << 12);
    var1_64 = (((static_cast<I64>(1) << 47) + var1_64)) * static_cast<I64>(calib.dig_P1) >> 33;

    F32 pressure = 0.0f;
    if (var1_64 != 0) {
        p_64 = 1048576 - adc_P;
        p_64 = (((p_64 << 31) - var2_64) * 3125) / var1_64;
        var1_64 = (static_cast<I64>(calib.dig_P9) * (p_64 >> 13) * (p_64 >> 13)) >> 25;
        var2_64 = (static_cast<I64>(calib.dig_P8) * p_64) >> 19;
        p_64 = ((p_64 + var1_64 + var2_64) >> 8) + (static_cast<I64>(calib.dig_P7) << 4);
        pressure = static_cast<F32>(p_64) / 256.0f;
    }

    // Calculate altitude using barometric formula
    F32 altitude = calculate_altitude(pressure, seaLevelPressure);

    bmpData.set_pressure(pressure);
    bmpData.set_temperature(temperature);
    bmpData.set_altitude(altitude);

    return bmpData;
}

F32 BmpManager ::calculate_altitude(F32 pressure, F32 seaLevelPressure) {
    if (seaLevelPressure <= 0.0f || pressure <= 0.0f) {
        return 0.0f;  // Return 0 for invalid inputs
    }

    F32 ratio = pressure / seaLevelPressure;
    if (ratio <= 0.0f) {
        return 0.0f;
    }

    // Using standard atmosphere model constants
    // Barometric formula for altitude calculation
    // altitude = 44330 * (1 - (pressure / seaLevelPressure)^(1/5.255))
    static const F32 STANDARD_ALTITUDE_CONSTANT = 44330.0f;
    static const F32 PRESSURE_EXPONENT = 1.0f / 5.255f;

    F32 altitude = STANDARD_ALTITUDE_CONSTANT * (1.0f - pow(ratio, PRESSURE_EXPONENT));

    return altitude;
}

}  // namespace Bmp280