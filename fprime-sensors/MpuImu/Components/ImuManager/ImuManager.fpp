module MpuImu {
    @ Component emitting telemetry read from an MpuImu
    queued component ImuManager {

        @ Port for I2C bus communication
        output port busWriteRead: Drv.I2cWriteRead
        
        @ Port for I2C bus communication
        output port busWrite: Drv.I2c

        @ Port for transmitting IMU sensor data
        output port imuDataPush: MpuImu.ImuDataOut

        @ Scheduling port for reading from IMU and writing to telemetry
        sync input port run: Svc.Sched

        @ Telemetry channel for IMU data
        telemetry Reading: ImuData

        event AccelerometerRangeUpdated(
            newRange: AccelerationRange
        ) severity activity high format "Accelerometer range updated to {}"

        event GyroscopeRangeUpdated(
            newRange: GyroscopeRange
        ) severity activity high format "Gyroscope range updated to {}"

        event I2cError(
            address: U32,
            status: Drv.I2cStatus
        ) severity warning high format "I2C error on address {} with status {}" throttle 5

        @ Parameter for setting the accelerometer range
        param ACCELEROMETER_RANGE: AccelerationRange default AccelerationRange.RANGE_2G

        @ Parameter for setting the gyroscope range
        param GYROSCOPE_RANGE: GyroscopeRange default GyroscopeRange.RANGE_250DEG

        @ Command to force a RESET
        async command RESET()

        @ ImuSM instance
        state machine instance imuStateMachine: ImuStateMachine

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Command receive port
        command recv port CmdDisp

        @ Command registration port
        command reg port CmdReg

        @ Command response port
        command resp port CmdStatus

        @ Event port
        event port Log

        @ Text event port
        text event port LogText

        @ Port for getting parameters
        param get port prmGet

        @ Port for setting parameters
        param set port prmSet

    }
}
