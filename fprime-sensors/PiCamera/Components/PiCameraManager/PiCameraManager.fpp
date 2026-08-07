module PiCamera {
    @ Camera component with Enable/Disable commands, telemetry monitor. Will report status
    active component PiCameraManager {

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for sending files via downlink
        output port sendFileOut: Svc.SendFileRequest

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port to return the value of a parameter
        param get port prmGetOut

        @ Port to set the value of a parameter
        param set port prmSetOut

        ## Pi Camera Commands: we want 2 commands "Enable Camera" and "Disable Camera"

        @ Turn the camera on
        async command ENABLE_CAMERA() opcode 0 # async means puitt this in a queue and handle it later
        # Essentially, the command calling is ferred to your component’s own thread, not run inline on the caller’s thread

        @ Turn the camera off
        async command DISABLE_CAMERA() opcode 1

        @ Take a photo
        async command CAPTURE_PHOTO() opcode 2

        @ Download latest image capture
        async command DOWNLOAD_IMAGE() opcode 3

        ## Telemetry Monitor called "Camera Enabled" which will be a Boolean
        # A telemetry monitor is basically a channel we watch in the GDS
        telemetry CAMERA_ENABLED: bool
        # this will tell us if the image is actually being captured (i.e. size > 0)
        telemetry LAST_IMAGE_SIZE: U32
        # total photos taken, verifies the camera keeps working
        telemetry IMAGES_CAPTURED: U32

        ## Events will be pushed to inform the user the camera is enabled or disabled once one of those commands are sent
        # Severity: how important the event is (will be displayed in GDS log)
        @ Event: camera was enabled
        event CAMERA_ENABLED_EVENT \
        severity activity low \ 
        format "Camera enabled" # the text the GDS show


        @ Event: camera was disabled
        event CAMERA_DISABLED_EVENT \
        severity activity low \
        format "Camera disabled"

        @ Event: photo captured successfully
        event PHOTO_CAPTURED_SUCCESS(
            filename: string size 80 @< Name of captured image file, max 80 characters
            filesize: U32 @< Size of captured image in bytes
        ) \
        severity activity high \ 
        format "Photo captured successfully: {} ({} bytes)" @< placeholders get replaces with filename and filesize

        @ Event: photo capture failed
        event PHOTO_CAPTURE_FAILED(
            error: string size 120 @< Error message explaining what went wrong (max 120 characters)
        ) \
        severity warning high \
        format "Photo capture failed: {}"

        @ Event: image download started
        event IMAGE_DOWNLOAD_STARTED(
            filename: string size 80 @< Name of file being downloaded
        ) \
        severity activity low \
        format "Started download of image: {}"

        @ Event: image download completed successfully
        event IMAGE_DOWNLOAD_SUCCESS(
            filename: string size 80 @< Name of downloaded file
        ) \
        severity activity high \
        format "Image download completed: {}"

        # note: activity severity is informational, warning severity denotes an actual error
        @ Event: image download failed
        event IMAGE_DOWNLOAD_FAILED(
            filename: string size 80 @< Name of file that failed
            error: string size 120 @< Error message
        ) \
        severity warning high \
        format "Image download failed for {}: {}"


    }
}
