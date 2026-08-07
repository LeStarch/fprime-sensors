// ======================================================================
// \title  PiCameraManager.hpp
// \author madisonw
// \brief  hpp file for PiCameraManager component implementation class
// ======================================================================
#ifndef PiCamera_PiCameraManager_HPP
#define PiCamera_PiCameraManager_HPP
#include "PiCameraManager/PiCameraManagerComponentAc.hpp"
#include <string>  
namespace PiCamera {
class PiCameraManager final : public PiCameraManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------
    //! Construct PiCameraManager object
    PiCameraManager(const char* const compName  //!< The component name
    );
    //! Destroy PiCameraManager object
    ~PiCameraManager();
  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------
    //! Handler implementation for command ENABLE_CAMERA
    //!
    //! Turn the camera on
    void ENABLE_CAMERA_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                  U32 cmdSeq            //!< The command sequence number
                                  ) override;
    //! Handler implementation for command DISABLE_CAMERA
    //!
    //! Turn the camera off
    void DISABLE_CAMERA_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                   U32 cmdSeq            //!< The command sequence number
                                   ) override;
    //! Handler implementation for command CAPTURE_PHOTO
    //!
    //! Take a photo
    void CAPTURE_PHOTO_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                  U32 cmdSeq             //!< The command sequence number
                                  ) override;
    //! Handler implementation for command DOWNLOAD_IMAGE
    //!
    //! Download latest image capture
    void DOWNLOAD_IMAGE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                   U32 cmdSeq            //!< The command sequence number
                                   ) override;

    // ----------------------------------------------------------------------
    // Private Member Variables - the things we will need to remember (hence store in variables)
    // ----------------------------------------------------------------------
    bool m_cameraEnabled;                //!< Camera enabled state (is it on or off?)
    U32 m_imageCounter;                  //!< Counter for captured images
    std::string m_lastCapturedFile;      //!< Path to last captured image
    U32 m_lastImageSize;                 //!< Size of last captured image in bytes
    static const char* IMAGE_DIR;        //!< Directory to store images
    static const U32 IMAGE_WIDTH;        //!< Image width in pixels (so the camera knows what resolution to capture)
    static const U32 IMAGE_HEIGHT;       //!< Image height in pixels

    // ----------------------------------------------------------------------
    // Private helper methods
    // ----------------------------------------------------------------------
    //! Initialize camera system (true/false for success/failure)
    // connects pi camera hardware through system commands, called when ENABLE_CAMERA = true
    bool initializeCamera();
    //! Shutdown camera system, releases hardware resources, called when DISABLE_CAMERA = true
    void shutdownCamera();
    //! Capture image using system command approach (works with actual Pi camera)
    bool captureImageWithSystemCommand(const std::string& filename, std::string& errorMsg);
    // copy image form local machine to pi before download
    bool copyImageToLocalAndOpen(const std::string& sourceFile, std::string& errorMsg);
    //! Generate unique filename for image
    std::string generateImageFilename();
    //! Get file size in bytes
    U32 getFileSize(const std::string& filepath);
    //! Check if file exists
    bool fileExists(const std::string& filepath);
    
};
}  // namespace PiCamera
#endif