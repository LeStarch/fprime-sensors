// ======================================================================
// \title  PiCameraManager.cpp
// \author madisonw
// \brief  cpp file for PiCameraManager component implementation class
// ======================================================================

#include "PiCameraManager/PiCameraManager.hpp"
#include <Os/FileSystem.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <cstdlib>  // For system() calls

namespace PiCamera {

// Static constants
// IMAGE_DIR is where all capture images will be stored on the Pi
const char* PiCameraManager::IMAGE_DIR = "/tmp/fprime_images";
// resolution settings: we're using 1920 x 1080 (HD)
const U32 PiCameraManager::IMAGE_WIDTH = 1920;
const U32 PiCameraManager::IMAGE_HEIGHT = 1080;

// Component construction and destruction

// Constructor: called when a component is created (default method)
PiCameraManager::PiCameraManager(const char* const compName) 
    : PiCameraManagerComponentBase(compName),
      m_cameraEnabled(false), // start with the camera disabled
      m_imageCounter(0), // the # of images starts at 0
      m_lastImageSize(0) { // last image size starts at 0
    
    // Create image directory (if it doesn't exist)
    // 0755 = permissions: can read, write, execute
    mkdir(IMAGE_DIR, 0755);
}

// Destructor: when a component is destroyed
PiCameraManager::~PiCameraManager() {
    if (m_cameraEnabled) {
        shutdownCamera();
    }
}

// Command Handlers

// turns the camera on
void PiCameraManager::ENABLE_CAMERA_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // confirm the camera is on
    if (m_cameraEnabled) {
        // Emit the camera enabled event
        this->log_ACTIVITY_LO_CAMERA_ENABLED_EVENT();
        
        // Update telemetry channel (reports specific parameters)
        this->tlmWrite_CAMERA_ENABLED(true);
        
        // Send command response
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
        return;
    }

    // Initialises the camera (connects to the initializeCamera method below)
    if (initializeCamera()) {
        // it's enabled until proven otherwise
        m_cameraEnabled = true;
        
        // Emit the camera enabled event
        this->log_ACTIVITY_LO_CAMERA_ENABLED_EVENT();
        
        // Update telemetry channel
        this->tlmWrite_CAMERA_ENABLED(true);
        
        // Send command response
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else { // if it fails, print the error message
        Fw::LogStringArg errorStr("Failed to initialize camera");
        // send the event to GDS with high (HI) priority warning
        this->log_WARNING_HI_PHOTO_CAPTURE_FAILED(errorStr);
        // tell F Prime it has failed too
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

// Turns the camera off
void PiCameraManager::DISABLE_CAMERA_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // if camera is currently enabled, shut it down
    if (m_cameraEnabled) {
        shutdownCamera();
        m_cameraEnabled = false;
    }
    
    // Emit the camera disabled event
    this->log_ACTIVITY_LO_CAMERA_DISABLED_EVENT();
    
    // Update telemetry channel
    this->tlmWrite_CAMERA_ENABLED(false);
    
    // Send command response
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// Takes a photo
void PiCameraManager::CAPTURE_PHOTO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // Add actual photo capture logic here

    // if the camera is disabled, we cannot take a photo
    // hence, send a warning event and error response to F'
    if (!m_cameraEnabled) {
        Fw::LogStringArg errorStr("Camera not enabled");
        this->log_WARNING_HI_PHOTO_CAPTURE_FAILED(errorStr);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    // generate a unique filename with a timestamp
    std::string filename = generateImageFilename();
    std::string errorMsg;
    
    // Try to capture real image using system commands (C++11 compatible)
    // C++11 compatible means we're using system() calls as opposed to the C++17 libcamera API
    // if the photo capture has succeeded, update all tracking variables
    if (captureImageWithSystemCommand(filename, errorMsg)) {
        U32 filesize = getFileSize(filename); // get the actual file size
        m_lastCapturedFile = filename; // remember this file
        m_lastImageSize = filesize; // remember the size
        m_imageCounter++; // increment total image count

        // Update telemetry - sends image size and counter data to GDS
        this->tlmWrite_LAST_IMAGE_SIZE(filesize);
        this->tlmWrite_IMAGES_CAPTURED(m_imageCounter);

        // Emit photo captured event - tells user if capture was successful
        Fw::LogStringArg filenameStr(filename.c_str()); // convert to F Prime string type
        this->log_ACTIVITY_HI_PHOTO_CAPTURED_SUCCESS(filenameStr, filesize);
        
        // Send successful command response
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
        // if the capture fails, report the error
        Fw::LogStringArg errorStr(errorMsg.c_str()); // convert error message to f prime string
        this->log_WARNING_HI_PHOTO_CAPTURE_FAILED(errorStr);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void PiCameraManager::DOWNLOAD_IMAGE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // Check if we have a captured image
    if (m_lastCapturedFile.empty() || !fileExists(m_lastCapturedFile)) {
        Fw::LogStringArg errorStr("No image available");
        Fw::LogStringArg emptyFilename("");
        this->log_WARNING_HI_IMAGE_DOWNLOAD_FAILED(emptyFilename, errorStr);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    
    // Extract filename for display
    size_t lastSlash = m_lastCapturedFile.find_last_of('/');
    std::string baseFilename = (lastSlash != std::string::npos) ? 
                               m_lastCapturedFile.substr(lastSlash + 1) : 
                               m_lastCapturedFile;
    
    // Log download events
    Fw::LogStringArg filenameStr(baseFilename.c_str());
    this->log_ACTIVITY_LO_IMAGE_DOWNLOAD_STARTED(filenameStr);
    this->log_ACTIVITY_HI_IMAGE_DOWNLOAD_SUCCESS(filenameStr);
    
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// Private helper methods - C++11 COMPATIBLE IMPLEMENTATION

bool PiCameraManager::initializeCamera() {
    // Check if Pi camera is available using system command
    // libcamera-hello is a Pi-specific utility and --list-camera will list all cameras
    // this will return true if at least one camera is successfully found
    // 2>/dev/null suppresses error messages, grep -q makes it silent (only returns true/false)
    int result = system("libcamera-hello --list-cameras >/dev/null 2>&1"); // system() executes shell command
    return (result == 0); // returns true upon camera being detected
}

void PiCameraManager::shutdownCamera() {
    // for system command approach, everything is cleaned and handled automatically
}

bool PiCameraManager::captureImageWithSystemCommand(const std::string& filename, std::string& errorMsg) {
    // Use libcamera-still command (available on Pi OS)
    // build command string with desired parameters
    std::stringstream cmd;
    // this is basically building a shell command for Pi's licamera tool
    cmd << "libcamera-still -o " << filename // output filename
        << " --width " << IMAGE_WIDTH // set image widfth
        << " --height " << IMAGE_HEIGHT // set image height
        << " --nopreview --timeout 1 2>/dev/null"; // no preview window, 1 second timeout, suppress errors (for cleaner logs)

    // execute command and get return code
    // converts from string stream (like StringBuilder in Java) --> C++ string (cmd/str()) --> char array (.c_str() - like char[] to make a word)
    // this is because system() coommand needs to read char arrays
    int result = system(cmd.str().c_str());
    
    // check if the command succeeded AND the file was created
    if (result == 0 && fileExists(filename)) { //libcamera-still returns - when photo is captured successfully (built in functionality)
        return true;
    } else {
        errorMsg = "Camera capture command failed";
        return false;
    }
}

std::string PiCameraManager::generateImageFilename() {
    // Use simpler timestamp format: img20250514.jpg
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << IMAGE_DIR << "/img"
       << std::put_time(std::localtime(&time_t), "%Y%m%d")
       << ".jpg";
    return ss.str();
}

U32 PiCameraManager::getFileSize(const std::string& filepath) {
    FwSizeType size = 0;
    Os::FileSystem::Status status = Os::FileSystem::getFileSize(filepath.c_str(), size);
        
    // Return size if successful, otherwise return 0
    return (status == Os::FileSystem::OP_OK) ? static_cast<U32>(size) : 0;
}

bool PiCameraManager::fileExists(const std::string& filepath) {
    // use stat() to check if file exists (we don't need any file info for this)
    struct stat buffer;
    // retun 0 if the file exists and you can accesible it, else non-zero
    return (stat(filepath.c_str(), &buffer) == 0);
}

} // namespace PiCamera