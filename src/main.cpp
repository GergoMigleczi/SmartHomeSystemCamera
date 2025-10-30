/**
 * @file        main.cpp
 * @brief       Handles image capture, serial communication,
 *              and LED control for the ESP32-CAM device.
 *
 * @details     This firmware runs on the ESP32-CAM and:
 *              - Receives UART commands from the main board
 *              - Captures images and sends them in chunks
 *              - Uses ACK-based transmission for reliability
 *              - Controls onboard flash LED for illumination
 *
 * @author      Gergo Migleczi
 * @date        2025-10-30
 * @version     v1.0
 * 
 * @hardware    ESP32-CAM Module
 * @framework   Arduino
 *
 *  * @dependencies
 *   - esp_camera.h
 * 
 */

// =====================================================
// ==================== Includes =======================
// =====================================================

#include <Arduino.h>
#include "esp_camera.h"
#include "secrets.h"
#include "camera_pins.h"

// =====================================================
// =================== Configuration ===================
// =====================================================
const bool DEBUG = true;                   // Enable or disable debug logging
const int FLASH_LED_PIN = 4;               // GPIO pin for flash LED
const int UART_RX_PIN = 12;                // UART receive pin
const int UART_TX_PIN = 13;                // UART transmit pin
const int UART_BAUD_RATE = 115200;         // UART baud rate
const size_t UART_BUFFER_SIZE = 1024;      // UART buffer size for RX/TX
const size_t CHUNK_SIZE = 1024;            // Image transmission chunk size (bytes)
const unsigned long ACK_TIMEOUT_MS = 15000;// Timeout for ACK response from main board (ms)

// =====================================================
// ================= Protocol Commands =================
// =====================================================
const char CAPTURE_CMD = 'C';              // Command: Capture image
const char BLINK_CMD = 'B';                // Command: Blink LED
const char RESPONSE_START = 'S';           // Response: Start of image transmission
const char RESPONSE_END = 'E';             // Response: End of image transmission
const char RESPONSE_ERROR = 'X';           // Response: Error marker
const char RESPONSE_ACK = 'A';             // Response: Acknowledge chunk receipt

// =====================================================
// ================== Camera Settings ==================
// =====================================================
const int XCLK_FREQ_HZ = 20000000;         // External clock frequency for camera
const framesize_t FRAME_SIZE = FRAMESIZE_VGA; // Image resolution
const int JPEG_QUALITY = 30;               // JPEG compression quality (lower = higher compression)

// =====================================================
// =================== Global Objects ==================
// =====================================================
HardwareSerial MainBoardSerial(2);         // UART channel for main board communication

// =====================================================
// =============== Function Declarations ===============
// =====================================================
void initializeSerial();
void initializeCamera();
void processCommand(char cmd);
void handleCaptureCommand();
void handleBlinkCommand();
camera_fb_t* captureImage();
bool sendImageData(camera_fb_t *fb);
bool sendImageChunk(uint8_t *data, size_t offset, size_t size);
bool waitForAck();
void setFlash(bool state);
void flashLED(int flashes, int onTime = 100, int offTime = 100);
void logDebug(const char* message);
void logDebugf(const char* format, ...);

// =====================================================
// ====================== Setup ========================
// =====================================================

/**
 * @brief  Initialises serial communication and camera.
 */
void setup() {
  Serial.begin(115200);                    
  pinMode(FLASH_LED_PIN, OUTPUT);          
  
  initializeSerial();                      
  initializeCamera();                      
  
  // --- Indicate successful setup ---
  flashLED(2);                             
}

// =====================================================
// ======================= Loop ========================
// =====================================================

/**
 * @brief  Main control loop for reading commands and processing them.
 */
void loop() {
  if (MainBoardSerial.available() > 0) {
    char cmd = MainBoardSerial.read();     
    processCommand(cmd);                   
  }
  delay(50);                              
}

// =====================================================
// ============== System Initialisation ================
// =====================================================

/**
 * @brief  Configure UART communication between ESP32-CAM and main board.
 */
void initializeSerial() {
  MainBoardSerial.setRxBufferSize(UART_BUFFER_SIZE);
  MainBoardSerial.setTxBufferSize(UART_BUFFER_SIZE);
  MainBoardSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

/**
 * @brief  Initialise the camera module with predefined parameters.
 */
void initializeCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  // --- Pin configuration ---
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  // --- Camera parameters ---
  config.xclk_freq_hz = XCLK_FREQ_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 1;
  
  // --- Initialise camera and report status ---
  if (esp_camera_init(&config) != ESP_OK) {
    logDebug("Camera initialization failed!");
    return;
  }
  
  logDebug("Camera initialized successfully!");
}

// =====================================================
// ================ Command Processing =================
// =====================================================
/**
 * @brief  Parse and execute commands received from main board.
 */
void processCommand(char cmd) {
  logDebugf("Received command: %c", cmd);
  
  switch (cmd) {
    case CAPTURE_CMD:
      // --- Capture and send image ---
      handleCaptureCommand();              
      break;

    case BLINK_CMD:
      // --- Flash LED ---
      handleBlinkCommand();                 
      break;

    default:
      logDebugf("Unknown command: %c", cmd);
  }
}

/**
 * @brief  Handle capture command from main board.
 *         Captures an image and sends it via UART.
 */
void handleCaptureCommand() {
  // --- Attempt image capture ---
  camera_fb_t *fb = captureImage();        
  
  if (fb) {
    // --- Transmit image if capture successful ---
    sendImageData(fb);                     
  } else {
    // --- Notify error to main board ---
    MainBoardSerial.write(RESPONSE_ERROR); 
    logDebug("Capture failed, sent error marker");
  }
}

/**
 * @brief  Handle blink command from main board.
 *         Flashes LED a few times for visual feedback.
 */
void handleBlinkCommand() {
  flashLED(3, 300, 200);
}

// =====================================================
// ================= Camera Operations =================
// =====================================================
/**
 * @brief  Discards a specified number of frames to stabilise exposure.
 */
void flushCameraBuffer(int frameCount) {
  for (int i = 0; i < frameCount; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
  }
}

/**
 * @brief  Capture an image using the camera module with flash assistance.
 * @return Pointer to captured frame buffer (must be released).
 */
camera_fb_t* captureImage() {
  const int FRAMES_TO_FLUSH = 10;
  const int FLASH_STABILIZATION_DELAY_MS = 150;
  
  setFlash(true);       
  // --- Discard old frames ---           
  flushCameraBuffer(FRAMES_TO_FLUSH);
  // --- Wait for exposure to stabilise ---     
  delay(FLASH_STABILIZATION_DELAY_MS);     
  
  // --- Capture frame ---
  camera_fb_t* fb = esp_camera_fb_get();   
  setFlash(false);                         
  
  if (!fb) {
    logDebug("Camera capture failed!");
    return NULL;
  }
  
  logDebugf("Image captured: %d bytes, %dx%d", fb->len, fb->width, fb->height);
  return fb;
}

// =====================================================
// ================= Image Transmission ================
// =====================================================
/**
 * @brief  Send an image to main board in multiple chunks with ACK verification.
 * @param  fb Pointer to frame buffer containing image data.
 * @return True if successful, false if transmission failed.
 */
bool sendImageData(camera_fb_t *fb) {
  if (!fb) return false;
  
  // --- Send start marker and total size ---
  MainBoardSerial.write(RESPONSE_START);
  uint32_t imageSize = fb->len;
  MainBoardSerial.write((uint8_t*)&imageSize, 4);
  
  // --- Send image data in fixed-size chunks ---
  bool success = true;
  for (uint32_t offset = 0; offset < imageSize; offset += CHUNK_SIZE) {
    size_t chunkSize = min(CHUNK_SIZE, imageSize - offset);
    
    if (!sendImageChunk(fb->buf, offset, chunkSize)) {
      success = false;
      break;
    }
  }
  
  // --- Finalise transmission ---
  if (success) {
    MainBoardSerial.write(RESPONSE_END);   
    logDebugf("Sent image (%lu bytes) to main board", imageSize);
  }
  
  // --- Release frame buffer memory ---
  esp_camera_fb_return(fb);                
  return success;
}

/**
 * @brief  Send a single image data chunk and wait for acknowledgement.
 * @param  data   Pointer to image data buffer.
 * @param  offset Start offset in buffer.
 * @param  size   Number of bytes to send.
 * @return True if ACK received, false on timeout.
 */
bool sendImageChunk(uint8_t *data, size_t offset, size_t size) {
  MainBoardSerial.write(data + offset, size);
  MainBoardSerial.flush();
  
  if (!waitForAck()) {
    logDebugf("Timeout waiting for ACK at offset %lu", offset);
    return false;
  }
  
  return true;
}

/**
 * @brief  Wait for ACK response from main board to confirm chunk receipt.
 * @return True if ACK received before timeout, false otherwise.
 */
bool waitForAck() {
  unsigned long startTime = millis();
  
  while (millis() - startTime < ACK_TIMEOUT_MS) {
    if (MainBoardSerial.available() > 0) {
      char response = MainBoardSerial.read();
      
      if (response == RESPONSE_ACK) {
        return true;
      }
      
      logDebugf("Unexpected response: %c", response);
    }
    delay(1); // Prevent tight loop
  }
  
  return false; // Timed out
}

// =====================================================
// ==================== LED Control ====================
// =====================================================
/**
 * @brief  Control the state of the flash LED.
 * @param  state True to turn ON, false to turn OFF.
 */
void setFlash(bool state) {
  digitalWrite(FLASH_LED_PIN, state ? HIGH : LOW);
}

/**
 * @brief  Blink the LED a specified number of times.
 * @param  flashes Number of flashes.
 * @param  onTime  LED ON duration in ms.
 * @param  offTime LED OFF duration in ms.
 */
void flashLED(int flashes, int onTime, int offTime) {
  for (int i = 0; i < flashes; i++) {
    setFlash(true);
    delay(onTime);
    setFlash(false);
    if (i < flashes - 1) delay(offTime);
  }
}

// =====================================================
// ==================== Debugging ======================
// =====================================================

/**
 * @brief  Prints a simple debug message to Serial if DEBUG mode is enabled.
 * @param  message Message to print.
 */
void logDebug(const char* message) {
  if (!DEBUG) return;
  Serial.print("[CAM] ");
  Serial.println(message);
}

/**
 * @brief  Prints a formatted debug message to Serial (printf-style).
 * @param  format Printf-style format string.
 * @param  ... Arguments to format.
 */
void logDebugf(const char* format, ...) {
  if (!DEBUG) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.print("[CAM] ");
  Serial.println(buffer);
}
