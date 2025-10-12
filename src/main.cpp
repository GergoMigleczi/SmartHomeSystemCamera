#include <Arduino.h>
#include "esp_camera.h"

// ==== Camera Model Selection ====
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// ==== UART Configuration ====
const int UART_RX_PIN = 12;  // RX pin (receives from main board TX)
const int UART_TX_PIN = 13;  // TX pin (sends to main board RX)
const int UART_BAUD_RATE = 115200;

// ==== Communication Protocol ====
const char CAPTURE_CMD = 'C';           // Command to trigger capture
const char RESPONSE_START = 'S';        // Start of image transmission
const char RESPONSE_END = 'E';          // End of image transmission
const char RESPONSE_ERROR = 'X';        // Error during capture

// ==== Camera Configuration ====
const framesize_t IMAGE_SIZE = FRAMESIZE_VGA;  // 640x480
const int JPEG_QUALITY = 12;                    // 0-63, lower = higher quality

// ==== Global State ====
HardwareSerial MainBoardSerial(2);  // Use UART2
bool cameraInitialized = false;

// -----------------------------------------------------
// Function Declarations
// -----------------------------------------------------
bool initializeCamera();
void processIncomingCommands();
camera_fb_t* captureImage();
bool sendImage(camera_fb_t* fb);
void sendImageData(camera_fb_t* fb);
void sendErrorResponse();
void logStatus(const char* message);

// -----------------------------------------------------
// Setup
// -----------------------------------------------------
void setup() {
  // Initialize debug serial (USB)
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Camera Board Starting ===");
  
  // Initialize UART for communication with main board
  MainBoardSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  logStatus("UART initialized");
  
  // Initialize camera
  cameraInitialized = initializeCamera();
  
  if (cameraInitialized) {
    logStatus("Camera ready - Waiting for commands...");
  } else {
    logStatus("Camera initialization FAILED!");
  }
}

// -----------------------------------------------------
// Main Loop
// -----------------------------------------------------
void loop() {
  // Check for incoming commands from main board
  processIncomingCommands();
  
  delay(10);  // Small delay to prevent tight loop
}

// -----------------------------------------------------
// Initialize camera with configuration
// -----------------------------------------------------
bool initializeCamera() {
  camera_config_t config;
  
  // Pin configuration (from camera_pins.h based on selected model)
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // Compressed format
  
  // Frame configuration
  if (psramFound()) {
    config.frame_size = IMAGE_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count = 2;  // Use 2 frame buffers for better performance
    logStatus("PSRAM found - Using dual frame buffers");
  } else {
    config.frame_size = FRAMESIZE_SVGA;  // Smaller size without PSRAM
    config.jpeg_quality = 12;
    config.fb_count = 1;
    logStatus("No PSRAM - Using single frame buffer");
  }
  
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  // Additional sensor settings for better image quality
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_whitebal(s, 1);       // Enable white balance
    s->set_awb_gain(s, 1);       // Enable auto white balance gain
    s->set_wb_mode(s, 0);        // Auto white balance mode
  }
  
  return true;
}

// -----------------------------------------------------
// Listen for and process commands from main board
// -----------------------------------------------------
void processIncomingCommands() {
  if (MainBoardSerial.available() > 0) {
    char command = MainBoardSerial.read();
    
    if (command == CAPTURE_CMD) {
      logStatus("Capture command received");
      
      if (cameraInitialized) {
        // Capture image
        camera_fb_t* fb = captureImage();
        
        if (fb != NULL) {
          // Send image to main board
          bool success = sendImage(fb);
          
          // Return frame buffer to driver
          esp_camera_fb_return(fb);
          
          if (success) {
            logStatus("Image captured and sent successfully");
          } else {
            logStatus("Failed to send image");
          }
        } else {
          logStatus("Failed to capture image");
        }
      } else {
        logStatus("Camera not initialized - Cannot capture");
        sendErrorResponse();
      }
    }
  }
}

// -----------------------------------------------------
// Capture image from camera
// Responsibility: Acquire frame buffer from camera sensor
// Returns: Pointer to frame buffer, or NULL on failure
// -----------------------------------------------------
camera_fb_t* captureImage() {
  camera_fb_t* fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("Camera capture failed");
    sendErrorResponse();
    return NULL;
  }
  
  Serial.printf("Image captured: %d bytes, %dx%d\n", 
                fb->len, fb->width, fb->height);
  
  return fb;
}

// -----------------------------------------------------
// Send captured image to main board
// Responsibility: Transmit frame buffer data via UART
// Returns: true if successful, false otherwise
// -----------------------------------------------------
bool sendImage(camera_fb_t* fb) {
  if (fb == NULL) {
    return false;
  }
  
  // Send image data with protocol markers
  sendImageData(fb);
  
  return true;
}

// -----------------------------------------------------
// Send image data with protocol markers
// -----------------------------------------------------
void sendImageData(camera_fb_t* fb) {
  // Send start marker
  MainBoardSerial.write(RESPONSE_START);
  
  // Send image size (4 bytes, little-endian)
  uint32_t imageSize = fb->len;
  MainBoardSerial.write((uint8_t)(imageSize & 0xFF));
  MainBoardSerial.write((uint8_t)((imageSize >> 8) & 0xFF));
  MainBoardSerial.write((uint8_t)((imageSize >> 16) & 0xFF));
  MainBoardSerial.write((uint8_t)((imageSize >> 24) & 0xFF));
  
  // Send image data in chunks
  const size_t CHUNK_SIZE = 512;
  size_t bytesSent = 0;
  
  while (bytesSent < fb->len) {
    size_t bytesToSend = min(CHUNK_SIZE, fb->len - bytesSent);
    size_t written = MainBoardSerial.write(fb->buf + bytesSent, bytesToSend);
    bytesSent += written;
    
    // Small delay to prevent buffer overflow
    delay(5);
  }
  
  // Send end marker
  MainBoardSerial.write(RESPONSE_END);
  
  Serial.printf("Sent %d bytes to main board\n", bytesSent);
}

// -----------------------------------------------------
// Send error response to main board
// -----------------------------------------------------
void sendErrorResponse() {
  MainBoardSerial.write(RESPONSE_ERROR);
  Serial.println("Error response sent to main board");
}

// -----------------------------------------------------
// Log status message to debug serial
// -----------------------------------------------------
void logStatus(const char* message) {
  Serial.print("[CAM] ");
  Serial.println(message);
}