#include "WiFi.h"
#include "WiFiClientSecure.h"
#include "esp_camera.h"
#include "secrets.h"                // Contains WIFI_SSID, WIFI_PASS, TELEGRAM_TOKEN, TELEGRAM_CHAT_ID
#include <UniversalTelegramBot.h>
#include "camera_pins.h"            // Pin definitions for your ESP32-CAM module

// Variables for Telegram photo upload (optional advanced handling)
static camera_fb_t *telegram_fb = nullptr;

// GPIO pin for flash LED (specific to AI Thinker ESP32-CAM)
#define FLASH_LED_PIN 4

// ==== UART Configuration ====
const int UART_RX_PIN = 12;  // RX pin (receives from main board TX)
const int UART_TX_PIN = 13;  // TX pin (sends to main board RX)
const int UART_BAUD_RATE = 115200;

// ==== Communication Protocol ====
const char CAPTURE_CMD = 'C';           // Command to trigger capture
const char RESPONSE_START = 'S';        // Start of image transmission
const char RESPONSE_END = 'E';          // End of image transmission
const char RESPONSE_ERROR = 'X';   
const char RESPONSE_ACK = 'A';  // Acknowledgment


// ==== Global State ====
HardwareSerial MainBoardSerial(2); // Error during capture

// --- Function declarations ---
void flashCameraLED(int flashes, int onTime = 100, int offTime = 100);
camera_fb_t* captureImage();
void sendImageOverUART(camera_fb_t *fb);
void logStatus(const char* message);
void logStatusf(const char* format, ...);

// -----------------------------------------------------
// Setup function
// Responsibility: Initialise Wi-Fi, camera, and peripherals
// -----------------------------------------------------
void setup() {
  Serial.begin(115200);
  MainBoardSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Configure flash LED pin
  pinMode(FLASH_LED_PIN, OUTPUT);

  // --- Configure camera settings ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  // Camera data pins
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  // Camera control and sync pins
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  // Power and reset pins
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Camera parameters
  config.xclk_freq_hz = 20000000;    // External clock frequency
  config.pixel_format = PIXFORMAT_JPEG;  // Output format (JPEG)
  config.frame_size = FRAMESIZE_VGA;     // Resolution (640x480)
  config.jpeg_quality = 10;              // 0-63 (lower = better quality)
  config.fb_count = 1;                   // Number of frame buffers

  // Initialise the camera
  if (esp_camera_init(&config) != ESP_OK) {
    logStatus("Camera initialisation failed!");
    return;
  }

  Serial.println("Camera initialised successfully!");
  flashCameraLED(2);  
}


// -----------------------------------------------------
// Main loop
// Responsibility: Periodically capture and send image
// -----------------------------------------------------
void loop() {
  // Check for incoming commands from main board
  if (MainBoardSerial.available() > 0) {
    char command = MainBoardSerial.read();
    
    logStatus("Received command from main board");
    if (command == CAPTURE_CMD) {
      camera_fb_t *fb = captureImage();

      // Send captured image over UART to main board
      sendImageOverUART(fb);

      // Send captured photo to Telegram via raw HTTP POST
      //sendPhotoToTelegram(fb);
    }
  }

  delay(50);
}


void flashCameraLED(int flashes, int onTime, int offTime) {
  for (int i = 0; i < flashes; i++) {
    digitalWrite(FLASH_LED_PIN, HIGH);  // Turn flash ON
    delay(onTime);                      // Keep it ON for exposure
    digitalWrite(FLASH_LED_PIN, LOW);   // Turn flash OFF
    delay(offTime);                     // Pause before next flash
  }
}

// -----------------------------------------------------
// Capture image from camera
// Responsibility: Acquire frame buffer from camera sensor
// Returns: Pointer to frame buffer, or NULL on failure
// -----------------------------------------------------
camera_fb_t* captureImage() {
  digitalWrite(FLASH_LED_PIN, HIGH);  // Turn flash ON for lighting
  delay(100);                         // Short delay for exposure

  camera_fb_t* fb = esp_camera_fb_get();  // Capture photo

  digitalWrite(FLASH_LED_PIN, LOW);   // Turn flash OFF

  if (!fb) {
    logStatus("Camera capture failed!");
    return NULL;
  }

  // Log image size and resolution
  Serial.printf("[CAM] Image captured: %d bytes, %dx%d\n", 
                fb->len, fb->width, fb->height);

  return fb;
}

// -----------------------------------------------------
// Send image over UART
// Responsibility: send captured image to main board via UART
// Parameters: fb = camera frame buffer containing JPEG image
// -----------------------------------------------------
void sendImageOverUART(camera_fb_t *fb) {
  if (!fb) {
    MainBoardSerial.write(RESPONSE_ERROR);
    logStatus("❌ No frame buffer, sending error marker");
    return;
  }

  // --- 1. Send start marker ---
  MainBoardSerial.write(RESPONSE_START);

  // --- 2. Send image size (4 bytes, little endian) ---
  uint32_t imageSize = fb->len;
  MainBoardSerial.write((uint8_t*)&imageSize, 4);

  // --- 3. Send image data in chunks ---
  const size_t CHUNK_SIZE = 128;  // safe UART buffer size
  uint8_t *bufPtr = fb->buf;

  for (uint32_t i = 0; i < imageSize; i += CHUNK_SIZE) {
    size_t bytesToSend = min(CHUNK_SIZE, imageSize - i);
    MainBoardSerial.write(bufPtr + i, bytesToSend);
    MainBoardSerial.flush(); // ensure data is sent out

    // --- Wait for ACK with timeout ---
        const unsigned long TIMEOUT_MS = 5000; // 2 second per chunk
        unsigned long startTime = millis();
        bool ackReceived = false;

        while (millis() - startTime < TIMEOUT_MS) {
            if (MainBoardSerial.available() > 0) {
                char ack = MainBoardSerial.read();
                if (ack == RESPONSE_ACK) { // ACK
                    ackReceived = true;
                    break;
                } else {
                    logStatusf("⚠ Unexpected response: %c", ack);
                }
            }
            delay(1); // small yield
        }

        if (!ackReceived) {
            logStatusf("❌ Timeout waiting for ACK for chunk starting at %lu", i);
            esp_camera_fb_return(fb);
            return;
        }
  }

  // --- 4. Send end marker ---
  MainBoardSerial.write(RESPONSE_END);

  // --- 5. Free frame buffer ---
  esp_camera_fb_return(fb);

  logStatusf("✅ Sent image (%lu bytes) to main board\n", imageSize);
}

// -----------------------------------------------------
// Logging Functions
// -----------------------------------------------------
void logStatus(const char* message) {
  Serial.print("[CAM] ");
  Serial.println(message);
}

void logStatusf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.print("[CAM] ");
  Serial.println(buffer);
}