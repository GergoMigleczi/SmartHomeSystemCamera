#include "WiFi.h"
#include "WiFiClientSecure.h"
#include "esp_camera.h"
#include "secrets.h"                // Contains WIFI_SSID, WIFI_PASS, TELEGRAM_TOKEN, TELEGRAM_CHAT_ID
#include <UniversalTelegramBot.h>
#include "camera_pins.h"            // Pin definitions for your ESP32-CAM module

// Global objects for network and Telegram bot communication
WiFiClientSecure client;            // Secure WiFi client for HTTPS
UniversalTelegramBot bot(TELEGRAM_TOKEN, client);  // Telegram bot instance

// Variables for Telegram photo upload (optional advanced handling)
static camera_fb_t *telegram_fb = nullptr;
static size_t telegram_index = 0;

// Telegram API server host
const char* telegramHost = "api.telegram.org";

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

// ==== Global State ====
HardwareSerial MainBoardSerial(2); // Error during capture

// --- Function declarations ---
camera_fb_t* captureImage();
void sendPhotoToTelegram(camera_fb_t* fb);

// -----------------------------------------------------
// Setup function
// Responsibility: Initialise Wi-Fi, camera, and peripherals
// -----------------------------------------------------
void setup() {
  Serial.begin(115200);
  MainBoardSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");

  // Configure flash LED pin
  pinMode(FLASH_LED_PIN, OUTPUT);

  // Skip SSL certificate verification (Telegram uses HTTPS)
  client.setInsecure(); 

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
    Serial.println("Camera initialisation failed!");
    return;
  }

  Serial.println("Camera initialised successfully!");
  digitalWrite(FLASH_LED_PIN, HIGH);  // Turn flash ON for lighting
  delay(100);                         // Short delay for exposure
  digitalWrite(FLASH_LED_PIN, LOW);
  delay(100);  
  digitalWrite(FLASH_LED_PIN, HIGH);  // Turn flash ON for lighting
  delay(100);                         // Short delay for exposure
  digitalWrite(FLASH_LED_PIN, LOW);
}


// -----------------------------------------------------
// Main loop
// Responsibility: Periodically capture and send image
// -----------------------------------------------------
void loop() {
  // Check for incoming commands from main board
  if (MainBoardSerial.available() > 0) {
    char command = MainBoardSerial.read();
    
    if (command == CAPTURE_CMD) {
      camera_fb_t *fb = captureImage();

      // Send captured photo to Telegram via raw HTTP POST
      sendPhotoToTelegram(fb);
    }
  }

  delay(50);
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
    Serial.println("Camera capture failed!");
    return NULL;
  }

  // Log image size and resolution
  Serial.printf("Image captured: %d bytes, %dx%d\n", 
                fb->len, fb->width, fb->height);

  return fb;
}


// -----------------------------------------------------
// Send photo to Telegram using raw HTTPS POST
// Responsibility: Upload JPEG image to Telegram Bot API
// Parameters: fb = camera frame buffer containing JPEG image
// -----------------------------------------------------
void sendPhotoToTelegram(camera_fb_t* fb) {

  if (!fb) {
    Serial.println("Camera capture failed - no frame buffer");
    return;
  }

  // Connect to Telegram API server
  if (!client.connect(telegramHost, 443)) {
    Serial.println("Connection to Telegram failed!");
    esp_camera_fb_return(fb);
    return;
  }

  // Define multipart/form-data boundary (used for file upload)
  String boundary = "ESP32CAMBOUNDARY";

  // Build request headers and form body
  String head = "--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" +
                TELEGRAM_CHAT_ID +
                "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--" + boundary + "--\r\n";

  // Calculate total content length
  uint32_t imageLen = fb->len;
  uint32_t totalLen = head.length() + imageLen + tail.length();

  // --- Send HTTP POST request ---
  client.printf("POST /bot%s/sendPhoto HTTP/1.1\r\n", TELEGRAM_TOKEN);
  client.printf("Host: %s\r\n", telegramHost);
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.printf("Content-Length: %d\r\n\r\n", totalLen);

  // Write form-data body
  client.print(head);
  client.write(fb->buf, imageLen);   // Send raw JPEG data
  client.print(tail);

  // Release the frame buffer memory
  esp_camera_fb_return(fb);

  // --- Read Telegram server response (optional) ---
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break; // End of headers
  }

  String response = client.readString();
  Serial.println("Telegram response:");
  Serial.println(response);
}
