#include "esp_camera.h"
#include "Arduino.h"
#include "WiFi.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Variables
camera_fb_t * fb = NULL;
const char* ssid = "Corbett";
const char* password = "ConnorBuddy2018";
//const char* ssid = "TP-Link_E6BE"; // Gilbert Place
//const char* password = "19572871"; // Gilbert Place
WiFiClient client;
const int timerInterval = 100;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent

  //String serverName = "10.0.0.99";  //local headset IP 
  //String serverName = "192.168.0.100";  //Gilbert Place Magic Leap IP
  String serverName = "10.0.0.40";   
  const int serverPort = 6464;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());
  

  camera_config_t config;
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
  config.pixel_format = PIXFORMAT_JPEG;
  //config.pixel_format = PIXFORMAT_YUV422;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_HD;
    config.jpeg_quality = 9;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 9;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }



  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");   
  }

}

void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    if(client.connected())
    {
      sendPhoto();
    }
    else
    {
      if (client.connect(serverName.c_str(), serverPort)) {
        Serial.println("Connection successful!");   
      }
    }
    
    previousMillis = currentMillis;
  }

}

void sendPhoto() {


  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
    uint8_t *fbBuf = fb->buf;
    int32_t fbLen = fb->len;

    String intString = String(fbLen);
    //Serial.println("Sending Image of Size " + intString + " in Variable with size " + sizeof(fbLen));
    //send length of image to server

    if(fbLen > 200000){
      esp_camera_fb_return(fb);
      return;
    }
    client.write((byte*)&fbLen, sizeof(fbLen));
  

    for (size_t n=0; n<fbLen; n=n+32768) {
      if (n+32768 < fbLen) {
        client.write(fbBuf, 32768);
        fbBuf += 32768;
      }
      else if (fbLen%32768>0) {
        size_t remainder = fbLen%32768;
        client.write(fbBuf, remainder);
      }
    }   
    esp_camera_fb_return(fb);

    //client.stop();

}

