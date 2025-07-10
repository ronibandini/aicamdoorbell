// AIBell, AI Powered Doorbell using Edge Impulse and openAI
// Roni Bandini
// MIT License, July, 10 2025
// Hardware ESP32S3 Dev Module AI Camera 1.0
// Make sure to setup USB CDC On Boot, Partition 16mb Flash 3mb 9.9, Flash 16 128, OPI PSRAM

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include "ESP_I2S.h"
#include <Person_Detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <Wire.h>
#include <UniversalTelegramBot.h>
#include <SD.h>

#define SAMPLE_RATE     (16000)
#define SD_CS_PIN       10            

I2SClass i2s_out;

#define BOT_TOKEN ""
String chatOperativo="";

String imgUrl = "http://your web site/images/aidoorbell.png";

const char* ssid = "";
const char* password = "";
const char* openai_server = "api.openai.com";
const char* openai_api_endpoint = "/v1/audio/transcriptions";
const char* openai_api_key = "";  
const char* model = "whisper-1";  
const float threshold=0.7;
#define REC_TIME 6
#define RELAY_PIN 43
#define SAMPLE_RATE (16000)
#define DATA_PIN (GPIO_NUM_39)
#define CLOCK_PIN (GPIO_NUM_38)

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

void setup();
void loop();
void processAudioWithWhisper(uint8_t *wav_buffer, size_t wav_size);
void sendAudioToOpenAI(uint8_t *audio_data, size_t audio_size);

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 5
#define Y9_GPIO_NUM 4
#define Y8_GPIO_NUM 6
#define Y7_GPIO_NUM 7
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 17
#define Y4_GPIO_NUM 21
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 16
#define VSYNC_GPIO_NUM 1
#define HREF_GPIO_NUM 2
#define PCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 8
#define SIOC_GPIO_NUM 9

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 240
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t* snapshot_buf;

void door(bool open) {
  // optional door relay
  digitalWrite(RELAY_PIN, open ? HIGH : LOW);
}

void playWavFromSD(const char *filename) {
  Serial.print("Opening file: ");
  Serial.println(filename);

  File audioFile = SD.open(filename);
  if (!audioFile) {
    Serial.println("Failed to open audio file!");
    return;
  }

  // Skip WAV header (44 bytes)
  audioFile.seek(44);

  const size_t bufferSize = 512;
  uint8_t buffer[bufferSize];

  Serial.println("Playing audio...");

  while (audioFile.available()) {
    size_t bytesRead = audioFile.read(buffer, bufferSize);
    i2s_out.write(buffer, bytesRead);
  }

  audioFile.close();
  Serial.println("Playback finished.");
}
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_240X240,
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = 0,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr);

void setup() {
    Serial.begin(115200);
    Serial.println("AI Agent Doorbell - Roni Bandini 7/2025");

    if (ei_camera_init() == false) {
        Serial.println("Cam init failed\r\n");
    } else {
        Serial.println("Cam ok\r\n");
    }

  Serial.println("Mounting SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed to initialize SD card!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Setup I2S output for speaker (MAX98357)
  i2s_out.setPins(45, 46, 42);
  if (!i2s_out.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S for playback!");
    while (1);
  }

    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print(WiFi.localIP());
    bot.sendPhoto(chatOperativo, imgUrl, "AI Doorbell Started");
    ei_sleep(2000);
}

void loop() {
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (snapshot_buf == nullptr) {
        ei_printf("ERR: snapshot buffer failed\n");
        free(snapshot_buf);
        return;
    }
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;
    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Image capture failed\r\n");
        free(snapshot_buf);
        return;
    }
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Fallo en run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        ei_printf(" %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    if (!bb_found) {
        ei_printf(" No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf(" %s: %.5f\n", result.classification[ix].label,
            result.classification[ix].value);
        if (result.classification[ix].label=="person" and result.classification[ix].value>threshold){
            ei_printf("************* Visitor detected\n");

            uint8_t *wav_buffer;
            size_t wav_size;
            I2SClass i2s;

            // Play greeting

            playWavFromSD("/ring.wav");
            playWavFromSD("/greeting.wav");

            // record
            pinMode(3, OUTPUT);
            pinMode(41, OUTPUT);

            i2s.setPinsPdmRx(CLOCK_PIN, DATA_PIN);
            if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
              Serial.println("Fallo con I2S PDM RX");
              while(1);
            }

            Serial.println("Recording...");
            digitalWrite(3, HIGH);
            wav_buffer = i2s.recordWAV(REC_TIME, &wav_size);
            digitalWrite(3, LOW);

            playWavFromSD("/moment.wav");
            Serial.println("Speech to text...");

            processAudioWithWhisper(wav_buffer, wav_size);

            free(wav_buffer);
        }
    }
#endif
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf(" anomaly score: %.3f\n", result.anomaly);
#endif
    free(snapshot_buf);
}

bool ei_camera_init(void) {
    if (is_initialised) return true;
#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Fallo camara 0x%x\n", err);
        return false;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }
    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ei_printf("Cam deinit failed\n");
        return;
    }
    is_initialised = false;
    return;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf) {
    bool do_resize = false;
    if (!is_initialised) {
        ei_printf("ERR: cam not started\r\n");
        return false;
    }
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Cam picture failed\n");
        return false;
    }
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }
    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }
    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Model not valid for sensor"
#endif

void processAudioWithWhisper(uint8_t *wav_buffer, size_t wav_size) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return;
    }
    Serial.println("Calling Whisper API...");
    sendAudioToOpenAI(wav_buffer, wav_size);
}

void sendAudioToOpenAI(uint8_t *audio_data, size_t audio_size) {
    WiFiClientSecure client;
    client.setInsecure();
    Serial.println("Connecting with OpenAI API...");
    if (!client.connect(openai_server, 443)) {
       Serial.println("Connection failed");
        return;
    }
      String boundary = "ESP32FormBoundary";
      String multipartStart = "--" + boundary + "\r\n";
      String multipartEnd = "\r\n--" + boundary + "--\r\n";
      size_t contentLength = 0;
      String modelField =
          multipartStart +
          "Content-Disposition: form-data; name=\"model\"\r\n\r\n" +
          model + "\r\n";
      contentLength += modelField.length();
      String fileHeader =
          multipartStart +
          "Content-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\n" +
          "Content-Type: audio/wav\r\n\r\n";
      contentLength += fileHeader.length();
      contentLength += audio_size + multipartEnd.length();
      client.println("POST " + String(openai_api_endpoint) + " HTTP/1.1");
      client.println("Host: " + String(openai_server));
      client.println("Authorization: Bearer " + String(openai_api_key));
      client.println("Content-Type: multipart/form-data; boundary=" + boundary);
      client.println("Content-Length: " + String(contentLength));
      client.println("Connection: close");
      client.println();
      client.print(modelField);
      client.print(fileHeader);
      const size_t chunkSize = 1024;
      size_t bytesSent = 0;
      while (bytesSent < audio_size) {
          size_t bytesToSend = (audio_size - bytesSent);
          if (bytesToSend > chunkSize) {
              bytesToSend = chunkSize;
          }
          client.write(&audio_data[bytesSent], bytesToSend);
          bytesSent += bytesToSend;
      }
      client.print(multipartEnd);
      Serial.println("Waiting...");
      while (client.connected()) {
          String line = client.readStringUntil('\n');
          if (line == "\r") {
              Serial.println("Headers ok");
              break;
          }
      }
      String response = client.readString();
      Serial.println("Got the answer:");
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, response);
      String transcription = "";
      if (!error) {
          transcription = doc["text"].as<String>();
          
          // OpenAI bug "Thanks for watching" management  
          if (transcription.length() < 5 || transcription.indexOf("thank") != -1) {
            Serial.println("No audio was detected");
            transcription="";
          }
          Serial.println("\n--------- Transcript ---------");
          Serial.println(transcription);
          Serial.println("---------------------------------\n");
          
          if (transcription == "") {
              Serial.println("Empty transcription.");
              playWavFromSD("/noname.wav");
              return;
          }

          // Sending to openAI LLM
          Serial.println("Sending to chatGPT...");
          
          client.stop();
          
          // Connect to OpenAI API again
          if (!client.connect(openai_server, 443)) {
              Serial.println("openAI Connection Failed!");
              return;
          }
          
          DynamicJsonDocument chatRequestDoc(2048);
          chatRequestDoc["model"] = "gpt-3.5-turbo";
          JsonArray messages = chatRequestDoc.createNestedArray("messages");
          
          JsonObject systemMessage = messages.createNestedObject();
          systemMessage["role"] = "system";
          systemMessage["content"] = "You are a receptionist at an office. Today, only John Smith is allowed to enter. If a visitor's name matches either of them — even with spelling variations — greet them with: welcome, push the door. For all other visitors, respond with: sorry, I cannot let you in.";
          
          JsonObject userMessage = messages.createNestedObject();
          userMessage["role"] = "user";
          userMessage["content"] = "There is someone at the door that says: \"" + transcription + "\" how do you answer?";
          
          String chatRequestJson;
          serializeJson(chatRequestDoc, chatRequestJson);
          
          client.println("POST /v1/chat/completions HTTP/1.1");
          client.println("Host: " + String(openai_server));
          client.println("Authorization: Bearer " + String(openai_api_key));
          client.println("Content-Type: application/json");
          client.println("Content-Length: " + String(chatRequestJson.length()));
          client.println("Connection: close");
          client.println();
          client.println(chatRequestJson);
          
          Serial.println("Waiting for the answer...");
          while (client.connected()) {
              String line = client.readStringUntil('\n');
              if (line == "\r") {
                  Serial.println("Headers received");
                  break;
              }
          }
          
          String llmAnswer = client.readString();
          Serial.println("Answer received");
          
          int jsonStart = llmAnswer.indexOf('{');
          if (jsonStart != -1) {
            Serial.println("Removing chars that could break json");
            llmAnswer = llmAnswer.substring(jsonStart);
          }
          DynamicJsonDocument llmAnswerDoc(8192);
          DeserializationError analysisError = deserializeJson(llmAnswerDoc, llmAnswer);
          
          if (!analysisError) {
              const char* llmAnswerChar = llmAnswerDoc["choices"][0]["message"]["content"];
              Serial.println("\n--------- Answer ---------");
              Serial.println(llmAnswerChar);        
              Serial.println("-----------------------------\n");

              // check answer to enable the door

              if (strstr(llmAnswerChar, "Welcome") != NULL) {
                Serial.println("Opening door");
                playWavFromSD("/welcome.wav");
                door(true); 
                delay(10000);
                door(false); 
              }
              else{
                playWavFromSD("/sorry.wav");
                Serial.println("\nRemote notification");
                bot.sendMessage(chatOperativo, "There is someone not announced at the door: "+String(transcription), "");
              }
              

          } else {
              Serial.print("JSON parsing failed: ");
              Serial.println(analysisError.c_str());
              Serial.println("RAW answer:");
              Serial.println(llmAnswer);
          }
      } else {
          Serial.print("JSON parsing failed: ");
          Serial.println(error.c_str());
}

}// loop