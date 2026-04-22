/*
 * esp32_cam.ino  –  NURC 2026 ESP32-CAM Firmware
 * ================================================
 * Connects as a client to the "rc2" WiFi hotspot, then:
 *
 *  VIDEO  : Continuously captures JPEG frames and pushes them via UDP
 *           to the laptop (last sender's IP) on port 5000.
 *           No laptop-side configuration needed – the laptop's UDP socket
 *           on port 5000 auto-receives the stream.
 *
 *  CONTROL (UDP port 5002):
 *    "PING"           → reply "PONG"  (heartbeat)
 *    "FLASH_TOGGLE"   → toggle the onboard flash LED
 *    "SERVO:<pan>:<tilt>"  → move pan/tilt servos (0-180 degrees)
 *    "SERVO_RESET"    → return servos to centre (90°, 90°)
 *
 * Wiring (adjust pins to your board):
 *    Pan  servo signal  → GPIO 14
 *    Tilt servo signal  → GPIO 15
 *    Flash LED anode    → GPIO 4  (built-in on most AI-Thinker boards)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// WiFi credentials
// ---------------------------------------------------------------------------
const char* ssid     = "rc2";
const char* password = "244466666";

// ---------------------------------------------------------------------------
// Standard AI-Thinker ESP32-CAM pin map
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Hardware pins
// ---------------------------------------------------------------------------
#define FLASH_LED_PIN  4    // Built-in flash on AI-Thinker
#define PAN_SERVO_PIN  14   // Pan servo signal wire
#define TILT_SERVO_PIN 15   // Tilt servo signal wire

// Servo centre positions (degrees)
#define PAN_CENTER  90
#define TILT_CENTER 90

// ---------------------------------------------------------------------------
// Network configuration
// ---------------------------------------------------------------------------
#define CMD_PORT   5002   // Receive control commands from laptop
#define VIDEO_PORT 5000   // Send JPEG frames to laptop on this port

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
WiFiUDP cmdUdp;    // Listens for control commands
WiFiUDP vidUdp;    // Sends video frames

char     incomingPacket[512];
IPAddress laptopIP;       // Updated on every received packet
bool     laptopKnown = false;

bool     flashOn = false;
Servo    panServo;
Servo    tiltServo;
int      currentPan  = PAN_CENTER;
int      currentTilt = TILT_CENTER;

// ---------------------------------------------------------------------------
// Camera initialisation
// ---------------------------------------------------------------------------
void initCamera() {
    camera_config_t config;
    config.ledc_channel  = LEDC_CHANNEL_0;
    config.ledc_timer    = LEDC_TIMER_0;
    config.pin_d0        = Y2_GPIO_NUM;
    config.pin_d1        = Y3_GPIO_NUM;
    config.pin_d2        = Y4_GPIO_NUM;
    config.pin_d3        = Y5_GPIO_NUM;
    config.pin_d4        = Y6_GPIO_NUM;
    config.pin_d5        = Y7_GPIO_NUM;
    config.pin_d6        = Y8_GPIO_NUM;
    config.pin_d7        = Y9_GPIO_NUM;
    config.pin_xclk      = XCLK_GPIO_NUM;
    config.pin_pclk      = PCLK_GPIO_NUM;
    config.pin_vsync     = VSYNC_GPIO_NUM;
    config.pin_href      = HREF_GPIO_NUM;
    config.pin_sscb_sda  = SIOD_GPIO_NUM;
    config.pin_sscb_scl  = SIOC_GPIO_NUM;
    config.pin_pwdn      = PWDN_GPIO_NUM;
    config.pin_reset     = RESET_GPIO_NUM;
    config.xclk_freq_hz  = 20000000;
    config.pixel_format  = PIXFORMAT_JPEG;

    // Use smaller frame to keep UDP packets within 65507-byte limit
    if (psramFound()) {
        config.frame_size   = FRAMESIZE_QVGA;  // 320x240 — fits comfortably in one UDP packet
        config.jpeg_quality = 12;
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Camera init failed: 0x%x\n", err);
    } else {
        Serial.println("[CAM] Camera initialised (QVGA/JPEG).");
    }
}

// ---------------------------------------------------------------------------
// Servo helpers
// ---------------------------------------------------------------------------
void applyServo(int pan, int tilt) {
    currentPan  = constrain(pan,  0, 180);
    currentTilt = constrain(tilt, 0, 180);
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
    Serial.printf("[SERVO] Pan=%d  Tilt=%d\n", currentPan, currentTilt);
}

void resetServos() {
    applyServo(PAN_CENTER, TILT_CENTER);
    Serial.println("[SERVO] Reset to centre.");
}

// ---------------------------------------------------------------------------
// Command parser  ("PING", "FLASH_TOGGLE", "SERVO:<p>:<t>", "SERVO_RESET")
// ---------------------------------------------------------------------------
void handleCommand(const String& msg) {
    if (msg == "PING") {
        // Reply PONG to the sender
        cmdUdp.beginPacket(cmdUdp.remoteIP(), cmdUdp.remotePort());
        cmdUdp.print("PONG");
        cmdUdp.endPacket();

        // Remember the laptop's IP so we can push video to it
        laptopIP    = cmdUdp.remoteIP();
        laptopKnown = true;

    } else if (msg == "FLASH_TOGGLE") {
        flashOn = !flashOn;
        digitalWrite(FLASH_LED_PIN, flashOn ? HIGH : LOW);
        Serial.printf("[FLASH] %s\n", flashOn ? "ON" : "OFF");

    } else if (msg.startsWith("SERVO:")) {
        // Format: "SERVO:<pan>:<tilt>"
        int firstColon  = msg.indexOf(':', 6);
        if (firstColon == -1) return;
        int pan  = msg.substring(6, firstColon).toInt();
        int tilt = msg.substring(firstColon + 1).toInt();
        applyServo(pan, tilt);

    } else if (msg == "SERVO_RESET") {
        resetServos();
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Flash LED
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);

    // Servos
    panServo.attach(PAN_SERVO_PIN);
    tiltServo.attach(TILT_SERVO_PIN);
    resetServos();

    // Camera
    initCamera();

    // WiFi
    Serial.printf("[NET] Connecting to %s ...\n", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[NET] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // UDP sockets
    cmdUdp.begin(CMD_PORT);
    Serial.printf("[NET] Listening for commands on UDP port %d\n", CMD_PORT);
    Serial.printf("[NET] Will push video UDP to laptop port %d once laptop is known\n", VIDEO_PORT);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    // ---- 1. Handle incoming control commands ----
    int packetSize = cmdUdp.parsePacket();
    if (packetSize > 0) {
        int len = cmdUdp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if (len > 0) incomingPacket[len] = '\0';
        String msg = String(incomingPacket);
        msg.trim();
        handleCommand(msg);
    }

    // ---- 2. Capture and push JPEG frame (only if laptop IP is known) ----
    if (laptopKnown) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            if (fb->len <= 65000) {
                // Single-packet send (QVGA JPEG is typically 5-20 kB)
                vidUdp.beginPacket(laptopIP, VIDEO_PORT);
                vidUdp.write(fb->buf, fb->len);
                vidUdp.endPacket();
            } else {
                Serial.printf("[CAM] Frame too large for single UDP packet (%u bytes), skipping.\n", fb->len);
            }
            esp_camera_fb_return(fb);
        }
    }

    // Small yield to prevent watchdog resets
    delay(5);
}
