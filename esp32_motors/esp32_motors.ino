/*
 * esp32_motors.ino  –  NURC 2026 Motor Controller Firmware
 * ==========================================================
 * Connects as a client to the "rc2" WiFi hotspot, then listens
 * for UDP commands on port 5001 from the laptop dashboard.
 *
 * UDP Commands (all single-byte / short strings):
 *   "PING"  → reply "PONG", resets the fail-safe watchdog timer
 *   "F"     → go forward
 *   "B"     → go backward
 *   "L"     → turn left  (pivot)
 *   "R"     → turn right (pivot)
 *   "S"     → stop all motors
 *
 * Fail-safe: if no PING is received within HEARTBEAT_TIMEOUT ms,
 * motors are stopped automatically (e.g. laptop disconnects / crashes).
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// WiFi credentials
// ---------------------------------------------------------------------------
const char* ssid     = "rc2";
const char* password = "244466666";

// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------
WiFiUDP udp;
unsigned int localUdpPort = 5001;
char         incomingPacket[255];

// ---------------------------------------------------------------------------
// Fail-safe watchdog
// ---------------------------------------------------------------------------
unsigned long lastHeartbeatTime   = 0;
const unsigned long HEARTBEAT_TIMEOUT = 1500; // ms — slightly more forgiving than before
bool   motorsStopped = true; // track state to avoid redundant writes

// ---------------------------------------------------------------------------
// Motor servos (continuous rotation)
// ---------------------------------------------------------------------------
Servo frontLeft;
Servo backLeft;
Servo frontRight;
Servo backRight;

const int pinFL = 13;
const int pinBL = 12;
const int pinFR = 14;
const int pinBR = 27;

// Continuous servo neutral/speed constants (microseconds)
const int STOP_US     = 1500;
const int FWD_DIR     =  1;
const int REV_DIR     = -1;
const int SPEED_OFFSET = 300; // ±µs from neutral

int getSpeed(int dir) {
    return STOP_US + (dir * SPEED_OFFSET);
}

// ---------------------------------------------------------------------------
// Drive primitives
// ---------------------------------------------------------------------------
void setMotors(int fl, int bl, int fr, int br) {
    frontLeft.writeMicroseconds(getSpeed(fl));
    backLeft.writeMicroseconds(getSpeed(bl));
    // Right side physically mirrored on chassis → invert
    frontRight.writeMicroseconds(getSpeed(-fr));
    backRight.writeMicroseconds(getSpeed(-br));
    motorsStopped = (fl == 0 && bl == 0 && fr == 0 && br == 0);
}

void goForward()  { setMotors(FWD_DIR, FWD_DIR, FWD_DIR, FWD_DIR); Serial.println("[MOTOR] Forward");  }
void goBackward() { setMotors(REV_DIR, REV_DIR, REV_DIR, REV_DIR); Serial.println("[MOTOR] Backward"); }
void turnLeft()   { setMotors(REV_DIR, REV_DIR, FWD_DIR, FWD_DIR); Serial.println("[MOTOR] Left");     }
void turnRight()  { setMotors(FWD_DIR, FWD_DIR, REV_DIR, REV_DIR); Serial.println("[MOTOR] Right");    }

void stopMotors() {
    frontLeft.writeMicroseconds(STOP_US);
    backLeft.writeMicroseconds(STOP_US);
    frontRight.writeMicroseconds(STOP_US);
    backRight.writeMicroseconds(STOP_US);
    motorsStopped = true;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Attach and stop all drive servos
    frontLeft.attach(pinFL);
    backLeft.attach(pinBL);
    frontRight.attach(pinFR);
    backRight.attach(pinBR);
    stopMotors();

    // WiFi
    Serial.printf("[NET] Connecting to %s ...\n", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[NET] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // Start UDP server
    udp.begin(localUdpPort);
    Serial.printf("[NET] Waiting for commands on UDP port %d\n", localUdpPort);

    // Seed the watchdog so we don't immediately trigger it on boot
    lastHeartbeatTime = millis();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    // ---- 1. Receive and dispatch UDP commands ----
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if (len > 0) incomingPacket[len] = '\0';

        String msg = String(incomingPacket);
        msg.trim();

        if (msg == "PING") {
            // Heartbeat – reply and reset watchdog
            lastHeartbeatTime = millis();
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.print("PONG");
            udp.endPacket();
        }
        else if (msg == "F") { goForward();  }
        else if (msg == "B") { goBackward(); }
        else if (msg == "L") { turnLeft();   }
        else if (msg == "R") { turnRight();  }
        else if (msg == "S") { stopMotors(); }
        // Unknown commands are silently ignored
    }

    // ---- 2. Fail-safe: stop if heartbeat timed out ----
    if ((millis() - lastHeartbeatTime) > HEARTBEAT_TIMEOUT) {
        if (!motorsStopped) {
            Serial.println("[FAILSAFE] Heartbeat timeout – stopping motors!");
            stopMotors();
        }
    }

    delay(5);
}
