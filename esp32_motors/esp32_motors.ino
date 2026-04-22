#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

const char* ssid = "rc2";
const char* password = "244466666";

// Networking: UDP details
WiFiUDP udp;
unsigned int localUdpPort = 5001;
char incomingPacket[255];
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_TIMEOUT = 1000; // 1 second failsafe

// Hardware configurations
Servo frontLeft;
Servo backLeft;
Servo frontRight;
Servo backRight;

const int pinFL = 13;
const int pinBL = 12;
const int pinFR = 14;
const int pinBR = 27;

// Continuous servo parameters
const int STOP = 1500;
const int FWD_DIR = 1;  
const int REV_DIR = -1; 
const int SPEED_OFFSET = 300; // PWM offset from STOP (1200 - 1800)

int getSpeed(int dir) {
    return 1500 + (dir * SPEED_OFFSET);
}

void setup() {
    Serial.begin(115200);

    // Initialise servos (Timers are dynamically allocated by ESP32Servo)
    frontLeft.attach(pinFL);
    backLeft.attach(pinBL);
    frontRight.attach(pinFR);
    backRight.attach(pinBR);
    
    stopMotors();

    // WiFi connection
    Serial.printf("Connecting to Hotspot: %s ...\n", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nCONNECTED!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Begin UDP listening
    udp.begin(localUdpPort);
    Serial.printf("Listening for Motor commands and UDP Heartbeats on port: %d\n", localUdpPort);
}

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = '\0';
        }
        
        String msg = String(incomingPacket);
        msg.trim();
        
        if (msg == "PING") {
            // Heartbeat received, respond PONG to verify presence to laptop
            lastHeartbeatTime = millis();
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.printf("PONG");
            udp.endPacket();
        } 
        else if (msg == "F") { goForward(); }
        else if (msg == "B") { goBackward(); }
        else if (msg == "L") { turnLeft(); }
        else if (msg == "R") { turnRight(); }
        else if (msg == "S") { stopMotors(); }
    }

    // Fail-Safe: Stop motors unconditionally if no PING in HEARTBEAT_TIMEOUT ms
    if (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT) {
        stopMotors();
    }
    
    // Small delay to yield to OS and prevent watchdog triggers
    delay(10);
}

void setMotors(int fl, int bl, int fr, int br) {
    // Left side
    frontLeft.writeMicroseconds(getSpeed(fl));
    backLeft.writeMicroseconds(getSpeed(bl));
    
    // Right side: inverted because servos are usually physically mirrored on chassis!
    frontRight.writeMicroseconds(getSpeed(-fr));
    backRight.writeMicroseconds(getSpeed(-br));
}

// Drive implementations
void goForward()  { setMotors(FWD_DIR, FWD_DIR, FWD_DIR, FWD_DIR); }
void goBackward() { setMotors(REV_DIR, REV_DIR, REV_DIR, REV_DIR); }
void turnLeft()   { setMotors(REV_DIR, REV_DIR, FWD_DIR, FWD_DIR); }
void turnRight()  { setMotors(FWD_DIR, FWD_DIR, REV_DIR, REV_DIR); }
void stopMotors() {
    frontLeft.writeMicroseconds(STOP);
    backLeft.writeMicroseconds(STOP);
    frontRight.writeMicroseconds(STOP);
    backRight.writeMicroseconds(STOP);
}
