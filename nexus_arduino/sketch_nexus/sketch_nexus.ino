/*
*
* The documntation in this sketch and constants are based on:
* 1- NEXUS ROBOT User Manual (https://openhacks.com/uploadsproductos/file-1342627200__1_.pdf)
* 2- Omni4WD and MotorWheel library (https://github.com/lupusorina/nexus-robots/tree/master/documentation-libraries/lib/MotorWheel)
*
*
*/


/*
Mecanum4WD Top View (See position of Power Switch for reference)
From the top view of the robot you can't see the arduino board.


  			          Front MOTORS_FB
                ------------------
               |                  |
  wheel1_UL // |		              | \\ wheel4_UR	
                ------------------
                ------------------
               |                  |          
(Power Switch) |                  |
               |     Top View     |
               |                  |
               |  (Arduino board  |        
  wheel2_LL \\ |    not visible)  | // wheel3_LR
               |                  |  
                ------------------
  			          Back MOTORS_BF
                                           


Mecanum4WD Bottom View (See position of Power Switch for reference)
Here you can see the arduino board.


			          Front MOTORS_FB
              ------------------
             |                  |
wheel4_UR \\ |		              | // wheel1_UL	
              ------------------
              ------------------
             |                  |          
             |                  | (Power Switch)
             |   Bottom View    |
             |                  |
             |  (Arduino board  |        
wheel3_LR // |     visible)     | \\ wheel2_LL
             |                  |  
              ------------------
			          Back MOTORS_BF
*/  
 
/*
Initialize serial communication with:
- Baudrate: 115200  (For Arduino 328: Baudrates between 300 and 2800000 were tested and they worked. 
                     Baudrates that did not work in the tests 100, 200, 2850000 and 2900000).
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow control: None (default for Arduino Serial)


To find the max speed of the robot (see MotorWheel.h):
MAX_SPEEDRPM 8000 => see MotorWheel.h
REDUCTION_RATIO 64
CIRMM 314
=>  Output speed = 8000 / 64 = 125 RPM       
=>  125 RPM × 314 mm = 39,250 mm/min
=>  39,250 / 60 ≈ 654 mm/s
=>  max speed for each wheel = ~  654 mm/s (for safety use  500 mm/s)
=>  min speed for each wheel = ~ -654 mm/s (for safety use -500 mm/s)

Kinematic equations for "my" 4WD mecanum robot configured as:
v1_UL = -vy + vx - w*(r)     // in my case v1_UL = 4_UR in the library
v2_LL = -vy - vx - w*(r)     // in my case v2_LL = 3_LR in the library
v3_LR = +vy + vx - w*(r)     // in my case v3_LR = 1_UL in the library
v4_UR = +vy - vx - w*(r)     // in my case v4_UR = 2_LL in the library

Kinematic equations for the 4WD mecanum robot from the Omni4WD library (See UNCOMMENTED section in Omni4WD::setCarMove):
v_UL=Vty+Vtx-w(a+b);
v_LL=Vty-Vtx-w(a+b);
v_LR=Vty+Vtx+w(a+b);
v_UR=Vty-Vtx+w(a+b);
*/



#include <Arduino.h>
#include <Omni4WD.h>

#define minValue (int)-500
#define maxValue (int) 500

irqISR(irq1, isr1);
MotorWheel wheel1(3, 2, 4, 5, &irq1);
irqISR(irq2, isr2);
MotorWheel wheel2(11, 12, 14, 15, &irq2);
irqISR(irq3, isr3);
MotorWheel wheel3(9, 8, 16, 17, &irq3);
irqISR(irq4, isr4);
MotorWheel wheel4(10, 7, 18, 19, &irq4);

Omni4WD Omni(&wheel1, &wheel2, &wheel3, &wheel4);

int twist_v1 = 0, twist_v2 = 0, twist_v3 = 0, twist_v4 = 0;
int encoder_v1 = 0, encoder_v2 = 0, encoder_v3 = 0, encoder_v4 = 0;
int t1 = 0, t2 = 0, t3 = 0, t4 = 0;
// Watchdog
unsigned long lastSerialTime = 0;
const unsigned long serialTimeout = 500;  // 500 ms timeout -> if no commands for 0.5 sec stop the robot

void stopMotors() {
  Omni.wheelULSetSpeedMMPS(0);
  Omni.wheelURSetSpeedMMPS(0);
  Omni.wheelLLSetSpeedMMPS(0);
  Omni.wheelLRSetSpeedMMPS(0);
  Omni.PIDRegulate();
}

int clamp(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

//int rampingFunc(int v_target, int v_current, int steps){
//  if (v_target > v_current){
//    return min(v_current + steps, v_target);
//  } else if (v_current > v_target) {
//    return max(v_current - steps, v_target);
//  }
//  return v_current;
//}

//#ifndef TCCR1B
//#define TCCR1B	_SFR_IO8(0x2E)
//#endif
//
//#ifndef TCCR2 
//#define TCCR2	_SFR_IO8(0x25)
//#endif
//
//#ifndef TCCR2B
//#define TCCR2B TCCR2
//#endif

void setup() {
  Serial.begin(115200);
  TCCR1B = TCCR1B & 0xf8 | 0x01;
  TCCR2B = TCCR2B & 0xf8 | 0x01;
  Omni.PIDEnable(0.22, 0.02, 0.2, 2);
  lastSerialTime = millis();  // Initialize last time
}

String input = "";
bool inputValid = false;

void loop() {
  // 1. Read and apply incoming twist command
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read a full line
    input.trim();  // Remove trailing \r or spaces

    // Check framing
    if (input.startsWith("@") && input.endsWith("#")) {
      int starIndex = input.indexOf('*');
      if (starIndex > 0) {
        // Extract the payload and checksum
        String payload = input.substring(1, starIndex);  // Between @ and *
        String checksumStr = input.substring(starIndex + 1, input.length() - 1);  // Between * and #

        // Compute checksum (example: simple XOR of payload chars)
        byte computedChecksum = 0;
        for (int i = 0; i < payload.length(); i++) {
          computedChecksum ^= payload[i];
        }

        byte receivedChecksum = (byte)checksumStr.toInt();

        if (computedChecksum == receivedChecksum) {
          // Parse and use the data
          int c1 = payload.indexOf(',');
          int c2 = payload.indexOf(',', c1 + 1);
          int c3 = payload.indexOf(',', c2 + 1);

          if (c1 > 0 && c2 > 0 && c3 > 0) {
            twist_v1 = clamp(payload.substring(0, c1).toInt(), minValue, maxValue);
            twist_v2 = clamp(payload.substring(c1 + 1, c2).toInt(), minValue, maxValue);
            twist_v3 = clamp(payload.substring(c2 + 1, c3).toInt(), minValue, maxValue);
            twist_v4 = clamp(payload.substring(c3 + 1).toInt(), minValue, maxValue);

            Omni.wheelULSetSpeedMMPS(twist_v1);
            Omni.wheelURSetSpeedMMPS(twist_v2);
            Omni.wheelLLSetSpeedMMPS(twist_v3);
            Omni.wheelLRSetSpeedMMPS(twist_v4);
            Omni.PIDRegulate();

            lastSerialTime = millis();
            inputValid = true;
          }
        } else {
          // Checksum mismatch
          inputValid = false;
          // Serial.println("Checksum failed");
        }
      }
    }
  }

  // 2. Watchdog: stop motors if timeout
  if (millis() - lastSerialTime > serialTimeout) {
    stopMotors();
    Omni.PIDRegulate();
  }

  // 3. Send combined twist + encoder values every 20 ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 20) {
    lastSend = millis();
    if (inputValid) {
      encoder_v1 = Omni.wheelULGetSpeedMMPS();
      encoder_v2 = Omni.wheelURGetSpeedMMPS();
      encoder_v3 = Omni.wheelLLGetSpeedMMPS();
      encoder_v4 = Omni.wheelLRGetSpeedMMPS();
      // Construct payload string
      String payload = String(twist_v1) + "," + twist_v2 + "," + twist_v3 + "," + twist_v4 +
                       "|" + encoder_v1 + "," + encoder_v2 + "," + encoder_v3 + "," + encoder_v4;
      // Compute checksum (XOR of all characters in payload)
      byte checksum = 0;
      for (int i = 0; i < payload.length(); i++) {
        checksum ^= payload[i];
      }
      // Send formatted message with checksum
      Serial.print("%");
      Serial.print(payload);
      Serial.print("*");
      Serial.print(checksum);
      Serial.println("!");

      input = "";
      inputValid = false;
    }
  }
}




/*
#include <Arduino.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <MotorWheel.h>
//#include "CRC8.h"
//#include <avr/io.h>

#define WHEELSPAN 300                         
#ifndef TCCR1B
#define TCCR1B	_SFR_IO8(0x2E)
#endif

#ifndef TCCR2 
#define TCCR2	_SFR_IO8(0x25)
#endif

#ifndef TCCR2B
#define TCCR2B TCCR2
#endif

#define minValue (int)-500
#define maxValue (int) 500
#define BAUD_RATE 19200  // Set the desired baud rate

#define DEBUG

//MotorWheel(_pinPWM, _pinDir, _pinIRQ, _pinIRQB, _isr, ratio=REDUCTION_RATIO, cirMM=CIRMM);
//Motor PWM:Pin3, DIR:Pin2, Encoder A:Pin4, B:Pin5
irqISR(irq1, isr1);
MotorWheel wheel1_UL(3, 2, 4, 5, &irq1);
irqISR(irq2, isr2);
MotorWheel wheel2_LL(11, 12, 14, 15, &irq2);
irqISR(irq3, isr3);
MotorWheel wheel3_LR(9, 8, 16, 17, &irq3);
irqISR(irq4, isr4);
MotorWheel wheel4_UR(10, 7, 18, 19, &irq4);

Omni4WD Omni(&wheel1_UL, &wheel2_LL, &wheel3_LR, &wheel4_UR);

int v1_UL = 0, v2_UR = 0, v3_LL = 0, v4_LR = 0;
int enc_v1_UL = 0, enc_v2_UR = 0, enc_v3_LL = 0, enc_v4_LR = 0;
int t1 = 0, t2 = 0, t3 = 0, t4 = 0;
String input = "";
bool inputValid = false;
int receivedID = 0;
// Success rate tracking
int lastCounter = -1;           
int receivedPackets = 0;        
int droppedPackets = 0; 

// Watchdog
unsigned long lastSerialTime = 0;
const unsigned long serialTimeout = 500;  // 500 ms timeout -> if no commands for 0.5 sec stop the robot
*/

/*
uint8_t computeCRC8(const String &data) {
  uint8_t crc = 0x00;
  for (int i = 0; i < data.length(); i++) {
    crc ^= data[i];  // XOR byte into CRC
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void stopMotors() {
  Omni.wheelULSetSpeedMMPS(0);
  Omni.wheelURSetSpeedMMPS(0);
  Omni.wheelLLSetSpeedMMPS(0);
  Omni.wheelLRSetSpeedMMPS(0);
}

int clamp(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

*/

/*
void setup() {
  Serial.begin(BAUD_RATE);  // Replace with desired baud rate
  while (!Serial);          // Wait for serial port to be ready
  Serial.println("READY");
}

void loop() {
  Serial.println(" <Echo:500,-500,500,-500|Encoder:600,-600,600,-600#SR*CRC>");
  delay(1000);
}
*/



/*

void setup() {

  Serial.begin(BAUD_RATE); // Must match the settings in the ROS 2 Python node

  //while (!Serial) {
  //  ; // Wait for serial port to connect (for boards with native USB only)
  //}

  //Serial.flush();

  TCCR1B = TCCR1B & 0xf8 | 0x01;  // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01;  // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.35, 0.02, 0, 10);
  //Omni.PIDDisable();             // Turn off PID
  //Omni.setMotorAllStop();        // Stop all motors
  //Omni.PIDRegulate();            // Ensure update

  lastSerialTime = millis();  // Initialize last time
}

void loop() {
  
  Omni.wheelULSetSpeedMMPS(100, DIR_ADVANCE);
  Omni.wheelURSetSpeedMMPS(100, DIR_ADVANCE);
  Omni.wheelLLSetSpeedMMPS(100, DIR_ADVANCE);
  Omni.wheelLRSetSpeedMMPS(100, DIR_ADVANCE);
  Omni.PIDRegulate();
  delay(10000);
  
  //Omni.setCarStop();
  Omni.setCarBackoff();
  Omni.PIDRegulate();
  delay(10000);
  Omni.wheelULSetSpeedMMPS(0);
  Omni.wheelURSetSpeedMMPS(0);
  Omni.wheelLLSetSpeedMMPS(0);
  Omni.wheelLRSetSpeedMMPS(0);
  // 1. Receive command from Pi
  if (Serial.available()>0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    // <T:0,0,0,0,0|0*0>
    if (input.startsWith("<T:") && input.endsWith(">")) {
      int starIndex = input.indexOf('*');
      if (starIndex > 0) {
        String payload = input.substring(3, starIndex);  // after <T:
        String checksumStr = input.substring(starIndex + 1, input.length() - 1);

        uint8_t computedCRC = computeCRC8(payload);
        uint8_t receivedCRC = checksumStr.toInt();

        if (computedCRC == receivedCRC) {
          // Parse message
          int idx[5];
          int pos = -1;
          for (int i = 0; i < 5; i++) {
            idx[i] = payload.indexOf(',', pos + 1);
            pos = idx[i];
            if (pos == -1 && i < 4) return;
          }

          int currentCounter = payload.substring(0, idx[0]).toInt(); 
          v1_UL = clamp(payload.substring(idx[0] + 1, idx[1]).toInt(), minValue, maxValue);
          v2_UR = clamp(payload.substring(idx[1] + 1, idx[2]).toInt(), minValue, maxValue);
          v3_LL = clamp(payload.substring(idx[2] + 1, idx[3]).toInt(), minValue, maxValue);
          v4_LR = clamp(payload.substring(idx[3] + 1).toInt(), minValue, maxValue);

          // ✅ TRACK gaps in counter
          if (lastCounter >= 0) {
            int diff = (currentCounter - lastCounter + 256) % 256;
            if (diff > 1) droppedPackets += (diff - 1);
          }
          lastCounter = currentCounter;
          receivedPackets++;

          // Apply speeds
          Omni.wheelULSetSpeedMMPS(v1_UL);
          Omni.wheelURSetSpeedMMPS(v2_UR);
          Omni.wheelLLSetSpeedMMPS(v3_LL);
          Omni.wheelLRSetSpeedMMPS(v4_LR);
          Omni.PIDRegulate();

          lastSerialTime = millis();
          inputValid = true;
          receivedID = currentCounter;  // ✅ For reply
        }
      }
    }
  }

  // 2. Safety timeout
  if (millis() - lastSerialTime > serialTimeout || (v1_UL == v2_UR == v3_LL == v4_LR == 0)) {
    // Stop motors if no command received for a while
    //Serial.println("Stopping motors due to timeout");
    stopMotors();
    //Omni.PIDRegulate();
  }

  // 3. Send response
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 20) {
    lastSend = millis();
    if (inputValid) {
      enc_v1_UL = Omni.wheelULGetSpeedMMPS();
      enc_v2_UR = Omni.wheelURGetSpeedMMPS();
      enc_v3_LL = Omni.wheelLLGetSpeedMMPS();
      enc_v4_LR = Omni.wheelLRGetSpeedMMPS();

      // ✅ Accurate success rate
      int total = receivedPackets + droppedPackets;
      int SR = (total > 0) ? (receivedPackets * 100 / total) : 100;

      // Construct and send message
      //String payload = String(receivedID) + "," + String(enc_v1_UL) + "," +
      //                 String(enc_v2_UR) + "," + String(enc_v3_LL) + "," +
      //                 String(enc_v4_LR) + "|" + String(SR);
      
      String payload = String(receivedID) + "," + String(v1_UL) + "," +
                       String(v2_UR)      + "," + String(v3_LL) + "," +
                       String(v4_LR)      + "|" + String(SR);

      uint8_t crc = computeCRC8(payload);
      Serial.print("<E:" + payload + "*" + String(crc) + ">\n");

      inputValid = false;
    }
  }  
}
*/

/*
int rampingFunc(int v_target, int v_current, int steps){
  if (v_target > v_current){
    return min(v_current + steps, v_target);
  } else if (v_current > v_target) {
    return max(v_current - steps, v_target);
  }
  return v_current;
}
*/