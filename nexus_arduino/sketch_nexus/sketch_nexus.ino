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

void setup() {
  Serial.begin(115200);
  TCCR1B = TCCR1B & 0xf8 | 0x01;
  TCCR2B = TCCR2B & 0xf8 | 0x01;
  Omni.PIDEnable(0.35, 0.02, 0, 10);

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
