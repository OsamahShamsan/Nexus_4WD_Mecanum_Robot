#include <Arduino.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <MotorWheel.h>
//#include <avr/io.h>

irqISR(irq1, isr1);
MotorWheel wheel1(3, 2, 4, 5, &irq1);
irqISR(irq2, isr2);
MotorWheel wheel2(11, 12, 14, 15, &irq2);
irqISR(irq3, isr3);
MotorWheel wheel3(9, 8, 16, 17, &irq3);
irqISR(irq4, isr4);
MotorWheel wheel4(10, 7, 18, 19, &irq4);

Omni4WD Omni(&wheel1, &wheel2, &wheel3, &wheel4);

void setup() {

  Serial.begin(115200); // Must match the settings in the ROS 2 Python node

  TCCR1B = TCCR1B & 0xf8 | 0x01;  // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01;  // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.27, 0.01, 0, 10);
  //Omni.PIDDisable();             // Turn off PID
  //Omni.setMotorAllStop();        // Stop all motors
  Omni.PIDRegulate();            // Ensure update

}
int i = 0;
bool increasing = true;

void loop() {
     
  Omni.setCarAdvance(i);
  Omni.PIDRegulate();

  if (increasing) {
    i++;
    if (i >= 300)  {
        increasing = false; 
        delay(10); // Wait for 5 second before decreasing
      }
  } else {
    i--;
    if (i <= -300) {
      increasing = true;
      delay(10); // Wait for 5 second before increasing
      }
  }
  Serial.println(i);
  delay(10);
}
