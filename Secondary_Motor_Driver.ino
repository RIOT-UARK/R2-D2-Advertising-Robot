/*
  ARDUINO UNO FOR CONTROLLING THE POLOLU DOME MOTOR
  This board receives an integer over serial (may change this to a faster comm method later) from the Main ESP32 Receiver Board,
  then uses that to control the speed and direction of the Dome Motor.

  PINOUT
  Arduino -> PROK Motor Controller
  
  5V -> 5V
  GND -> GND
  Pin 2 -> IN1
  Pin 3 -> ENA
  Pin 4 -> IN2

  RX(PIN 0) -> ESP32 Receiver Board Pin TX 

  Using an Arduino Uno was necessary because the PROK Motor controller would only work with the Uno's 5V logic,
  and not an ESP32's 3V logic.
*/

// TODO: Implement Encoder Readings

unsigned long timeOfLastRec;
int driveValue;

void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  Serial.begin(115200);


 delay(2000); //Delay because the ESP32 will spew garbage over serial upon startup.
}

void loop() {
 
 

  if (Serial.available() > 0) {
      driveValue = Serial.read();  
      timeOfLastRec = millis();
      
  Serial.println(driveValue);
  
    if (driveValue > 14 && driveValue < 100) {
      digitalWrite(2, LOW);
      digitalWrite(4, HIGH);
      
      driveValue = int(((float(driveValue) - 14) * 255.0) / 85.0);
      
      }
    else if (driveValue < 241 && driveValue > 155) {
      digitalWrite(2, HIGH);
      digitalWrite(4, LOW);
      
      driveValue = int(((240 - float(driveValue)) * 255.0) / 85.0);
      
      }
    else {
      driveValue = 0;
    }
  
  analogWrite(3, driveValue);

  }

  if ((millis() - timeOfLastRec) > 200) {
  analogWrite(3, 0);
  }
}
