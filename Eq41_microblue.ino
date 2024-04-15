#include <SoftwareSerial.h>  // SoftwareSerial for serial communication with HM10 bluetooth module.
#include <ArduinoBlue.h>     // ArduinoBlue bluetooth library
#include <Servo_Hardware_PWM.h>

Servo myBras;  // Créer un objet servomoteur
Servo myPince;

int myBras_pos = 69;
int myPince_pos = 200;

// MOTOR LEFT PINS
const int enableLeftMotor = 9;
const int leftMotorPin1 = 26;
const int leftMotorPin2 = 28;

// MOTOR RIGHT PINS
const int enableRightMotor = 4;
int rightMotorPin1 = 22;
int rightMotorPin2 = 24;

const int BLUETOOTH_TX = 11;
const int BLUETOOTH_RX = 10;

SoftwareSerial BLESerial(BLUETOOTH_TX, BLUETOOTH_RX);  // Object for serial communication to HM 10 bluetooth module using ditigal pins.

int throttle, steering, button, sliderId, sliderVal, a, b;


void setup() {
  delay(500);

  // Start communication with HM10 bluetooth module.
  BLESerial.begin(9600);

  // Begin serial communication with computer.
  Serial.begin(9600);

  // Set pin modes


  Serial.println("SETUP COMPLETE");

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  myBras.attach(5);
  myBras.write(myBras_pos);
  myPince.attach(6);
  myPince.write(myPince_pos);
}

struct BLEMessage {
  String id = "";
  String value = "";
};

struct BLEMessage getBLEMessage() {
  BLEMessage msg;
  // Read id and value if detect signature code 1
  if (BLESerial.available()) {
    int v = BLESerial.read();
    if (v == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 1000) {
        if (BLESerial.available()) {
          v = BLESerial.read();
          if (v == 2) break;       // Move to next while loop
          msg.id.concat((char)v);  // Append character to id string
        }
      }
      while (millis() - startTime < 1000) {
        if (BLESerial.available()) {
          v = BLESerial.read();
          if (v == 3) break;          // Finish appending to message strings
          msg.value.concat((char)v);  // Append character to value string
        }
      }
    }
    msg.value.toLowerCase();     // Convert all letters in value to lowercase
    msg.value.replace(" ", "");  // Filter out all spaces in message value
  }
  // Print id and value if Serial exist
  if (Serial) {
    if (msg.id.length() > 0 && msg.value.length() > 0) {
      Serial.print("id:");
      Serial.print(msg.id);
      Serial.print(" ");
      Serial.print("value:");
      Serial.println(msg.value);
    }
  }
  return msg;
}

void loop() {

  BLEMessage msg = getBLEMessage();

  if (msg.id == "d0") {
    int throttle, steering;
    sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);
    throttle -= 512;
    steering -= 512;
    controlDrive(throttle, steering);
  } else if (msg.id == "d1") {
    int throttle, steering;
    sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);
    throttle -= 512;
    steering -= 512;
    controlServo(throttle, steering);
  }
}

void controlDrive(int throttle, int steering) {
  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;

  if (throttle < 0) {
    rightMotorSpeed = -200;
    leftMotorSpeed = 200;
  }

  if (throttle > 0) {
    rightMotorSpeed = 200;
    leftMotorSpeed = -200;
  }

  if (steering > 0) {
    rightMotorSpeed = 180;
    leftMotorSpeed = 180;
  }

  if (steering < 0) {
    rightMotorSpeed = -180;
    leftMotorSpeed = -180;
  }

  //Affichage des infos
  if (throttle != 0) {
    Serial.print("Throttle: ");
    Serial.println(throttle);
  }
  if (steering != 0) {
    Serial.print("Steering: ");
    Serial.println(steering);
  }
  rotateMotorRight(rightMotorSpeed);
  rotateMotorLeft(leftMotorSpeed);
}

void rotateMotorRight(int rightMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed >= 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
}
void rotateMotorLeft(int leftMotorSpeed) {
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed >= 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

void controlServo(int throttle, int steering) {
  //Affichage des infos
  if (throttle != 0) {
    Serial.print("Throttle: ");
    Serial.println(throttle);
  }
  if (steering != 0) {
    Serial.print("Steering: ");
    Serial.println(steering);
  }
  // CONTRÔLE DU BRAS
  while (steering > 0 && myBras_pos > 0 && myBras_pos <= 70) {
    myBras_pos--;
    delay(15);
    myBras.write(myBras_pos);
    Serial.println(myBras_pos);
    BLEMessage msg = getBLEMessage();
    if (msg.id == "d1") {
      sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);
      throttle -= 512;
      steering -= 512;
    }
  }
  while (steering < 0 && myBras_pos >= 0 && myBras_pos < 70) {
    myBras_pos++;
    delay(15);
    myBras.write(myBras_pos);
    Serial.println(myBras_pos);
    BLEMessage msg = getBLEMessage();
    if (msg.id == "d1") {
      sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);
      throttle -= 512;
      steering -= 512;
    }
    
  }

  //PINCE
  while (throttle > 0 && myPince_pos >= 0 && myPince_pos < 200) {
    myPince_pos++;
    delay(15);
    myPince.write(myPince_pos);
    Serial.println(myPince_pos);
    BLEMessage msg = getBLEMessage();
    if (msg.id == "d1") {
      sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);
      throttle -= 512;
      steering -= 512;
    }
  }
  while (throttle < 0 && myPince_pos > 0 && myPince_pos <= 200) {
    myPince_pos--;
    delay(15);
    myPince.write(myPince_pos);
    Serial.println(myPince_pos);
    BLEMessage msg = getBLEMessage();
    if (msg.id == "d1") {
      sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);
      throttle -= 512;
      steering -= 512;
    }
  }
}
