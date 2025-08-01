//Type 0 - Follower
//Type 1 - Odometry
#define TYPE 2

#if TYPE == 0

#include "ArduPID.h"
ArduPID myController;

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#include <ESP32Encoder.h>

ESP32Encoder encoderD;
ESP32Encoder encoderE;

//#include "MovingAverage.h"
//MovingAverage<uint8_t, 64> filter;

double input;
double output;
double temp_input;

double setpoint = 0;
double p = 45;
double i = 15;
double d = 30;

int maxSpeed = 65;

//H-bridge
#define a1a 5
#define a1b 17
#define b1a 19
#define b1b 18

#define DEBUG 0

#define sensorOffset 0

int r_d1, r_d2, r_d3, r_d4, r_d5, r_d6, r_d7, r_d8;
int c_r_d1, c_r_d2, c_r_d3, c_r_d4, c_r_d5, c_r_d6, c_r_d7, c_r_d8;

String message = "";

int encoderEValue;
int encoderDValue;

void calibration(void);
void arrayRead(void);

void setup() {

  encoderE.attachHalfQuad(4, 16);
  encoderD.attachHalfQuad(2, 15);

  Serial.begin(115200);

  pinMode(a1a, OUTPUT);
  pinMode(a1b, OUTPUT);
  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);

  calibration();

  myController.begin(&input, &output, &setpoint, p, i, d);
  myController.setOutputLimits(-maxSpeed, maxSpeed);
  //myController.setBias(255.0 / 2.0);
  //myController.setWindUpLimits(-10, 10);

  myController.start();

  SerialBT.begin("tuiufollower");
}

void calibration() {
  c_r_d1 = 0;
  c_r_d2 = 0;
  c_r_d3 = 0;
  c_r_d4 = 0;
  c_r_d5 = 0;
  c_r_d6 = 0;
  c_r_d7 = 0;
  c_r_d8 = 0;

  for (int i = 0; i < 99; i++) {
    arrayRead();
    c_r_d1 = c_r_d1 + r_d1;
    c_r_d2 = c_r_d2 + r_d2;
    c_r_d3 = c_r_d3 + r_d3;
    c_r_d4 = c_r_d4 + r_d4;
    c_r_d5 = c_r_d5 + r_d5;
    c_r_d6 = c_r_d6 + r_d6;
    c_r_d7 = c_r_d7 + r_d7;
    c_r_d8 = c_r_d8 + r_d8;
  }

  c_r_d1 = c_r_d1 / 100;
  c_r_d2 = c_r_d2 / 100;
  c_r_d3 = c_r_d3 / 100;
  c_r_d4 = c_r_d4 / 100;
  c_r_d5 = c_r_d5 / 100;
  c_r_d6 = c_r_d6 / 100;
  c_r_d7 = c_r_d7 / 100;
  c_r_d8 = c_r_d8 / 100;

  c_r_d1 = c_r_d1 + sensorOffset;
  c_r_d2 = c_r_d2 + sensorOffset;
  c_r_d3 = c_r_d3 + sensorOffset;
  c_r_d4 = c_r_d4 + sensorOffset;
  c_r_d5 = c_r_d5 + sensorOffset;
  c_r_d6 = c_r_d6 + sensorOffset;
  c_r_d7 = c_r_d7 + sensorOffset;
  c_r_d8 = c_r_d8 + sensorOffset;

#if DEBUG == 2
  Serial.println("CALIBRATION VALUES");
  Serial.print(c_r_d1);
  Serial.print(" - ");
  Serial.print(c_r_d2);
  Serial.print(" - ");
  Serial.print(c_r_d3);
  Serial.print(" - ");
  Serial.print(c_r_d4);
  Serial.print(" - ");
  Serial.print(c_r_d5);
  Serial.print(" - ");
  Serial.print(c_r_d6);
  Serial.print(" - ");
  Serial.print(c_r_d7);
  Serial.print(" - ");
  Serial.print(c_r_d8);
  Serial.println(" - ");

#endif
}

void arrayRead() {
  r_d1 = analogRead(13);
  delay(1);
  r_d2 = analogRead(12);
  delay(1);
  r_d3 = analogRead(14);
  delay(1);
  r_d4 = analogRead(27);
  delay(1);
  r_d5 = analogRead(26);
  delay(1);
  r_d6 = analogRead(25);
  delay(1);
  r_d7 = analogRead(33);
  delay(1);
  r_d8 = analogRead(32);
  delay(1);
}

void moveMotors(int _a, int _b) {
  analogWrite(a1b, 128 + _a);
  analogWrite(a1a, 128 - _a);
  analogWrite(b1a, 128 + _b);
  analogWrite(b1b, 128 - _b);
}

int calculatePosition() {

  //TODO
  //Aplicar média móvel para reduzir ruído
  int position = 0;
  r_d1 = map(r_d1, c_r_d1, 4095, 0, 1000);
  if (r_d1 < 0) r_d1 = 0;
  r_d2 = map(r_d2, c_r_d2, 4095, 0, 1000);
  if (r_d2 < 0) r_d2 = 0;
  r_d3 = map(r_d3, c_r_d3, 4095, 0, 1000);
  if (r_d3 < 0) r_d3 = 0;
  r_d4 = map(r_d4, c_r_d4, 4095, 0, 1000);
  if (r_d4 < 0) r_d4 = 0;
  r_d5 = map(r_d5, c_r_d5, 4095, 0, 1000);
  if (r_d5 < 0) r_d5 = 0;
  r_d6 = map(r_d6, c_r_d6, 4095, 0, 1000);
  if (r_d6 < 0) r_d6 = 0;
  r_d7 = map(r_d7, c_r_d7, 4095, 0, 1000);
  if (r_d7 < 0) r_d7 = 0;
  r_d8 = map(r_d8, c_r_d8, 4095, 0, 1000);
  if (r_d8 < 0) r_d8 = 0;

  r_d1 = r_d1 * (8);
  r_d2 = r_d2 * (4);
  r_d3 = r_d3 * (2);
  r_d4 = r_d4 * (1);
  r_d5 = r_d5 * (-1);
  r_d6 = r_d6 * (-2);
  r_d7 = r_d7 * (-4);
  r_d8 = r_d8 * (-8);

  position = r_d1 + r_d2 + r_d3 + r_d4 + r_d5 + r_d6 + r_d7 + r_d8;

  //Serial.println(position);

  /*
  r_d2 = r_d2 - c_r_d2;
  if (r_d2 < 0) r_d2 = 0;
  r_d3 = r_d3 - c_r_d3;
  if (r_d3 < 0) r_d3 = 0;
  r_d4 = r_d4 - c_r_d4;
  if (r_d4 < 0) r_d4 = 0;
  r_d5 = r_d5 - c_r_d5;
  if (r_d5 < 0) r_d5 = 0;
  r_d6 = r_d6 - c_r_d6;
  if (r_d6 < 0) r_d6 = 0;
  r_d7 = r_d7 - c_r_d7;
  if (r_d7 < 0) r_d7 = 0;
  r_d8 = r_d8 - c_r_d8;
  if (r_d8 < 0) r_d8 = 0;
*/


  /*
  if (r_d1 > c_r_d1)
    position = position + 8;
  if (r_d2 > c_r_d2)
    position = position + 4;
  if (r_d3 > c_r_d3)
    position = position + 2;
  if (r_d4 > c_r_d4)
    position = position + 1;
  if (r_d5 > c_r_d5)
    position = position - 1;
  if (r_d6 > c_r_d6)
    position = position - 2;
  if (r_d7 > c_r_d7)
    position = position - 4;
  if (r_d8 > c_r_d8)
    position = position - 8;
  */
  return position;
}

void encoderRead() {
  encoderEValue = encoderE.getCount();
  delay(1);
  encoderDValue = encoderD.getCount();
  delay(1);
}

void loop() {

  arrayRead();
  encoderRead();

  //Serial.println(String((int32_t)encoderEValue) + " | " + String((int32_t)encoderDValue));

  //temp_input = calculatePosition();
  //input = filter.add(temp_input);

  input = calculatePosition();
  SerialBT.println(String((int32_t)encoderEValue) + " | " + String((int32_t)encoderDValue));

  myController.compute();


  if (output == 0) {
    moveMotors(maxSpeed, maxSpeed);
  } else if (output < 0) {
    moveMotors(maxSpeed, (maxSpeed + output));
  } else {
    moveMotors((maxSpeed - output), maxSpeed);
  }


  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n') {
      message += String(incomingChar);
    } else {
      message = "";
    }
  }
  if (message == "p+") {
    myController.stop();
    p = p + 5;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "p-") {
    myController.stop();
    p = p - 5;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "i+") {
    myController.stop();
    i = i + 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "i-") {
    myController.stop();
    i = i - 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "d+") {
    myController.stop();
    d = d + 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "d-") {
    myController.stop();
    d = d - 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "s+") {
    myController.stop();
    maxSpeed = maxSpeed + 1;
    myController.setOutputLimits(-maxSpeed, maxSpeed);
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "s-") {
    myController.stop();
    maxSpeed = maxSpeed - 1;
    myController.setOutputLimits(-maxSpeed, maxSpeed);
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  }


#if DEBUG == 1
  Serial.print(r_d1);
  Serial.print(" | ");
  Serial.print(r_d2);
  Serial.print(" | ");
  Serial.print(r_d3);
  Serial.print(" | ");
  Serial.print(r_d4);
  Serial.print(" | ");
  Serial.print(r_d5);
  Serial.print(" | ");
  Serial.print(r_d6);
  Serial.print(" | ");
  Serial.print(r_d7);
  Serial.print(" | ");
  Serial.print(r_d8);
  Serial.println(" | ");

#endif
}

#endif
#if TYPE == 1

#include "ArduPID.h"
ArduPID myController;

/*
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
*/

#include <ESP32Encoder.h>

ESP32Encoder encoderD;
ESP32Encoder encoderE;

double input;
double output;
double temp_input;

double setpoint = 0;
double p = 45;
double i = 15;
double d = 30;

int maxSpeed = 65;

//H-bridge
#define a1a 5
#define a1b 17
#define b1a 19
#define b1b 18

#define DEBUG 0

#define sensorOffset 0

int r_d1, r_d2, r_d3, r_d4, r_d5, r_d6, r_d7, r_d8;
int c_r_d1, c_r_d2, c_r_d3, c_r_d4, c_r_d5, c_r_d6, c_r_d7, c_r_d8;

String message = "";

int encoderEValue;
int encoderDValue;

void calibration(void);
void arrayRead(void);

void setup() {

  encoderE.attachHalfQuad(4, 16);
  encoderD.attachHalfQuad(2, 15);

  Serial.begin(115200);

  pinMode(a1a, OUTPUT);
  pinMode(a1b, OUTPUT);
  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);


  myController.begin(&input, &output, &setpoint, p, i, d);
  myController.setOutputLimits(-maxSpeed, maxSpeed);
  //myController.setBias(255.0 / 2.0);
  //myController.setWindUpLimits(-10, 10);
  myController.start();

  //SerialBT.begin("tuiufollower_odometry");
}

void moveMotors(int _a, int _b) {
  analogWrite(a1b, 128 + _a);
  analogWrite(a1a, 128 - _a);
  analogWrite(b1a, 128 + _b);
  analogWrite(b1b, 128 - _b);
}

void encoderRead() {
  encoderEValue = encoderE.getCount();
  delay(1);
  encoderDValue = encoderD.getCount();
  delay(1);
}

void loop() {

  moveMotors(60, 60);
  encoderRead();
  Serial.println(String((int32_t)encoderEValue) + " | " + String((int32_t)encoderDValue));

  //SerialBT.println(String((int32_t)encoderEValue) + " | " + String((int32_t)encoderDValue));

  //myController.compute();

  /*
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n') {
      message += String(incomingChar);
    } else {
      message = "";
    }
  }
  if (message == "p+") {
    myController.stop();
    p = p + 5;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "p-") {
    myController.stop();
    p = p - 5;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "i+") {
    myController.stop();
    i = i + 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "i-") {
    myController.stop();
    i = i - 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "d+") {
    myController.stop();
    d = d + 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "d-") {
    myController.stop();
    d = d - 1;
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "s+") {
    myController.stop();
    maxSpeed = maxSpeed + 1;
    myController.setOutputLimits(-maxSpeed, maxSpeed);
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  } else if (message == "s-") {
    myController.stop();
    maxSpeed = maxSpeed - 1;
    myController.setOutputLimits(-maxSpeed, maxSpeed);
    myController.begin(&input, &output, &setpoint, p, i, d);
    myController.start();
  }
  */
}

#endif

#if TYPE == 2

#include <Arduino.h>

#if !defined( ESP32 )
	#error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

#define _TIMERINTERRUPT_LOGLEVEL_ 4
#include "ESP32_New_TimerInterrupt.h"

#define PIN_D19             19        // Pin D19 mapped to pin GPIO9 of ESP32

bool IRAM_ATTR TimerHandler0(void * timerNo)
{
	static bool toggle0 = false;
	digitalWrite(PIN_D19, toggle0);
	toggle0 = !toggle0;

	return true;
}

#define TIMER0_INTERVAL_MS 500

ESP32Timer ITimer0(0);

void setup()
{
	pinMode(PIN_D19, OUTPUT);

	Serial.begin(115200);

	if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
	{
		Serial.print(F("Starting  ITimer0 OK, millis() = "));
		Serial.println(millis());
	}
	else
		Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

	Serial.flush();
}

void loop()
{

}

#endif
