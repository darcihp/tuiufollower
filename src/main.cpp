//Type 0 - Follower
//Type 1 - Odometry
#define TYPE 3
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

#include <Arduino.h>

#define _TIMERINTERRUPT_LOGLEVEL_ 4
#include "ESP32_New_TimerInterrupt.h"
#define TIMER0_INTERVAL_MS 100
ESP32Timer ITimer0(0);

#include <ESP32Encoder.h>
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

//H-bridge
#define a1a 5
#define a1b 17
#define b1a 19
#define b1b 18

long ticks_encoder_l;
long ticks_encoder_r;

float wheel_radius_l = 0.034;
float wheel_radius_r = 0.034;
float baseline = 0.087;

float interruption_time = 100.0;
//360degrees/ 60ppr
const float speed_constant = 6;

float last_x = 0;
float last_y = 0;
float last_theta = 0;

int state_machine = 0;

bool IRAM_ATTR TimerHandler0(void * timerNo);
void moveMotors(int _a, int _b);

void setup() {

  Serial.begin(115200);

  pinMode(a1a, OUTPUT);
  pinMode(a1b, OUTPUT);
  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);
  
  ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0);
  //attachInterrupt(digitalPinToInterrupt(encoder_l), count_encoder_l, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoder_r), count_encoder_r, RISING);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoder_l.attachFullQuad(4, 16);
  encoder_r.attachFullQuad(2, 15);
  encoder_l.setFilter(1023);
  encoder_r.setFilter(1023);
  encoder_l.setCount (0);
  encoder_r.setCount (0); 
}

bool IRAM_ATTR TimerHandler0(void * timerNo)
{
  ticks_encoder_l = encoder_l.getCount();
  ticks_encoder_r = encoder_r.getCount();
  
  //°
  float last_degrees_l = ticks_encoder_l * speed_constant;  
  float last_degrees_r = ticks_encoder_r * speed_constant;
  //°/s
  float last_speed_deg_s_l = last_degrees_l / (interruption_time/1000);
  float last_speed_deg_s_r = last_degrees_r / (interruption_time/1000);
  //rad/s
  float last_speed_rad_s_l = last_speed_deg_s_l * 0.017453292519943;
  float last_speed_rad_s_r = last_speed_deg_s_r * 0.017453292519943;

  /*
  Serial.print(">last_speed_deg_s_l: ");
  Serial.println(last_speed_deg_s_l);
  Serial.print(">last_speed_deg_s_r: ");
  Serial.println(last_speed_deg_s_r);
  Serial.print(">last_speed_rad_s_l: ");
  Serial.println(last_speed_rad_s_l);
  Serial.print(">last_speed_rad_s_r: ");
  Serial.println(last_speed_rad_s_r);
  */
  
  encoder_l.clearCount();
  encoder_r.clearCount();

  //wheel velocity l
  float vl = wheel_radius_l * last_speed_rad_s_l;
  //wheel velocity r
  float vr = wheel_radius_r * last_speed_rad_s_r;

  Serial.print("vl: ");
  Serial.println(vl);
  Serial.print("vr: ");
  Serial.println(vr);

  //float x = ((vl/2)*cos(last_theta)) +  ((vr/2)*cos(last_theta));
  float x = (((vl/2)*cos(last_theta)) +  ((vr/2)*cos(last_theta))) * (interruption_time/1000);
  float y = (((vl/2)*sin(last_theta)) +  ((vr/2)*sin(last_theta))) * (interruption_time/1000);
  //float theta = (-(vl/baseline) + (vr/baseline)) * (interruption_time/1000);
  float theta = ((vr - vl)/baseline) * (interruption_time/1000);
  //float theta = 0;

  last_x += x;
  last_y += y;
  last_theta += theta;
  
  Serial.print(">last_x:");
  Serial.println(last_x);

  Serial.print(">last_y:");
  Serial.println(last_y);

  Serial.print(">last_theta:");
  Serial.println(last_theta);

  //Serial.println(String((int32_t)ticks_encoder_r) + " | " + String((int32_t)ticks_encoder_l));


	return true;
}


void moveMotors(int _a, int _b) {
  analogWrite(a1b, 128 + _a);delay(1);
  analogWrite(a1a, 128 - _a);delay(1);
  analogWrite(b1a, 128 + _b);delay(1);
  analogWrite(b1b, 128 - _b);delay(1);
}

void loop()
{
  int speed_r = 65;
  int speed_l = 60;

  if (state_machine == 0)
  {
    moveMotors(speed_l, speed_r);
  }
   if(last_x >= 0.5 && state_machine == 0)
  {
    moveMotors(-speed_l, speed_r);
    state_machine = 1;
  }

  if(last_theta >= 1.57 && state_machine == 1)
  {
    moveMotors(speed_l, speed_r);
    state_machine = 3;
  }

  if(last_y >= 0.5 && state_machine == 3)
  {
    moveMotors(-speed_l, speed_r);
    state_machine = 4;
  }

  if(last_theta >= 3.14 && state_machine == 4)
  {
    moveMotors(speed_l, speed_r);
    state_machine = 5;
  }

  if(last_x <= 0 && state_machine == 5)
  {
    moveMotors(-speed_l, speed_r);
    state_machine = 6;
  }

  if(last_theta >= 4.71 && state_machine == 6)
  {
    moveMotors(speed_l, speed_r);
    state_machine = 7;
  }

   if(last_y <= 0 && state_machine == 7)
  {
    moveMotors(-speed_l, speed_r);
    state_machine = 8;
  }

  if(last_theta >= 6.28 && state_machine == 8)
  {
    moveMotors(0, 0);
  }
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

#if TYPE == 3

#include <Arduino.h>
#include <FastPID.h>

#include <esp_now.h>
#include <WiFi.h>

#define _TIMERINTERRUPT_LOGLEVEL_ 4
#include "ESP32_New_TimerInterrupt.h"
#define TIMER0_INTERVAL_MS 100
ESP32Timer ITimer0(0);

#include <ESP32Encoder.h>
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

//H-bridge
#define a1a 5
#define a1b 17
#define b1a 19
#define b1b 18

long ticks_encoder_l;
long ticks_encoder_r;

float wheel_radius_l = 0.034;
float wheel_radius_r = 0.034;
float baseline = 0.150;

float interruption_time = 100.0;
//360degrees/ 60ppr
const float speed_constant = 6;

float last_x = 0;
float last_y = 0;
float last_theta = 0;

int state_machine = 0;

float input_l;
float output_l;
float setpoint_l = 10;
float p_l = 5;
float i_l = 2;
float d_l = 0.2;
float hz_l = 100;
int output_bits_l = 8;
bool output_signed_l = true;

FastPID myPID_l(p_l, i_l, d_l, hz_l, output_bits_l, output_signed_l);

float input_r;
float output_r;
float setpoint_r = 10;
float p_r = 5;
float i_r = 2;
float d_r = 0.2;
float hz_r = 100;
int output_bits_r = 8;
bool output_signed_r = true;
FastPID myPID_r(p_r, i_r, d_r, hz_r, output_bits_r, output_signed_r);

float last_target;

bool connection;
typedef struct struct_message {
  //char a[32];
  int vrx;
  int vry;
  bool btn_b_l;
  bool btn_b_r;
  bool btn_r_l;
  bool btn_r_r;
  //float c;
  //bool d;
} struct_message;
struct_message myData;

bool IRAM_ATTR TimerHandler0(void * timerNo);
void moveMotors(int _a, int _b);

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  connection = true;
  memcpy(&myData, incomingData, sizeof(myData)); 
  
  int max_speed = 50;

  if (myData.btn_b_l == 1)
  {
    moveMotors(max_speed, max_speed);
  }
  else if (myData.btn_b_r == 1)
  {
    moveMotors(-max_speed, -max_speed);
  }
  else if (myData.btn_r_l == 1)
  {
    moveMotors(-max_speed, max_speed);
  }
  else if (myData.btn_r_r == 1)
  {
    moveMotors(max_speed, -max_speed);
  }
  else
  {
    int m_vrx = map(myData.vrx, 0, 180, max_speed, -max_speed);
    int m_vry = map(myData.vry, 90, -90, max_speed, -max_speed);

    moveMotors(m_vrx - m_vry, m_vrx + m_vry);
  }
  /*
  Serial.print(">vrx: ");
  Serial.println(myData.vrx);
  Serial.print(">m_vrx: ");
  Serial.println(m_vrx);

  Serial.print(">vry: ");
  Serial.println(myData.vry);
  Serial.print(">m_vry: ");
  Serial.println(m_vry);
  */

  /*
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("vrx: ");
  Serial.println(myData.vrx);
  Serial.print("vry: ");
  Serial.println(myData.vry);
  Serial.print("btn_b_l: ");
  Serial.println(myData.btn_b_l);
  Serial.print("btn_b_r: ");
  Serial.println(myData.btn_b_r);
  Serial.print("btn_r_l: ");
  Serial.println(myData.btn_r_l);
  Serial.print("btn_r_r: ");
  Serial.println(myData.btn_r_r);  
  Serial.println();
  */

  /*
  Serial.print(">vrx: ");
  Serial.println(myData.vrx);
  Serial.print(">vry: ");
  Serial.println(myData.vry);
  Serial.print(">btn_b_l: ");
  Serial.println(myData.btn_b_l);
  Serial.print(">btn_b_r: ");
  Serial.println(myData.btn_b_r);
  Serial.print(">btn_r_l: ");
  Serial.println(myData.btn_r_l);
  Serial.print(">btn_r_r: ");
  Serial.println(myData.btn_r_r);
  */

}


void setup() {

  Serial.begin(115200);

  pinMode(a1a, OUTPUT);
  pinMode(a1b, OUTPUT);
  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);
  
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0);
  //attachInterrupt(digitalPinToInterrupt(encoder_l), count_encoder_l, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoder_r), count_encoder_r, RISING);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoder_l.attachFullQuad(4, 16);
  encoder_r.attachFullQuad(2, 15);
  encoder_l.setFilter(1023);
  encoder_r.setFilter(1023);
  encoder_l.setCount (0);
  encoder_r.setCount (0);
  
  myPID_l.setOutputRange(-100, 100);
  myPID_r.setOutputRange(-100, 100);
}

bool IRAM_ATTR TimerHandler0(void * timerNo)
{
  ticks_encoder_l = encoder_l.getCount();
  ticks_encoder_r = encoder_r.getCount();
  
  //°
  float last_degrees_l = ticks_encoder_l * speed_constant;  
  float last_degrees_r = ticks_encoder_r * speed_constant;
  //°/s
  float last_speed_deg_s_l = last_degrees_l / (interruption_time/1000);
  float last_speed_deg_s_r = last_degrees_r / (interruption_time/1000);
  //rad/s
  float last_speed_rad_s_l = last_speed_deg_s_l * 0.017453292519943;
  float last_speed_rad_s_r = last_speed_deg_s_r * 0.017453292519943;

  /*
  Serial.print(">last_speed_deg_s_l: ");
  Serial.println(last_speed_deg_s_l);
  Serial.print(">last_speed_deg_s_r: ");
  Serial.println(last_speed_deg_s_r);
  Serial.print(">last_speed_rad_s_l: ");
  Serial.println(last_speed_rad_s_l);
  Serial.print(">last_speed_rad_s_r: ");
  Serial.println(last_speed_rad_s_r);
  */
  
  encoder_l.clearCount();
  encoder_r.clearCount();

  //wheel velocity l
  float vl = wheel_radius_l * last_speed_rad_s_l;
  //wheel velocity r
  float vr = wheel_radius_r * last_speed_rad_s_r;

  /*
  Serial.print("vl: ");
  Serial.println(vl);
  Serial.print("vr: ");
  Serial.println(vr);
  */

  //float x = ((vl/2)*cos(last_theta)) +  ((vr/2)*cos(last_theta));
  float x = (((vl/2)*cos(last_theta)) +  ((vr/2)*cos(last_theta))) * (interruption_time/1000);
  float y = (((vl/2)*sin(last_theta)) +  ((vr/2)*sin(last_theta))) * (interruption_time/1000);
  //float theta = (-(vl/baseline) + (vr/baseline)) * (interruption_time/1000);
  float theta = ((vr - vl)/baseline) * (interruption_time/1000);
  //float theta = 0;

  last_x += x;
  last_y += y;
  last_theta += theta;
  
  
  Serial.print(">last_x:");
  Serial.println(last_x);

  Serial.print(">last_y:");
  Serial.println(last_y);

  Serial.print(">last_theta:");
  Serial.println(last_theta);
  
  //Serial.println(String((int32_t)ticks_encoder_r) + " | " + String((int32_t)ticks_encoder_l));


	return true;
}


void moveMotors(int _a, int _b) {
  analogWrite(a1b, 128 + _a);delay(1);
  analogWrite(a1a, 128 - _a);delay(1);
  analogWrite(b1a, 128 + _b);delay(1);
  analogWrite(b1b, 128 - _b);delay(1);
}


void loop()
{
  /*
  int speed_l = 20;
  int speed_r = 20;
  
  if (state_machine == 0)
  {
    setpoint_l = speed_l;
    setpoint_r = speed_r;
    last_target = 0.5;
  }
  if(last_x >= last_target && state_machine == 0)
  {
    setpoint_l = -speed_l;
    setpoint_r = speed_r;
    state_machine = 1;
    last_target = last_theta + 1.57;
  }

  if(last_theta >= last_target && state_machine == 1)
  {
    setpoint_l = speed_l;
    setpoint_r = speed_r;
    state_machine = 3;
    last_target = last_y + 0.5;
  }

  if(last_y >= last_target && state_machine == 3)
  {
    setpoint_l = -speed_l;
    setpoint_r = speed_r;
    state_machine = 4;
    last_target = last_theta + 1.57;
  }

  //if(last_theta >= 3.14 && state_machine == 4)
  if(last_theta >= last_target && state_machine == 4)
  {
    setpoint_l = speed_l;
    setpoint_r = speed_r;
    state_machine = 5;
    last_target = last_x - 0.5;
  }
  //if(last_x <= last_target && state_machine == 5)
  if(last_x <= 0 && state_machine == 5)
  {
    setpoint_l = -speed_l;
    setpoint_r = speed_r;
    state_machine = 6;
    last_target = last_theta + 1.57;
  }

  if(last_theta >= last_target && state_machine == 6)
  {
    setpoint_l = speed_l;
    setpoint_r = speed_r;
    state_machine = 7;
    last_target = last_y - 0.5;
  }

  //if(last_y <= last_target && state_machine == 7)
  if(last_y <= 0 && state_machine == 7)
  {
    setpoint_l = -speed_l;
    setpoint_r = speed_r;
    state_machine = 8;
    last_target = last_theta + 1.57;
  }

  if(last_theta >= last_target && state_machine == 8)
  {
    setpoint_l = 0;
    setpoint_r = 0;
    state_machine = 9;
  }

  input_l = ticks_encoder_l;
  input_r = ticks_encoder_r;
  output_l = myPID_l.step(setpoint_l, input_l);
  output_r = myPID_r.step(setpoint_r, input_r);
  
  if(state_machine == 9)
    moveMotors(0, 0);
  else
    moveMotors(output_l, output_r);
  */

  /*
  Serial.print(">output_l:");
  Serial.println(output_l);

  Serial.print(">output_r:");
  Serial.println(output_r);
  */
}


#endif
