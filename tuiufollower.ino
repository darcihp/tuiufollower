/*
|   |   |   |   |   |   |   |   |


*/



//H-bridge
#define a1a 5
#define a1b 17
#define b1a 19
#define b1b 18

#define DEBUG 2

#define sensorOffset 300

int r_d1, r_d2, r_d3, r_d4, r_d5, r_d6, r_d7, r_d8;
int c_r_d1, c_r_d2, c_r_d3, c_r_d4, c_r_d5, c_r_d6, c_r_d7, c_r_d8;

void setup() {

  Serial.begin(115200);

  pinMode(a1a, OUTPUT);
  pinMode(a1b, OUTPUT);
  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);

  calibration();
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

  for (int i = 0; i < 100; i++) {
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
  analogWrite(a1a, _a);
  analogWrite(a1b, 255 - _a);
  analogWrite(b1a, _b);
  analogWrite(b1b, 255 - _b);
}

int calculatePosition() {
  int position = 0;

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
  return position;
}

void loop() {

  arrayRead();
  Serial.println(calculatePosition());

  

  //moveMotors(128, 128);

#if DEBUG == 1
  Serial.print(r_d1);
  Serial.print(" - ");
  Serial.print(r_d2);
  Serial.print(" - ");
  Serial.print(r_d3);
  Serial.print(" - ");
  Serial.print(r_d4);
  Serial.print(" - ");
  Serial.print(r_d5);
  Serial.print(" - ");
  Serial.print(r_d6);
  Serial.print(" - ");
  Serial.print(r_d7);
  Serial.print(" - ");
  Serial.print(r_d8);
  Serial.println(" - ");

#endif
}