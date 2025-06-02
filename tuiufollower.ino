#include <driver/adc.h>

#define ir 21

//H-bridge
#define a1a 5
#define a1b 17
#define b1a 19
#define b1b 18

#define DEBUG 0

void setup() {

  Serial.begin(115200);

  adc1_config_width(ADC_WIDTH_BIT_12);

  //Pin 32
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
  //Pin 33
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

  //Pin 13
  adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_DB_11);
  //Pin 12
  adc2_config_channel_atten(ADC2_CHANNEL_5, ADC_ATTEN_DB_11);
  //Pin 14
  adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_11);
  //Pin 27
  adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11);
  //Pin 26
  adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_11);
  //Pin 25
  adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_11);

  pinMode(ir, OUTPUT);
  digitalWrite(ir, HIGH);

  pinMode(a1a, OUTPUT);
  pinMode(a1b, OUTPUT);
  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);
}

void loop() {

  int r_d1, r_d2, r_d3, r_d4, r_d5, r_d6;
  esp_err_t r = adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_12Bit, &r_d1);
  r = adc2_get_raw(ADC2_CHANNEL_5, ADC_WIDTH_12Bit, &r_d2);
  r = adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_12Bit, &r_d3);
  r = adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_12Bit, &r_d4);
  r = adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_12Bit, &r_d5);
  r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &r_d6);
  int r_d7 = adc1_get_raw(ADC1_CHANNEL_5);
  int r_d8 = adc1_get_raw(ADC1_CHANNEL_4);


  /*
  digitalWrite(a1a, HIGH);
  digitalWrite(a1b, LOW);
  digitalWrite(b1a, HIGH);
  digitalWrite(b1b, LOW);
  */


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