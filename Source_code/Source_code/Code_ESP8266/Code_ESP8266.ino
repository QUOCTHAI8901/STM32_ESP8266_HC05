#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#define BLYNK_TEMPLATE_ID "TMPL62XEdPdio"
#define BLYNK_DEVICE_NAME "DKTB"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial
#define APP_DEBUG
#define USE_NODE_MCU_BOARD
#include "BlynkEdgent.h"
SoftwareSerial s(3, 1);
void setup()
{
  s.begin(9600);
  delay(100);

  BlynkEdgent.begin(); 
 
}

void loop() {
  BlynkEdgent.run();
  }
  BLYNK_CONNECTED(){
  Blynk.syncAll();
  }
  BLYNK_WRITE(V1){
  int p1=param.asInt();
  if(p1==1) {s.write('A');}
  if(p1==0) {s.write('B');}
}
BLYNK_WRITE(V2){
  int p2=param.asInt();
  if(p2==1) {s.write('C');}
  if(p2==0) {s.write('D');}
}
BLYNK_WRITE(V3){
  int p3=param.asInt();
  if(p3==1) {s.write('c');}
  if(p3==0) {s.write('d');}
}
BLYNK_WRITE(V4){
  int p4=param.asInt();
  if(p4==1) {s.write('e');}
  if(p4==0) {s.write('f');}
}
BLYNK_WRITE(V0){
  int p0=param.asInt();
  if(p0==1) {s.write('a');}
  if(p0==0) {s.write('b');}
}
