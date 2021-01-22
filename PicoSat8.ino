#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <I2Cdev.h>
#include <BMP085.h>
#include <Wire.h>
#include <MPU6050.h>
#include <avr/wdt.h>

#define RFM95_CS 10 
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 915.0
#define DHTPIN 6
#define DHTTYPE DHT22
#define A_R 16384.0
#define G_R 131.0
#define RAD_A_DEG = 57.295779
#define MPU 0x68

SoftwareSerial SerialGPS(5,6);
TinyGPSPlus gps;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
DHT_Unified dht(DHTPIN, DHTTYPE);
BMP085 barometer;

float Acc[2];
float Gy[3];
float Angle[3];
long tiempo_prev;
float dt;

struct telemetria{
  float latitud, longitud, velocidad, curso, temperatura, humedad, temperatura2, presion, agx, agy, agz;
  int hora, minuto, segundo;
  int16_t ax, ay, az, gx, gy, gz;
  int32_t altitud, altitud2;
} sat;

void setup() {
  wdt_disable();
  Serial.begin(9600);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(924.68)) {
    Serial.println(F("setFrequency failed"));
    while (1);
  }
  rf95.setTxPower(23, false);
  SerialGPS.begin(9600);
  SerialGPS.println("$PMTK001,886,3*36");
  dht.begin();
  barometer.initialize();
  Serial.println(barometer.testConnection() ? "GY-81 conexión exitosa" : "GY-81 conexión fallida");
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  MPU6050 sensor;
  sensor.setXAccelOffset(-1063);
  sensor.setYAccelOffset(690);
  sensor.setZAccelOffset(961);
  sensor.setXGyroOffset(-53);
  sensor.setYGyroOffset(-3);
  sensor.setZGyroOffset(11);   
  setWDT();
}

void loop() {
  sensorDHT();
  sensorBMP085();
  sensorMPU6050();
  if(SerialGPS.available()){
    sensorGPS();
    //imprimirCadenaITA();
    imprimirCadena1();
    imprimirCadena2();
    wdt_reset();
  }
}

void sensorGPS(){
  while(SerialGPS.available()){
   if(gps.encode(SerialGPS.read())){
    obtenerDatosGPS(gps);
   }
  }
}

void obtenerDatosGPS(TinyGPSPlus &gps)
{
  sat.latitud = gps.location.lat();
  sat.longitud = gps.location.lng();
  sat.hora = gps.time.hour();
  sat.minuto = gps.time.minute();
  sat.segundo = gps.time.second();
  sat.altitud = gps.altitude.meters();
  sat.velocidad = gps.speed.kmph();
  sat.curso = gps.course.deg();
}

void sensorDHT(){
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    sat.temperatura = sat.temperatura;
  }
  else {
    sat.temperatura = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    sat.humedad = sat.humedad;
  }
  else {
    sat.humedad = event.relative_humidity;
  }
}

void sensorMPU6050(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);
  sat.ax=Wire.read()<<8|Wire.read();
  sat.ay=Wire.read()<<8|Wire.read();
  sat.az=Wire.read()<<8|Wire.read();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);
  sat.gx=Wire.read()<<8|Wire.read();
  sat.gy=Wire.read()<<8|Wire.read();
  sat.gz=Wire.read()<<8|Wire.read();
  angulo();
}

void sensorBMP085(){
  barometer.setControl(BMP085_MODE_TEMPERATURE);
  sat.temperatura2 = barometer.getTemperatureC();
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  sat.presion = barometer.getPressure();
  sat.altitud2 = barometer.getAltitude(sat.presion);
}

void angulo(){
  Acc[1] = atan(-1*(sat.ax/A_R)/sqrt(pow((sat.ay/A_R),2) + pow((sat.az/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((sat.ay/A_R)/sqrt(pow((sat.ax/A_R),2) + pow((sat.az/A_R),2)))*RAD_TO_DEG;
  Gy[0] = sat.gx/G_R;
  Gy[1] = sat.gy/G_R;
  Gy[2] = sat.gz/G_R;
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];
  Angle[2] = Angle[2]+Gy[2]*dt;
  if(Angle[2] > 360)
    Angle[2] = 0;
  else if(Angle[2] < 0)
    Angle[2] = 360;
  sat.agx = Angle[0];
  sat.agy = Angle[1];
  sat.agz = Angle[2];
}

void imprimirCadena1(){
  String cadena = "";
  cadena = "0;" + String(sat.latitud, 6) + ",1;";
  cadena += String(sat.longitud, 6) + ",2;";
  cadena += String(sat.altitud) + ",3;";
  cadena += String(sat.velocidad, 2) + ",4;";
  cadena += String(sat.curso, 2) + ",5;";
  cadena += String(sat.temperatura, 1) + ",6;";
  cadena += String(sat.humedad, 2) + ",7;";
  cadena += String(sat.agx, 2) + ",8;";
  cadena += String(sat.agy, 2) + ",9;";
  cadena += String(sat.agz, 2) + ' ';

  char todoch[cadena.length()+1];
  cadena.toCharArray(todoch,cadena.length());
  Serial.println(todoch);
  rf95.send((uint8_t *)todoch,cadena.length());
  cadena = "";
}

void imprimirCadena2(){
  String cadena = "";
  cadena =  "10;" + String(sat.ax) + ",11;";
  cadena += String(sat.ay) + ",12;";
  cadena += String(sat.az) + ",13;";
  cadena += String(sat.gx) + ",14;";
  cadena += String(sat.gy) + ",15;";
  cadena += String(sat.gz) + ",16;";
  cadena += String(sat.presion * 0.01, 0) + ",17;";
  cadena += String(sat.temperatura2, 1) + ",18;";
  cadena += String(sat.altitud2) + ",19;";
  cadena += String(analogRead(A0)) + ' ';
  
  char todoch[cadena.length()+1];
  cadena.toCharArray(todoch,cadena.length());
  Serial.println(todoch);
  rf95.send((uint8_t *)todoch,cadena.length());
  cadena = "";
}

/*void imprimirCadenaITA(){
  String cadena = "";
  cadena = "ITC," + String(sat.temperatura, 2) + ',';
  cadena +=  String(sat.humedad, 2) + ',';
  cadena += String(sat.presion * 0.01, 2) + ',' ;
  cadena += String(sat.temperatura, 2) + ",0,0,0,"; 
  cadena += String(sat.ax) + ',' + String(sat.ay) + ',' + String(sat.az) + ',';
  cadena += String(sat.gx) + ',' + String(sat.gy) + ',' + String(sat.gz) + ',';
  cadena += String(sat.latitud, 6) + ',';
  cadena += String(sat.longitud, 6) + ',';
  cadena += String(sat.altitud) + ',';
  cadena += String(sat.velocidad, 2) + ',';

  char todoch[cadena.length()+1];
  cadena.toCharArray(todoch,cadena.length());
  Serial.println(todoch);
  rf95.send((uint8_t *)todoch,cadena.length());
}*/

void setWDT() {
   WDTCSR |= 0b00011000;
   WDTCSR = 0b00001000 |  0b01000111;
   wdt_reset();
}
