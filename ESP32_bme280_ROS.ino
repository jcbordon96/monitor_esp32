#include <ros.h>
#include <WiFi.h>
#include <avisense/Measurement.h>
#include <std_msgs/Bool.h>
#include "time.h"
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_MEASURE  30000
#define TIME_TO_SEND 300000
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define DEBUG 1
#define ROSSERIAL_ARDUINO_TCP
Adafruit_BME280 bme;
float sum_temperature = 0;
float sum_humidity = 0;
float sum_pressure = 0;
int len_temperature = 0;
int len_humidity = 0;
int len_pressure = 0;
int last_send_time = 0;
const char* ssid = "AVISenseNetwork";
//const char* ssid = "AVISenseNetwork G7";
const char* password = "apelie2022";
const char* ntpServer = "south-america.pool.ntp.org";
const long gmtOffset_sec = -3*3600;
const int daylightOffset_sec = 0;
int last_measure_time = 0;
float temperature = 0;
float humidity = 0;
float pressure = 0;
float prom_temperature = 0;
float prom_humidity = 0;
float prom_pressure = 0;
bool load_confirm = false;
bool status;
long last_measure = 0;
long last_send = 0;
void listenCallback( const std_msgs::Bool& msg){
  load_confirm= msg.data;
}

//ROS
ros::NodeHandle nh;
avisense::Measurement measure_msg;
ros::Publisher measurement_pub("/monitor_measurement/load", &measure_msg);
ros::Subscriber<std_msgs::Bool> sub("/monitor_measurement/confirm", &listenCallback );
IPAddress server(192,168,150,102);      // ROSMASTER IP
const uint16_t serverPort = 11411;  
char time_stamp[50];
char monitor_id[] = "1";
void setupWiFi() {                    // connect to ROS server as as a client
  if(DEBUG){
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  if(DEBUG){
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }
  
}

void setup(){
  Serial.begin(115200);
  status = bme.begin(0x76);  
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(measurement_pub);
  nh.subscribe(sub);
  

  

  
}

void loop(){
  nh.spinOnce();
  if(millis()-last_measure > TIME_TO_MEASURE){
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure();
    Serial.println("Temperatura:" + String(temperature) + "/Humedad:" +  String(humidity) + "/Presion:" +  String(pressure));
    sum_temperature += temperature;
    len_temperature++;
    sum_humidity += humidity;
    len_humidity++;
    sum_pressure += pressure;
    len_pressure++;
    prom_temperature = sum_temperature/len_temperature;
    prom_humidity = sum_humidity/len_humidity;
    prom_pressure = sum_pressure/len_pressure;
    Serial.println("Promedios: Temperatura:" + String(prom_temperature) + "/Humedad:" +  String(prom_humidity) + "/Presion:" +  String(prom_pressure));
    last_measure = millis();
   if(WiFi.status() != WL_CONNECTED){
      Serial.println("WiFi desconectado");
      setupWiFi();
    }
  }
  
  if (millis()-last_send > TIME_TO_SEND){
    Serial.println("Voy a enviar data");
    struct tm timeinfo;
    if(nh.connected()){
      measure_msg.header.frame_id = monitor_id;
      measure_msg.header.stamp = nh.now();
      measure_msg.temp_environment = prom_temperature;
      measure_msg.humidity = prom_humidity;
      measurement_pub.publish(&measure_msg);
      delay(100);
      last_send_time = 0;
      sum_temperature = 0;
      len_temperature = 0;
      sum_humidity = 0;
      len_humidity = 0;
      sum_pressure = 0;
      len_pressure = 0;
      prom_temperature = 0;
      prom_humidity = 0;
      prom_pressure = 0;
      last_send = millis();
    }
    else{
      Serial.println("Fallo conectarse a ros");
      delay(1000);
      if(WiFi.status() != WL_CONNECTED){
        Serial.println("WiFi desconectado");
        setupWiFi();
      }
      else{
        Serial.println("WiFi esta conectado");
      }
      
//      nh.getHardware()->setConnection(server, serverPort);
//      nh.initNode();
//      nh.advertise(measurement_pub);
//      nh.subscribe(sub);
      
    }
   
  }

}
