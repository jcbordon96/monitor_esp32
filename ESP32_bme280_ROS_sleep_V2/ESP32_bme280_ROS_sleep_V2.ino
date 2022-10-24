#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <WiFi.h>
#include <avisense/Measurement.h>
#include <std_msgs/String.h>
#include "time.h"
#include <esp_sleep.h>
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_MEASURE  60     //60seg funcionamiento en campo
#define TIME_TO_SEND 300        //300seg funcionamiento en campo
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define DEBUG 1
Adafruit_BME280 bme;
RTC_DATA_ATTR bool connect_request = true;  
RTC_DATA_ATTR bool first_connect = true;  
RTC_DATA_ATTR float sum_temperature = 0;
RTC_DATA_ATTR float sum_humidity = 0;
RTC_DATA_ATTR float sum_pressure = 0;
RTC_DATA_ATTR int len_temperature = 0;
RTC_DATA_ATTR int len_humidity = 0;
RTC_DATA_ATTR int len_pressure = 0;
RTC_DATA_ATTR int last_send_time = 0;
const char* ssid = "AVISenseNetwork"; //G8
//const char* ssid = "AVISenseNetwork G7";
//const char* ssid = "Apelie";         //office
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
int counter = 0;
String load_confirm = "";
bool status;
long last_measure = 0;
long last_send = 0;
int first_wifi = 0;
long send_time_elapsed = 0;

//battery voltage input
float voltage = 0.0;
float voltageRead = 0.0;
float voltageDividerFactor = 2.4;  // 005=2.5, 006=2.4, 007=, 008=

void listenCallback( const std_msgs::String& msg){
  load_confirm= msg.data;
}

//ROS
ros::NodeHandle nh;
avisense::Measurement measure_msg;
ros::Publisher measurement_pub("/monitor_measurement/load", &measure_msg);
ros::Subscriber<std_msgs::String> sub("/monitor_measurement/confirmation", &listenCallback );
IPAddress server(192,168,150,102);      // ROSMASTER IP
//IPAddress server(192,168,0,120);      // ROSMASTER IP OFFICE

const uint16_t serverPort = 11411;  
char time_stamp[50];
char monitor_id[] = "5";
void setupWiFi() {                    // connect to ROS server as as a client
  if(DEBUG){
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
  }
  first_wifi = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    first_wifi += 1;
    if(first_wifi>60){
      esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
      break;
    }
    
  }
  if(DEBUG && first_wifi<240){
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
  //
  voltageRead = analogRead(33);
  voltage = (voltageRead / 4095.0) * 3.3 * (4.2 / voltageDividerFactor);
  Serial.println("Batería: " + String(voltage));
  //
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
  last_send_time += TIME_TO_MEASURE;
  Serial.println("Falta para enviar: " +String(TIME_TO_SEND-last_send_time) + " segundos");
  if (last_send_time >= TIME_TO_SEND || first_connect == true){  
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(measurement_pub);
    nh.subscribe(sub);
    nh.spinOnce();
    first_connect = false;
    /*measure_msg.header.frame_id = monitor_id;
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

    nh.spinOnce();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("A mimir por " + String(TIME_TO_MEASURE) + " segundos");
    esp_sleep_enable_timer_wakeup(TIME_TO_MEASURE * uS_TO_S_FACTOR);
    esp_deep_sleep_start();*/
  }
  else{
  Serial.println("A mimir por " + String(TIME_TO_MEASURE) + " segundos");
  esp_sleep_enable_timer_wakeup(TIME_TO_MEASURE * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
  }
  

  

  
}

void loop(){
  nh.spinOnce();
  if(counter>10){
    if(millis() - send_time_elapsed > 60000){
      send_time_elapsed = 0;
      Serial.println("Reinicio porque nunca me confirmaron");
      esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    Serial.println("Voy a enviar");
    if(String(load_confirm) == monitor_id){
      
      load_confirm = "";
      Serial.println("Confirmado recepcion");
      last_send_time = 0;
      Serial.println("A mimir por " + String(TIME_TO_MEASURE) + " segundos");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      esp_sleep_enable_timer_wakeup(TIME_TO_MEASURE * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
  
    }
    measure_msg.header.frame_id = monitor_id;
    measure_msg.header.stamp = nh.now();
    //
    measure_msg.temp_surface = voltage;
    //
    measure_msg.temp_environment = prom_temperature;
    measure_msg.humidity = prom_humidity;
    measurement_pub.publish(&measure_msg);
    delay(1000);

    sum_temperature = 0;
    len_temperature = 0;
    sum_humidity = 0;
    len_humidity = 0;
    sum_pressure = 0;
    len_pressure = 0;
    prom_temperature = 0;
    prom_humidity = 0;
    prom_pressure = 0;
    counter = 0;

    
    

  }
  delay(500);
  counter +=1;
  Serial.println(counter);
  
    

}
