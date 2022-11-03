#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  //for wifi
  WiFi.begin("437 wifi", "437family437");   //WiFi connection
 
  while (WiFi.status() != WL_CONNECTED) {  //Wait for the WiFI connection completion
    delay(500);
    Serial.println("Waiting for connection");
  }
}

void loop(void) 
{
  /* Get a new sensor event */ 
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  /* Display the floating point data */
  Serial.print("Xacc: ");
  Serial.print(acc.x());
  Serial.print(" Yacc: ");
  Serial.print(acc.y());
  Serial.print(" Zacc: ");
  Serial.print(acc.z());
  Serial.println("");
  Serial.print("Xgyro: ");
  Serial.print(gyro.x());
  Serial.print(" Ygyro: ");
  Serial.print(gyro.y());
  Serial.print(" Zgyro: ");
  Serial.print(gyro.z());
  Serial.println("");

  delay(100);
  //wifi
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
 
    HTTPClient http;    //Declare object of class HTTPClient
    WiFiClient client;
    
    http.begin(client, "http://10.0.0.160:5000/tracker-runtime/GyroAcc1");      //Specify request destination
    http.addHeader("Content-Type", "text/plain");  //Specify content-type header
    String test = ("imu1," + (String)acc.x() + "," + (String)acc.y() + "," + (String)acc.z() + "," + (String)gyro.x() + "," + (String)gyro.y() + "," + (String)gyro.z());
    int httpCode = http.POST(test);
    String payload = http.getString();                  //Get the response payload
    Serial.println("return: ");
    Serial.println(httpCode);   //Print HTTP return code
    Serial.println("pay: ");
    Serial.println(payload);    //Print request response payload
    http.end();  //Close connection
  } 
  else {
    Serial.println("Error in WiFi connection");
  }
  delay(3000);  //Send a request every 30 seconds
}