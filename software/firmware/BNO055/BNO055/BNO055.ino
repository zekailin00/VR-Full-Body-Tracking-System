#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(115200);
  WiFi.begin("TP-Link_D8B1", "zekailin");
  //WiFi.begin("437 wifi", "437family437");   //WiFi connection
  while (WiFi.status() != WL_CONNECTED) {  //Wait for the WiFI connection completion
    delay(500);
    Serial.println("Waiting for connection");
  }
  Serial.begin(115200);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t  angVelocityData , accelerometerData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  //wifi
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
 
    HTTPClient http;    //Declare object of class HTTPClient
    WiFiClient client;

    //Specify request destination
    //http.begin(client, "http://192.168.1.100:5000/tracker-runtime/GyroAcc1"); //at zekai's home
    http.begin(client, "http://192.168.1.140:5000/tracker-runtime/GyroAcc1"); //xps at zekai's home
    //http.begin(client, "http://10.0.0.160:5000/tracker-runtime/GyroAcc1");      //at hsiuting's home
    http.addHeader("Content-Type", "text/plain");  //Specify content-type header

    //remember to change imu number for each imu, 1.065 scale for difference between BNO055 and LSM6DSOX 
    String imudata = ("imu5," + (String)((-1.129)*accelerometerData.acceleration.x-0.7) + "," \
                              + (String)((-1.129)*accelerometerData.acceleration.y+0.03) + "," \
                              + (String)((1.129)*accelerometerData.acceleration.z-0.56) + "," \
                              + (String)angVelocityData.acceleration.x + "," \
                              + (String)angVelocityData.acceleration.y + "," \
                              + (String)angVelocityData.acceleration.z);
    int httpCode = http.POST(imudata);
    String payload = http.getString();                  //Get the response payload
    Serial.println("return: ");
    Serial.println(httpCode);   //Print HTTP return code
    Serial.println("pay: ");
    Serial.println(payload);    //Print request response payload
    Serial.println(imudata);
    http.end();  //Close connection
  } 
  else {
    Serial.println("Error in WiFi connection");
  }
  delay(33);  //adjust frequency here
}