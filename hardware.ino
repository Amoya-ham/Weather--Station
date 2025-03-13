//##################################################################################################################
//##                                      ELET2415 DATA ACQUISITION SYSTEM CODE                                   ##
//##                                                                                                              ##
//##################################################################################################################

// LIBRARY IMPORTS
#include <rom/rtc.h> 
#include <math.h>  // https://www.tutorialspoint.com/c_standard_library/math_h.htm 
#include <ctype.h>
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"

//############### IMPORT HEADER FILES ##################

#ifndef _WIFI_H 
#include <WiFi.h>
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef STDLIB_H
#include <stdlib.h>
#endif

#ifndef ARDUINO_H
#include <Arduino.h>
#endif 
 
#ifndef ARDUINOJSON_H
#include <ArduinoJson.h>
#endif

#ifndef FORECAST_H
#include "foreCast.h"
#endif

 
// DEFINE AND INITIALISE VARIABLES
#define ARDUINOJSON_USE_DOUBLE      1 

// DEFINE THE CONTROL PINS FOR SOIL MOISTURE SENSOR
#define soil_m    32

// DEFINE THE CONTROL PINS FOR TFT DISPLAY
#define TFT_CS    5
#define TFT_RST   17
#define TFT_DC    16
#define TFT_MOSI  23
#define TFT_CLK   18
#define TFT_MISO  19

// IMPORT FONTS FOR TFT DISPLAY
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h> 

// DEFINE COLOURS FOR TFT
#define BLACK        0x0000  
#define WHITE        0xFFFF  
#define RED          0xF800  
#define GREEN        0x07E0  
#define BLUE         0x001F  
#define CYAN         0x07FF  
#define MAGENTA      0xF81F  
#define YELLOW       0xFFE0  
#define ORANGE       0xFD20  
#define PURPLE       0x780F  
#define PINK         0xF81F  
#define GRAY         0x8410  
#define DARK_GRAY    0x4208  
#define BROWN        0xA145 
#define CLOUDY		   0xC618	
#define Rainy		     0x051F	
#define Stormy		   0x3800	
#define Foggy		     0xBDF7	
#define NIGHT     	 0x000F	
#define SUNSET		   0xFC60	
#define Midnight     0x0018	
#define Star         0xFFFF	
#define Moon         0xFFE0	
#define Nebula 	     0x780F	
#define Glow         0x07BF	
#define Cloud     	 0x4208	
#define APPLE_BLUE     0x007AFF  // Main UI elements
#define SOFT_BLACK     0x222222  // Text on light backgrounds
#define LIGHT_GRAY     0xD4D4D4  // UI separators
#define DARK_GRAY      0x666666  // Secondary text
#define HIGHLIGHT_CYAN 0x32AADC
#define MIDNIGHT_BLUE  0x0018    // Slightly lighter than Night Blue
#define STAR_WHITE     0xFFFF    // Stars, highlights
#define MOON_YELLOW    0xFFE0    // Moon details
#define NEBULA_PURPLE  0x780F    // Cosmic glow
#define GLOW_BLUE      0x07BF    // Subtle UI glow
#define CLOUD_GRAY     0x4208
#define SKY_BLUE       0x87CEEB  // Clear daytime sky
#define DEEP_BLUE      0x003366  // Water, gradients
#define ELECTRIC_YELLOW 0xFFD700 // Lightning, bright highlights
#define MIST_GRAY      0xD3D3D3  // Subtle fog effects
#define ICY_BLUE       0xB0E0E6
#define Navy           0x000F



// DEFINE THE CONTROL PINS FOR THE DHT22 
#define DHTPIN 33
#define DHTTYPE DHT22

// DEFINE THE CONTROL PINS FOR THE BMP280
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)


// MQTT CLIENT CONFIG  
static const char* pubtopic      = "620161390";                    // Add your ID number here
static const char* subtopic[]    = {"620161390_sub","/elet2415"};  // Array of Topics(Strings) to subscribe to
static const char* mqtt_server   = "broker.emqx.io";         // Broker IP address or Domain name as a String

static uint16_t mqtt_port        = 1883;

// WIFI CREDENTIALS
const char* ssid       = "MonaConnect";     // Add your Wi-Fi ssid
const char* password   = "";    // Add your Wi-Fi password 

// TASK HANDLES 
TaskHandle_t xMQTT_Connect          = NULL; 
TaskHandle_t xNTPHandle             = NULL;  
TaskHandle_t xLOOPHandle            = NULL;  
TaskHandle_t xUpdateHandle          = NULL;
TaskHandle_t xButtonCheckeHandle    = NULL;   

// FUNCTION DECLARATION   
void checkHEAP(const char* Name);   // RETURN REMAINING HEAP SIZE FOR A TASK
void initMQTT(void);                // CONFIG AND INITIALIZE MQTT PROTOCOL
unsigned long getTimeStamp(void);   // GET 10 DIGIT TIMESTAMP FOR CURRENT TIME
void callback(char* topic, byte* payload, unsigned int length);
void initialize(void);
bool publish(const char *topic, const char *payload); // PUBLISH MQTT MESSAGE(PAYLOAD) TO A TOPIC
void vButtonCheck( void * pvParameters );
void vUpdate( void * pvParameters );
 

/* Declare your functions below */ 
void display();
double HeatIndex(double temperature , double humidity );
double MoistureValue();

#ifndef NTP_H
#include "NTP.h"
#endif

#ifndef MQTT_H
#include "mqtt.h"
#endif

/* Init class Instances for the DHT22 etcc */
DHT dht(DHTPIN, DHTTYPE);  

double temperature;  
double humidity;   
double BMPtemp;  //temperature read by the BMP sensor
int SoilConValue; //value of the soil moisture sensor readings after conversion
int SoilUnConValue;//value of soil moisture before conversion
double CalHeatIndex;  //heat index calculated using temperature and humidity

/* Initialize class objects*/
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);  // INIT SERIAL   

  // INITIALIZE ALL SENSORS AND DEVICES
  /* TFT DISPLAY SET UP */
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_NAVY);
  for (int i = 0; i < 30; i++) {  // Adjust number of stars as needed
    int x = random(tft.width());  
    int y = random(tft.height());  
    tft.drawPixel(x, y, ILI9341_WHITE);  // White stars
  }
  // Draw Moon (top-left)
  tft.fillCircle(30, 30, 15, ILI9341_YELLOW);  // Moon shape
  tft.fillCircle(35, 25, 5, ILI9341_LIGHTGREY);  // Crater 1
  tft.fillCircle(25, 35, 4, ILI9341_LIGHTGREY );  // Crater 2

  // Draw Shooting Star (top area)
  tft.drawLine(220, 20, 250, 5, ILI9341_WHITE);  // Shooting star tail
  tft.fillCircle(250, 5, 3, ILI9341_WHITE);  // Shooting star head

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(40,100);
  tft.println("ELECT 2415");
  tft.setCursor(20,130);
  tft.println("WEATHER STATION");
  delay(2000);
  
  
  /* Add all other necessary sensor Initializations and Configurations here */
  dht.begin();
  Wire.begin(21, 22);
  bmp.begin(0x76);
  pinMode(DHTPIN, INPUT);
  pinMode(soil_m, INPUT);
  initialize();     // INIT WIFI, MQTT & NTP 
  // vButtonCheckFunction(); // UNCOMMENT IF USING BUTTONS INT THIS LAB, THEN ADD LOGIC FOR INTERFACING WITH BUTTONS IN THE vButtonCheck FUNCTION
}
  


void loop() {
  // put your main code here, to run repeatedly:
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

  Serial.println("***************My Weather Station****************");
  temperature  = dht.readTemperature();
  Serial.print(F("DHT Temperature: "));
  Serial.print(temperature );
  Serial.println(" *C");

  Serial.print(F("BMP Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  humidity = dht.readHumidity();
  Serial.print("Humidity =  ");
  Serial.print(humidity);
  Serial.println("%");
          
  CalHeatIndex = HeatIndex(temperature , humidity);
  Serial.print("Heat Index =  ");
  Serial.println(CalHeatIndex);

  SoilConValue = MoistureValue();
  Serial.print(F("Soil Moisture = "));
  Serial.print(SoilConValue);
  Serial.println("%");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure()/100);
  Serial.println(" hPa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println("************************************************");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  display();

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}


//####################################################################
//#                          UTIL FUNCTIONS                          #       
//####################################################################
// void vButtonCheck( void * pvParameters )  {
//     configASSERT( ( ( uint32_t ) pvParameters ) == 1 );     
      
//     for( ;; ) {
//         // Add code here to check if a button(S) is pressed
//         // then execute appropriate function if a button is pressed 
//         vTaskDelay(200 / portTICK_PERIOD_MS);  
//     }
// }

void vUpdate( void * pvParameters )  {
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );    
           
    for( ;; ) {
      // #######################################################
      // ## This function must PUBLISH to topic every second. ##
      // #######################################################
   
      double humidity  = dht.readHumidity();
      double temperature  = dht.readTemperature();
      double BMPtemp = bmp.readTemperature();
      double pressure  = bmp.readPressure();
      double altitude  = bmp.readAltitude(1013.25);

      // 1. Create JSon object
      StaticJsonDocument<1000> doc;
      // 2. Create message buffer/array to store serialized JSON object
      char message[1100]  = { 0 };
      // 3. Add key:value pairs to JSon object based on above schema
      doc["id"]                 = "620161390";
      doc["timestamp"]          = getTimeStamp();
      doc["celsTemperature"]    = temperature;
      doc["fahrTemperature"]    = (temperature  * 9.0/5.0) + 32;
      doc["bmp_temp"]           = BMPtemp;
      doc["humidity"]           = humidity;
      doc["heatindex"]          = HeatIndex(temperature , humidity);
      doc["soilMoisture"]       = MoistureValue();
      doc["pressure"]           = pressure;
      doc["altitude"]           = altitude;

      // 4. Seralize / Covert JSon object to JSon string and store in message array
      serializeJson(doc, message); 
      Serial.println(message);
      // 5. Publish message to a topic subscribed to by both backend and frontend                
      if(mqtt.connected()){
        publish(pubtopic, message);
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }  
     
}

unsigned long getTimeStamp(void) {
          // RETURNS 10 DIGIT TIMESTAMP REPRESENTING CURRENT TIME
          time_t now;         
          time(&now); // Retrieve time[Timestamp] from system and save to &now variable
          return now;
}

void callback(char* topic, byte* payload, unsigned int length) {
  // ############## MQTT CALLBACK  ######################################
  // RUNS WHENEVER A MESSAGE IS RECEIVED ON A TOPIC SUBSCRIBED TO
  
  Serial.printf("\nMessage received : ( topic: %s ) \n",topic ); 
  char *received = new char[length + 1] {0}; 
  
  for (int i = 0; i < length; i++) { 
    received[i] = (char)payload[i];    
  }

  // PRINT RECEIVED MESSAGE
  Serial.printf("Payload : %s \n",received);

 
  // CONVERT MESSAGE TO JSON
  StaticJsonDocument<1000> doc;
  DeserializationError error = deserializeJson(doc, received);  

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }


  // PROCESS MESSAGE
  const char* type = doc["type"]; 

  if (strcmp(type, "sensors") == 0){
    // 1. EXTRACT ALL PARAMETERS: NODES, RED,GREEN, BLUE, AND BRIGHTNESS FROM JSON OBJECT
    int ctemp       = doc["celsTemperature"];
    int ftemp       = doc["fahrTemperature"];
    int btemp       = doc["bmp_temp"];
    int humid       = doc["humidity"];
    int heatIndex   = doc["heatindex"];
    int soilmoist   = doc["soilMoisture"];
    int pressure    = doc["pressure"];
    int altitude    = doc["altitude"];
  }
}

bool publish(const char *topic, const char *payload){   
    bool res = false;
    try{
      res = mqtt.publish(topic,payload);
      // Serial.printf("\nres : %d\n",res);
      if(!res){
        res = false;
        throw false;
      }
    }
    catch(...){
      Serial.printf("\nError (%d) >> Unable to publish message\n", res);
    }
  return res;
}



// //***** Complete the util functions below ******

void display(){

   // Set display rotation
  tft.setRotation(1);
  
  // NIGHT SKY BACKGROUND (Only drawn once at setup)
  static bool initialized = false;
  if (!initialized) {
    tft.fillScreen(NIGHT);
    
    // Draw stars (random small white dots)
    for (int i = 0; i < 20; i++) {
      tft.drawPixel(random(0, tft.width()), random(0, tft.height()/2), STAR_WHITE);
    }
    
    // Draw Moon (simple circle for now)
    tft.fillCircle(30, 30, 15, MOON_YELLOW);
    
    // Draw Static Labels
    tft.setTextColor(APPLE_BLUE);
    tft.setTextSize(2);
    
    tft.setCursor(15, 90); tft.println("DHT TEMP:");
    tft.setCursor(15, 115); tft.println("BMP TEMP:");
    tft.setCursor(15, 140); tft.println("HUMIDITY:");
    tft.setCursor(15, 165); tft.println("PRESSURE:");
    tft.setCursor(15, 190); tft.println("SOIL MOISTURE:");
    tft.setCursor(15, 215); tft.println("ALTITUDE:");
    
    initialized = true;
  }
  
  // Update only values
  tft.setTextColor(SUNSET, NIGHT);
  tft.setTextSize(2);
  
  tft.fillRect(140, 90, 50, 20, NIGHT);
  tft.setCursor(140, 90); tft.print(temperature); tft.setTextSize(1); tft.print("o"); tft.setTextSize(2); tft.print("C");
  
  tft.fillRect(140, 115, 50, 20, NIGHT);
  tft.setCursor(140, 115); tft.print(bmp.readTemperature()); tft.setTextSize(1); tft.print("o"); tft.setTextSize(2); tft.print("C");
  
  tft.fillRect(140, 140, 50, 20, NIGHT);
  tft.setCursor(140, 140); tft.print(humidity); tft.print("%");
  
  tft.fillRect(140, 165, 80, 20, NIGHT);
  tft.setCursor(140, 165); tft.print(bmp.readPressure()/100); tft.print(" hPa");
  
  tft.fillRect(180, 190, 50, 20, NIGHT);
  tft.setCursor(180, 190); tft.print(SoilConValue); tft.print("%");
  
  tft.fillRect(140, 215, 80, 20, NIGHT);
  tft.setCursor(140, 215); tft.print(bmp.readAltitude(1013.25)); tft.print(" m");
}




double HeatIndex(double temperature , double humidity ){
    // CALCULATE AND RETURN HEAT INDEX USING EQUATION FOUND AT https://byjus.com/heat-index-formula/#:~:text=The%20heat%20index%20formula%20is,an%20implied%20humidity%20of%2020%25

  double ft = (temperature  * 9.0/5.0) + 32;
  double H_I = -42.379 + (2.04901523 * ft) + (10.14333127 * humidity ) + (-0.22475541 * ft * humidity) + (-0.00683783 * pow(ft, 2))  + (-0.05481717 * pow(humidity ,2)) + (0.00122874 * pow(ft, 2) * humidity)  + (0.00085282 * ft * pow(humidity ,2)) + (-0.00000199 * pow(ft, 2) * pow(humidity,2));
  CalHeatIndex =  (5.0/9.0) * (H_I - 32);
  return CalHeatIndex;
}

double MoistureValue(){    
  SoilUnConValue = analogRead(soil_m);
  SoilConValue = map(SoilUnConValue, 1500, 3900, 100, 0);
  if (SoilConValue < 0)
  {
    SoilConValue = 0;
  }
  else if (SoilConValue >= 100)
  {
    SoilConValue = 100;
  }  
  return SoilConValue; 
}