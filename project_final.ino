//------------------------------------------------------------------------------------
// Wifi and Thingspeak setup:
#include <WiFi.h>            // wifi library

#include "ThingSpeak.h"   // thingspeak library
unsigned long myChannelNumber = 1411153;    // the channel number for upload
const char * myWriteAPIKey = "Y4YIOD6AK7IJSP0V";    // the API key for upload

const char* ssid = "HUJI-guest";     // wifi SSID name
const char* password = "" ;   // wifi pasword
 
const char* server = "api.thingspeak.com";    // website address

WiFiClient client;    

String wifi_status;   // set as string variable
//------------------------------------------------------------------------------------
//pressure setup
#include <Wire.h>   // required for connect to ADS 
#include <Adafruit_ADS1X15.h>   // tensiometer library

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
float pressure_0;   // set as float variable
float voltage_0;    // set as float variable
float pressure_1;   // set as float variable
float voltage_1;    // set as float variable

//------------------------------------------------------------------------------------
//weight setup
#include "HX711.h"    // weight library
 
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 32;   // location of sensor on the breadboard
const int LOADCELL_SCK_PIN = 33;    // location of sensor on the breadboard
float zero;   // set as float variable
float sum;    // set as float variable
float average;    // set as float variable
HX711 scale;    // function from weight library
 

//------------------------------------------------------------------------------------
//LCD setup
#include <Wire.h>     // required for connect to LCD
#include <LiquidCrystal_I2C.h>    // LCD library

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


//**************************************************************************************
void setup() {
  //------------------------------------------------------------------------------------
  // Wifi and Thingspeak setup:
  Serial.begin(9600);   // set serial channel 
  WiFi.disconnect();    // disconnect from previous wifi
  delay(10);    // wait a bit
  WiFi.begin(ssid, password);   // connect to wifi 

  Serial.println();   // print on the serial monitor
  Serial.print("Connecting to ");   // print on the serial monitor
  Serial.println(ssid);   // print on the serial monitor

    ThingSpeak.begin(client);   // start comunication to thingspeak
 
//  WiFi.begin(ssid, password);   //    $$$$$$
  
 
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    wifi_status = "D";    
//    Serial.print(".");
//   }
//  Serial.println("");
//  Serial.print("NodeMcu connected to wifi...");
//  wifi_status = "C";
//  Serial.println(ssid);
//  Serial.println();
    // ^^^ code for debugging, check the internet connection


  //------------------------------------------------------------------------------------
  // pressure:
  Serial.println("Hello!");   // print on the serial monitor
  
  Serial.println("Getting single-ended readings from AIN0..3");   // print on the serial monitor
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");    // print on the serial monitor
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  // We choose ~4V because the voltage of our device is 3.3V, and it's makes higher resolution. 
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV     // 
  
  ads.begin();    // 

  {
    //------------------------------------------------------------------------------------
    //weight
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);   // 
  delay(1000);    // wait a moment for the system
  if (scale.is_ready()) {
    long reading = scale.read();    // reading scale
    
    for (int i=0; i<151; i++){     
        sum +=(reading*0.0094+1344);  // exchange values to gram by calibration equation, some repetition for more precision
        delay(10);}   // wait a bit
    zero = sum / 151;     // find & reset the bias of the weight  
    Serial.println("HX711 found.");   // print on the serial monitor
    
    
  } else {
    Serial.println("HX711 not found.");   // print on the serial monitor
  }
    //------------------------------------------------------------------------------------
    //LCD: 
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
}

  
}

//************************************************************************************** 
void loop() {
  for (int i=0; i<20; i++){     // delay between uploads because thingspeak needs minimum 15 sec delay between updates
    
  //------------------------------------------------------------------------------------
  // pressure:
  int16_t adc0, adc1, adc2, adc3;

  adc0 = ads.readADC_SingleEnded(0);    // read pins of ADS // pin of first pressure sensor
  adc1 = ads.readADC_SingleEnded(1);    // read pins of ADS // pin of second pressure sensor
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  voltage_0 =(adc0 * 0.125 * 2) / 1000;    // divided by 1000 to move from mV to V; 0.125 from SetGain; 2 from voltage divider;
  pressure_0 = (voltage_0/5 - 0.04) / 0.009;    // formula to exchange voltage to pressure according to the information of the sensor 
  Serial.print("Pressure_0: "); Serial.print(pressure_0);  Serial.println("kPa ");    // print on the serial monitor
  voltage_1 =(adc1 * 0.125 * 2) / 1000; // divided by 1000 to move from mV to V; 0.125 from SetGain; 2 from voltage divider;
  pressure_1 = (voltage_1/5 - 0.04) / 0.009;    // formula to exchange voltage to pressure according to the information of the sensor 
  Serial.print("Pressure_1: "); Serial.print(pressure_1);  Serial.println("kPa ");    // print on the serial monitor
  Serial.println(" ");
  
  delay(500);   // wait a bit

  //------------------------------------------------------------------------------------
  //weight
  if (scale.is_ready()) {
    long reading = scale.read();    // reading scale
    sum = 0;
    for (int i=0; i<51; i++){     
        sum +=(reading*0.0094+1344);    // exchange values to gram by calibration equation
        delay(10);}
    average = sum / 51;   // some repetition for more precision
    Serial.print("Weight: ");Serial.print(average-zero); Serial.println("g");   //  upload to serial  // reduction of the bias
  } else {
    Serial.println("HX711 not found.");
  }
  delay(500);   // wait a bit


  //------------------------------------------------------------------------------------
  //LCD setup:
 // when characters arrive over the serial port...
    
    delay(100);   // wait a bit for the entire message to arrive
    lcd.clear();    // clear the screen
    {lcd.setCursor(0,0);
    lcd.print(pressure_0);
    lcd.setCursor(6,0);
    lcd.print(pressure_1);
    lcd.setCursor(12,0);
    lcd.print(average-zero);
    lcd.setCursor(1,1);
    lcd.print("p_0");
    lcd.setCursor(7,1);
    lcd.print("p_1");
    lcd.setCursor(13,1);
    lcd.print("we");}   // display each parameter or character to the LCD
    delay (500);    // wait a bit
  }
      //------------------------------------------------------------------------------------
  
  
  //Thingspeak setup:
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    wifi_status = "D";

    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(1000);    // wait a bit
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifi_status = "C";
  }
  // ^^^ verify the integrity of wifi connection
  
  ThingSpeak.setField(1,average-zero);    // upload weight to thingspeak
  ThingSpeak.setField(2,pressure_0);    // upload pressure0 to thingspeak
  ThingSpeak.setField(3,pressure_1);    // upload pressure1 to thingspeak
  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);   // location for uploads

  Serial.println("uploaded to Thingspeak server....");    // print on the serial monitor

  client.stop();    
 
  Serial.println("Waiting to upload next reading...");    // print on the serial monitor
  Serial.println();

  
  
}
