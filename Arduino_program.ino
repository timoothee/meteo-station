#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <DFRobot_BMP3XX.h>
#include <LiquidCrystal_PCF8574.h>
#include <RTC.h>
#include <Wire.h>

/*******************************************************
* user data types area
********************************************************/
typedef struct RTC_initData {
  /* initial time */
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  /* initial date */
  uint8_t day;
  uint8_t month;
  uint16_t year;
} RTC_Data_Type;

typedef struct
{
  float Temp;
  float Humid;
  float Press;
  float Wind_speed;
  char Wind_dir[2];
  uint8_t Light;
  RTC_Data_Type Watch;
} InternalData_type;

/*******************************************************
* variables area
********************************************************/

/* Internal data handler  */
InternalData_type Data_Hdl;


const char* ssid = "iPhone X";              //replace with your ssid
const char* password = "012345678";          //replace with your pw
String thingName = "MeteoStation";  //thing for grouping all the data together
const char* host = "dweet.io";      //host :) for tcp connection

//THIS WAY WE REFER TO EACH VARIABLE AS AN INDEX IN THIS GLOBAL ARRAY.

String arrayVariableNames[] = { "Temperature", "HR", "Pressure", "WSpeed", "WDir", "Light" };

float arrayVariableValues[5] = { { 0 } };
String arrayVariableValues2[1] = { { "" } };


//tells the number of arguments inside each array
int numberVariables = 1 + sizeof(arrayVariableValues) / sizeof(arrayVariableValues[0]);


/* Rtc object*/
static PCF8563 rtc;
/* RTC init data  */
RTC_Data_Type RTC_initData = {
  /*seconds*/ /*minutes*/ /*hours*/
  0, 21, 10,
  /*  day  */ /* month*/ /* year */
  12, 5, 2023
};

/* LCD object */
LiquidCrystal_PCF8574 lcd(0x27);

/* LCD object */
DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE
#define BASE_ALTITUDE 350
/* Ligh sensor pin */
#define LIGHTPIN 13
/* Wind dir */
#define WINDPIN A0

/* Temp and Humid DHT */
#define DHTPIN 12     /* Digital pin connected to the DHT sensor */
#define DHTTYPE DHT11 /* type of sensor in use: */
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t event;

/* speed sensor for wind */
#define GPIO_SPEEDPIN 10
#define PULSEPERTURN 10
uint32_t pulses = 0;
uint32_t timeold = 0;
uint32_t rpm = 0;
uint16_t windAnalog_val;

/* server data */
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;
ESP8266WebServer webServer(80);

/*******************************************************
* local function prototypes
********************************************************/
void setUpRTC(RTC_Data_Type config_data);
void setUpLCD(void);
void setUpTempRH(void);
void setUpWind(void);
void setUpLight(void);
void setUpPressure(void);
void update_array(void);
void setUpServer(void);
void printDigits(byte digits);
char* getWind_Dir(void);
uint8_t getLight(void);
float getT(void);
void getWatch(void);
/*******************************************************
* main area
********************************************************/

void hw_wdt_disable() {
  *((volatile uint32_t*)0x60000900) &= ~(1);  // Hardware WDT OFF
}
void setup() {
  // put your setup code here, to run once:
  hw_wdt_disable();
  Serial.begin(9600);

  setUpRTC(RTC_initData);
  setUpLCD();
  setUpPressure();
  setUpLight();
  setUpTempRH();
  setUpWind();
  setUpSpeed();
  setUpServer();
}

void loop() {

  // put your main code here, to run repeatedly:
  Data_Hdl.Temp = getT();
  Data_Hdl.Humid = getRH();
  Data_Hdl.Press = 0.0075 * sensor.readPressPa();
  Data_Hdl.Wind_speed = getSpeed();
  strcpy(Data_Hdl.Wind_dir, getWind_Dir());
  Data_Hdl.Light = getLight();
  getWatch();
  dnsServer.processNextRequest();
  webServer.handleClient();
  showLCD();
  update_array();
  sendDweet();  //send data to dweet.io
  delay(2000);  //refresh rate (send a dweet every 2 seconds...)
}

/*******************************************************
* local function definition
********************************************************/

void setUpWind(void) {
  pinMode(WINDPIN, INPUT);
}

void setUpLight(void) {
  pinMode(LIGHTPIN, INPUT);
  Data_Hdl.Light = digitalRead(LIGHTPIN);
}

void setUpPressure(void) {
  int rslt;
  while (ERR_OK != (rslt = sensor.begin())) {
    if (ERR_DATA_BUS == rslt) {
      Serial.println("Data bus error!!!");
    } else if (ERR_IC_VERSION == rslt) {
      Serial.println("Chip versions do not match!!!");
    }
    delay(3000);
  }
  Serial.println("Begin ok!");

  while (!sensor.setSamplingMode(sensor.eUltraPrecision)) {
    Serial.println("Set samping mode fail, retrying....");
    delay(3000);
  }

  delay(100);
#ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  /**
   * Calibrate the sensor according to the current altitude
   * Please change to the local altitude when using it.
   */
  if (sensor.calibratedAbsoluteDifference(BASE_ALTITUDE)) {
    Serial.println("Absolute difference base value set successfully!");
  }
#endif

  /* Get the sampling period of the current measurement mode, unit: us */
  float sampingPeriodus = sensor.getSamplingPeriodUS();
  Serial.print("samping period : ");
  Serial.print(sampingPeriodus);
  Serial.println(" us");

  /* Get the sampling frequency of the current measurement mode, unit: Hz */
  float sampingFrequencyHz = 1000000 / sampingPeriodus;
  Serial.print("samping frequency : ");
  Serial.print(sampingFrequencyHz);
  Serial.println(" Hz");

  Serial.println();
  delay(1000);
}

void setUpRTC(RTC_Data_Type config_data) {
  /* initialize RTC */
  rtc.begin();
  /* set the initial time an date */
  rtc.setTime(config_data.hours, config_data.minutes, config_data.seconds);
  rtc.setDate(config_data.day, config_data.month, config_data.year);
  /* start the clock */
  rtc.startClock();
}

void setUpLCD(void) {
  int error;
  Serial.println("Dose: check for LCD");
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    lcd.begin(20, 4);  // initialize the lcd

  } else {
    Serial.println(": LCD not found.");
  }

  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(0, 1);
  lcd.print("mmHg:");
  lcd.setCursor(12, 1);
  lcd.print("HR:");
  lcd.setCursor(0, 2);
  lcd.print("Wspeed:");
  lcd.setCursor(0, 3);
  lcd.print("Light:");
  lcd.setCursor(12, 3);
  lcd.print("WDir:");
}

void setUpTempRH(void) {
  sensor_t sensor;
  // Initialize device.
  dht.begin();
  // Print temperature sensor details.
  dht.temperature().getSensor(&sensor);
  /*
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  */

  dht.humidity().getSensor(&sensor);

  /*
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  */
}

void IRAM_ATTR isr() {
  /* increment pulses */
  pulses++;
}

void setUpSpeed(void) {
  pinMode(GPIO_SPEEDPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GPIO_SPEEDPIN), isr, RISING);
}

float getSpeed(void) {
  float mpers = 0;
  if (millis() - timeold >= 1000) { /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
    //Don't process interrupts during calculations
    detachInterrupt(GPIO_SPEEDPIN);
    //Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt
    //happened once per revolution
    rpm = (60UL * 1000 / PULSEPERTURN) / (millis() - timeold) * pulses;
    timeold = millis();
    pulses = 0;
    //Restart the interrupt processing
    attachInterrupt(digitalPinToInterrupt(GPIO_SPEEDPIN), isr, RISING);
  }
  mpers = (0.003 * rpm) / 0.277;
  return mpers;
}

float getT(void) {
  float retVal = 0.0;
  /*read temp from sensors */
  dht.temperature().getEvent(&event);
  float temperature = sensor.readTempC();
  /* make average value and return it*/
  if (event.temperature > 0) {
    retVal = (event.temperature + temperature) / 2.0;
  } else {
    retVal = temperature;
  }

  return retVal;
}

float getRH(void) {
  float retVal = 0.0;
  /*read temp from sensors */
  dht.humidity().getEvent(&event);
  retVal = event.relative_humidity;
  return retVal;
}

void showLCD(void) {
  lcd.setCursor(5, 0);  // move cursor to (2, 0)
  lcd.print(Data_Hdl.Temp);
  lcd.setCursor(12, 0);  // move cursor to (2, 0)
  if (Data_Hdl.Watch.hours < 10) {
    lcd.print('0');
  }
  lcd.print(Data_Hdl.Watch.hours, DEC);
  printDigits(Data_Hdl.Watch.minutes);
  printDigits(Data_Hdl.Watch.seconds);
  lcd.setCursor(5, 1);
  lcd.print(Data_Hdl.Press);
  lcd.setCursor(15, 1);
  lcd.print(Data_Hdl.Humid);
  lcd.setCursor(11, 2);
  lcd.print("    ");
  lcd.setCursor(7, 2);
  lcd.print(Data_Hdl.Wind_speed);
  lcd.print("m/s");
  lcd.setCursor(6, 3);
  lcd.print(Data_Hdl.Light ? "Nigth" : "Day  ");
  lcd.setCursor(17, 3);
  lcd.print(Data_Hdl.Wind_dir);
  lcd.setCursor(19, 3);
  lcd.write(' ');
}

byte getLight(void) {
  byte retVal;
  retVal = digitalRead(LIGHTPIN);
  return retVal;
}

char* getWind_Dir(void) {
  static char retVal[2];
  windAnalog_val = analogRead(WINDPIN);
  if ((windAnalog_val > 1024) || (windAnalog_val < 25)) {
    retVal[0] = ' ';
    retVal[1] = 'N';

  } else if ((windAnalog_val > 83) && (windAnalog_val < 153)) {
    retVal[0] = 'N';
    retVal[1] = 'V';
  } else if ((windAnalog_val > 211) && (windAnalog_val < 281)) {
    retVal[0] = ' ';
    retVal[1] = 'V';
  } else if ((windAnalog_val > 339) && (windAnalog_val < 409)) {
    retVal[0] = 'S';
    retVal[1] = 'V';
  } else if ((windAnalog_val > 467) && (windAnalog_val < 537)) {
    retVal[0] = ' ';
    retVal[1] = 'S';
  } else if ((windAnalog_val > 595) && (windAnalog_val < 665)) {
    retVal[0] = 'S';
    retVal[1] = 'E';
  } else if ((windAnalog_val > 723) && (windAnalog_val < 850)) {
    retVal[0] = ' ';
    retVal[1] = 'E';
  } else if ((windAnalog_val > 851) && (windAnalog_val < 1021)) {
    retVal[0] = 'N';
    retVal[1] = 'E';
  }

  return retVal;
}

void getWatch(void) {
  Data_Hdl.Watch.day = rtc.getDay();
  Data_Hdl.Watch.month = rtc.getMonth();
  Data_Hdl.Watch.year = rtc.getYear();

  Data_Hdl.Watch.hours = rtc.getHours();
  Data_Hdl.Watch.minutes = rtc.getMinutes();
  Data_Hdl.Watch.seconds = rtc.getSeconds();
}
void printDigits(byte digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if (digits < 10)
    lcd.print('0');
  lcd.print(digits, DEC);
}

void setUpServer(void) {
  //Connect to WiFi Network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println("...");

  WiFi.begin(ssid, password);
  int retries = 0;
  while ((WiFi.status() != WL_CONNECTED) && (retries < 15)) {
    retries++;
    delay(500);
    Serial.print(".");
  }
  if (retries > 14) {
    Serial.println(F("WiFi conenction FAILED"));
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("WiFi connected!"));
    Serial.println(F("https://freeboard.io/board/tfrwem"));
  }
}

String getDweetString() {
  int i = 0;
  //use the dweet GET to post to dweet
  String dweetHttpGet = "GET /dweet/for/";
  dweetHttpGet = dweetHttpGet + String(thingName) + "?";


  for (i = 0; i < (numberVariables); i++) {
    if (i != numberVariables - 1) {
      dweetHttpGet = dweetHttpGet + String(arrayVariableNames[i]) + "=" + String(arrayVariableValues[i]) + "&";
    } else
      //the lastone doesnt have a "&" at the end
      dweetHttpGet = dweetHttpGet + String(arrayVariableNames[i]) + "=" + arrayVariableValues2[i - 5];
  }

  dweetHttpGet = dweetHttpGet + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n";
  return dweetHttpGet;  //this is our freshly made http string request
}

void sendDweet() {
  WiFiClient client;
  const int httpPort = 80;

  //connect to dweet.io
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  client.print(getDweetString());
  delay(10);  //wait...
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
}

void update_array() {
  arrayVariableValues[0] = Data_Hdl.Temp;
  arrayVariableValues[1] = Data_Hdl.Humid;
  arrayVariableValues[2] = Data_Hdl.Press;
  arrayVariableValues[3] = Data_Hdl.Wind_speed;
  arrayVariableValues[4] = (float)windAnalog_val;
  arrayVariableValues2[0] = Data_Hdl.Light ? "Nigth" : "Day";
}