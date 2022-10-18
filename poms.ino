#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "logos.h"

// REPLACE with your Domain name and URL path or IP address with path
const String serverName = "http://10.10.10.200:8000/";

// Keep this API Key value to be compatible with the PHP code provided in the project page.
String apiKeyValue = "tPmAT5Ab3j7F9";

MAX30105 particleSensor;
#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
int spo2Limit; // sets limit for spo2 level to beep
int addressSpo2Limit = 0;

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// define the number of bytes you want to access
#define EEPROM_SIZE 1

#define BUZZER 16
#define BTN_UP 25
#define BTN_DOWN 26
#define BTN_START 32
#define BTN_MENU 33
#define debounceTimeout 50
int startButtonPreviousState = LOW; // HI means NOT PRESSED
int menuButtonPreviousState = HIGH; // LO means PRESSED
long int lastDebounceTime;

bool isBeep = true;
bool isStart = false;
bool initialReading = true;
int optionSelected = 0;
String menuOption[] = {"WELCOME", "SET SPO2 LIMIT", "Machine Number", "WIFI"};
char machineNumber[25];

#define TRIGGER_PIN 0

// wifimanager can run in a blocking mode or a non blocking mode
// Be sure to know how to process loops with no delay() if using non blocking
bool wm_nonblocking = false; // change to true to use non blocking


WiFiManager wm; // global wm instance
WiFiManagerParameter custom_field; // global param ( for non blocking w params )

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  // Set Machine Number
  snprintf(machineNumber, 25, "DEVICE-%llX", WiFi.macAddress());
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_UP, INPUT);
  pinMode(BTN_DOWN, INPUT);
  pinMode(BTN_START, INPUT);
  pinMode(BTN_MENU, INPUT);

  // Get the SPO2Limit Value
  spo2Limit = EEPROM.read(addressSpo2Limit);
  if (spo2Limit <= 0 || spo2Limit > 100) {
    // set a default spo2limit
    spo2Limit = 90;
    EEPROM.write(addressSpo2Limit, spo2Limit);
    EEPROM.commit();
  }

  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1);
  }

  oled.display();
  delay(2000); // wait two seconds for initializing
  oled.clearDisplay();

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    oled.clearDisplay();
    oled.setTextSize(1);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 0);
    oled.println(F("MAX30105 was not found. Please check wiring/power."));
    oled.display();
    delay(1500);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  //  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //for WIFIMANAGER
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  if (wm_nonblocking) wm.setConfigPortalBlocking(false);

  // add a custom input field
  int customFieldLength = 40;
  const char* custom_radio_str = "<br/><label for='customfieldid'>Custom Field Label</label><input type='radio' name='customfieldid' value='1' checked> One<br><input type='radio' name='customfieldid' value='2'> Two<br><input type='radio' name='customfieldid' value='3'> Three";
  new (&custom_field) WiFiManagerParameter(custom_radio_str); // custom html input

  wm.addParameter(&custom_field);
  wm.setSaveParamsCallback(saveParamCallback);

  std::vector<const char *> menu = {"wifi", "info", "param", "sep", "restart", "exit"};
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");

  wm.setConfigPortalTimeout(30); // auto close configportal after n seconds

  bool res;
  // ITO YUNG WIFI HOTSPOT PARA PALITAN ANG WIFI CREDENTIALS
  res = wm.autoConnect((const char*)machineNumber, "password"); // password protected ap

  if (!res) {
    oledPrint(0, 0, "Failed to connect or hit timeout");
    Serial.println("Failed to connect or hit timeout");
    // ESP.restart();
  }
  else {
    //if you get here you have connected to the WiFi
    oledPrint(0, 0, "Wifi connected...");
    Serial.println("Wifi connected...");
  }
}
void checkButton() {
  // check for button press
  if (digitalRead(BTN_UP) == HIGH) {
    // poor mans debounce/press-hold, code not ideal for production
    delay(50);
    if (digitalRead(BTN_UP) == HIGH) {
      Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings, code not ideaa for production
      delay(3000); // reset delay hold
      if ( digitalRead(BTN_UP) == HIGH) {
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        wm.resetSettings();
        ESP.restart();
      }

      // start portal w delay
      Serial.println("Starting config portal");
      wm.setConfigPortalTimeout(120);

      if (!wm.startConfigPortal((const char*)machineNumber, "password")) {
        Serial.println("failed to connect or hit timeout");
        delay(3000);
        // ESP.restart();
      } else {
        //if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
      }
    }
  }
}

String getParam(String name) {
  //read parameter from server, for customhmtl input
  String value;
  if (wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback() {
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}

void readPulse() {
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  if (initialReading) {
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      oledPrint(0, 0, "INITIAL READING...\nPlease Wait");
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  // Display HR and SPO2 in OLED
  String message = "HR=" + String(heartRate) + "\n" + "SPO2=" + String(spo2) + "%";
  oledPrint(0, 0, message);

  if (spo2 < spo2Limit) {
    if (isBeep) {
      noTone(BUZZER);
      tone(BUZZER, 1900);
    } else {
      noTone(BUZZER);
    }
    isBeep = !isBeep;
  }
  
  initialReading = false;

  // Send data to server
  if (validSPO2 == 1 && validHeartRate == 1) {
    sendData(String(heartRate), String(spo2));
  }
}

void sendData(String hr, String spo2) {
  // SESEND NG DATA FROM DEVICE TO WEB SERVER
  //PAG NAKA KONEK, MAGSESEND
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName + "api/pulse-data");

    // Specify content-type header
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Prepare your HTTP POST request data
    String httpRequestData = "api_key=" + apiKeyValue + "&id=" + machineNumber + "&hr=" + hr + "&spo2=" + spo2 + "&spo2_limit=" + spo2Limit;

    // Send HTTP POST request
    int httpResponseCode = http.POST(httpRequestData);
    Serial.println(httpResponseCode);
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  //Send an HTTP POST request every 30 seconds
}

void oledPrint(int x, int y, String message)
{
  oled.clearDisplay();
  oled.setTextSize(1);         // set text size
  oled.setTextColor(WHITE);    // set text color
  oled.setCursor(x, y);
  oled.println(message);
  oled.display();
  delay(1);
}

void loop()
{
  // Read the button
  int startButtonPressed = digitalRead(BTN_START);
  int menuButtonPressed = digitalRead(BTN_MENU);
  int upButtonPressed = digitalRead(BTN_UP);
  int downButtonPressed = digitalRead(BTN_DOWN);

  // DISPLAY NG MENU
  if (!isStart) {
    if (menuButtonPreviousState == HIGH) {
      //menu is selected
      switch (optionSelected) {
        case 1:
          {
            // SET SPO2 LIMIT
            String msg = menuOption[optionSelected] + "\n\tSPO2 Level:" + spo2Limit + "%";
            oledPrint(0, 0, msg);
            break;
          }
        case 2:
          {
            // DISPLAY NG MACHINE NUMBER
            String msg = menuOption[optionSelected] + "\n\t" + String(machineNumber);
            oledPrint(0, 0, msg);
            break;
          }
        case 3:
          {
            // IIIRESET NG WIFI
            String msg = menuOption[optionSelected];
            oledPrint(0, 0, msg);
            oled.setCursor(0, 10);
            oled.print("PRESS UP BTN TO RESTART WIFI");
            oled.display();
            if (wm_nonblocking) wm.process(); // avoid delays() in loop when non-blocking and other long running code
            checkButton();
            break;
          }
        case 0:
        default:
          {
            // welcome
            oledPrint(0, 0, menuOption[optionSelected]);
            break;
          }
      }
    }
  }

  // Get the current time
  long int currentTime = millis();
  // check if button is not press
  if (startButtonPressed == LOW && menuButtonPressed == LOW && upButtonPressed == LOW && downButtonPressed == LOW) {
    lastDebounceTime = currentTime;
    startButtonPreviousState = LOW;
  }

  // CHECK KUNG NA CLICK NA IYUNG BUTTON
  if ((currentTime - lastDebounceTime) > debounceTimeout) {
    // Button is pressed
    if (startButtonPressed == HIGH) {
      // START/STOP Button is pressed
      menuButtonPreviousState = LOW;
      if (!isStart) {
        initialReading = true;
        isStart = true;
      } else {
        isStart = false;
        noTone(BUZZER);
        delay(1000);
      }
    } else if (menuButtonPressed == HIGH) {
      noTone(BUZZER);
      menuButtonPreviousState = LOW;
      optionSelected = (optionSelected < ARRAY_SIZE(menuOption) - 1) ? optionSelected + 1 : 0;
      delay(500);
    }
    else if (menuButtonPreviousState == HIGH && optionSelected == 1) {
      bool update = false;
      if (upButtonPressed == HIGH) {
        if (spo2Limit < 100)
          spo2Limit++;
        update = true;
      } else if (downButtonPressed == HIGH) {
        if (spo2Limit > 90)
          spo2Limit--;
        update = true;
      }

      if (update) {
        EEPROM.write(addressSpo2Limit, spo2Limit);
        EEPROM.commit();
        delay(500);
      }
    }
  }

  if (isStart) {
    readPulse();
  }
  //DISPLAY NG WIFI LOGO
  if (WiFi.status() == WL_CONNECTED) {
    oled.drawBitmap(100, 0, wifiLogo, 16, 16, WHITE);
  } else {
    oled.drawBitmap(100, 0, noWifiLogo, 16, 16, WHITE);
  }
  oled.display();
}
