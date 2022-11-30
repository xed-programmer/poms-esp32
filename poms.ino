#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
const byte NOTIF_BORDER = 0;

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
const byte spo2Limit = 90;

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define BUZZER 10
#define BTN_MENU 9
#define BTN_START 8
#define debounceTimeout 50

byte startButtonPressed, menuButtonPressed;
byte startButtonPreviousState = HIGH; // HI means NOT PRESSED
byte menuButtonPreviousState = LOW; // LO means PRESSED
long int lastDebounceTime;

bool isBeep = true;
bool isStart = false;
bool initialReading = true;
byte optionSelected = 0;
String menuOption[] = {"WELCOME", "Machine Number"};
char machineNumber[25];

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
//  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_MENU, INPUT_PULLUP);

  // initialize OLED display with I2C address 0x3C
  //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1);
  }

  oled.display();
  delay(2000); // wait two seconds for initializing
  oled.clearDisplay();

  // Initialize sensor
//  if (!particleSensor.begin(Wire, (uint32_t)400000)) //Use default I2C port, 400kHz speed
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    oled.clearDisplay();
    oled.setTextSize(1);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 0);
    oled.println(F("MAX30105 was not found. Please check wiring/power."));
    oled.display();
    delay(5000);
  }

    byte ledBrightness = 100; //Options: 0=Off to 255=50mA
    byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
//  particleSensor.setup(); //Configure sensor with default settings
//  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
//  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop()
{
  // Read the button
  startButtonPressed = digitalRead(BTN_START);
  menuButtonPressed = digitalRead(BTN_MENU);

  //  // DISPLAY NG MENU
  menu();
  //  // CHECK IF BUTTON CLICKED
  checkButtonClicked();

  if (isStart) {
    readPulse();
    displayPulseReading();
  }
}

void menu() {
  if (!isStart) {
    // REFACTOR MO YUNG PAGGDISPLAY NG SPO2, FROM PULSEREAD METHOD TO DITO SA CODE BLCOK NATO
    if (menuButtonPreviousState == LOW) {
      //menu is selected
      switch (optionSelected) {
        case 1:
          {
            // DISPLAY NG MACHINE NUMBER
            String msg = menuOption[optionSelected] + "\n\t" + String(machineNumber);
            oledPrint(0, 0, msg);
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
}

void checkButtonClicked() {
  // Get the current time
  long int currentTime = millis();
  // check if button is not press
  if (startButtonPressed == HIGH && menuButtonPressed == HIGH) {
    lastDebounceTime = currentTime;
    startButtonPreviousState = HIGH;
  }

  // CHECK KUNG NA CLICK NA IYUNG BUTTON
  if ((currentTime - lastDebounceTime) > debounceTimeout) {
    // Button is pressed
    if (startButtonPressed == LOW) {
      // START/STOP Button is pressed
      menuButtonPreviousState = HIGH;
      if (!isStart) {
        initialReading = true;
        isStart = true;
      } else {
        isStart = false;
        noTone(BUZZER);
        delay(1500);
      }
    } else if (menuButtonPressed == LOW) {
      noTone(BUZZER);
      menuButtonPreviousState = LOW;
      initialReading = true;
      isStart = true;
      optionSelected = (optionSelected < ARRAY_SIZE(menuOption) - 1) ? optionSelected + 1 : 0;
      delay(500);
    }
  }
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
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }
    initialReading = false;
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

    Serial.print(F("HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  // Display HR and SPO2 in OLED

  if (spo2 < spo2Limit) {
    if (isBeep) {
      noTone(BUZZER);
      tone(BUZZER, 1900);
    } else {
      noTone(BUZZER);
    }
    isBeep = !isBeep;
  } else {
    noTone(BUZZER);
  }

  // Send data to server
  if (validSPO2 == 1 && validHeartRate == 1) {
//    sendData(String(heartRate), String(spo2));
  }
}
void displayPulseReading() {
  // DISPLAY HR AND SPO2 READINGS

  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(5, NOTIF_BORDER);
  oled.println("HR");

  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(5, NOTIF_BORDER + (6 * 3));
  oled.println("SPO2");

  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(SCREEN_WIDTH / 2, NOTIF_BORDER);
  oled.println(heartRate);

  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(SCREEN_WIDTH / 2, NOTIF_BORDER + (6 * 3));
  oled.println(String(spo2) + "%");

  oled.display();
  delay(1);
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
