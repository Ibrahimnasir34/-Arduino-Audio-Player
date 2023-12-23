#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <MedianFilter.h>

MedianFilter ultrasonic_distance(7, 0);

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
int files = 0;

volatile unsigned long LastPulseTimeA;
int durationA;
int distance;
#define trigPinA 7
#define echoPinA 2
#define microphone 3
#define pinDFBusy 12
int clap = 0;
long detection_range_start = 0;
long detection_range = 0;
int status_microphone = 1; // normally HIGH, LOW when there is a signal / clap
boolean clap_status = false;
boolean status_lights = false;
unsigned long sonar_time = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
long randNumber;
int result;
int ultrasonic_flag = 0;

unsigned long previousMillis1 = 0;  // will store last time LED was updated
unsigned long previousMillis2 = 0;  // will store last time LED was updated
int interval1 = 0;  // interval at which to blink (milliseconds)
int interval2 = 0; 
int leds[4] = {6, 8, 9, 13};

int trig=2;
int echo=7;

long timeInMicro;
long distanceInCm;

void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(7, INPUT);
  for (int jj = 0; jj < sizeof(leds) / sizeof(int); jj++) {
    pinMode(leds[jj], OUTPUT);
    delay(10);
  }

  pinMode(microphone, INPUT);
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);
  pinMode(13, OUTPUT);
  pinMode(pinDFBusy, INPUT);
  randomSeed(analogRead(0));
  Serial.begin(115200);
  currentMillis = millis();
  previousMillis = currentMillis;
  attachInterrupt(digitalPinToInterrupt(echoPinA), EchoPinA_ISR, CHANGE);

  mySoftwareSerial.begin(9600);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(500);
  myDFPlayer.volume(30);
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.EQ(DFPLAYER_EQ_POP);
  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);

  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);

  Serial.println(myDFPlayer.readState());
  Serial.println(myDFPlayer.readVolume());
  Serial.println(myDFPlayer.readEQ());
  Serial.println(myDFPlayer.readFileCounts());
}

void loop() {
   // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  timeInMicro = pulseIn(echo, HIGH);
  distanceInCm = ((timeInMicro / 29) / 2);

  Serial.println(distanceInCm);

  for (int i = 0; i < sizeof(leds) / sizeof(int); i++) {
    digitalWrite(leds[i], HIGH);
    delayMicroseconds(300);
    digitalWrite(leds[i], LOW);
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    // Add the following code for LED blinking at random intervals
    // For LED at Pin 4
    if (currentMillis - previousMillis1 >= interval1) {
      previousMillis1 = currentMillis;
      interval1 = random(1000, 5000);
      digitalWrite(leds[0], !digitalRead(leds[0]));
    }

    // For LED at Pin 6
    if (currentMillis - previousMillis2 >= interval2) {
      previousMillis2 = currentMillis;
      interval2 = random(1000, 5000);
      digitalWrite(leds[1], !digitalRead(leds[1]));
    }
  }

  microphone_check();
  currentMillis = millis();

  if (currentMillis - previousMillis >= 50) {
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);
    previousMillis = currentMillis;
    Serial.print("Ping: ");
    Serial.print(LastPulseTimeA);
    Serial.print('\t');
    distance = LastPulseTimeA / 2 / 29.1;
    ultrasonic_distance.in(distance);
    result = ultrasonic_distance.out();
    Serial.println(distance);
    Serial.print(result);
    Serial.println("cm");

    if (ultrasonic_flag == 0) {
      if (result > 1 && result < 12) {
        randNumber = random(1, 5);
        Serial.println(randNumber);
        myDFPlayer.playLargeFolder(1, randNumber);
        Serial.println(myDFPlayer.readCurrentFileNumber());
        do {
          // do nothing while busy 
        } while (digitalRead(pinDFBusy) == LOW);
        ultrasonic_flag = 1;
      }
      if (result > 200 && result < 250) {
        myDFPlayer.playLargeFolder(2, 1);
        Serial.println(myDFPlayer.readCurrentFileNumber());
        do {
          // do nothing while busy
        } while (digitalRead(pinDFBusy) == LOW);
        ultrasonic_flag = 1;
      }
    }
    if (ultrasonic_flag > 0) {
      ultrasonic_flag++;
      Serial.println("ultrasonic_flag++");
    }

    if (ultrasonic_flag == 15) {
      ultrasonic_flag = 0;
      Serial.println("ultrasonic_flag = 0");
    }
  }

  if (clap_status == true) {
    clap_status = false;
    randNumber = random(1, 5);
    Serial.println(randNumber);
    myDFPlayer.playLargeFolder(1, randNumber);
    Serial.println(myDFPlayer.readCurrentFileNumber());
    do {
      // Serial.println("low");
    } while (digitalRead(pinDFBusy) == LOW);
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read());
  }
}

void EchoPinA_ISR() {
  static unsigned long startTimeA;

  if (digitalRead(2))
    startTimeA = micros();
  else
    LastPulseTimeA = micros() - startTimeA;
}

void microphone_check() {
  status_microphone = digitalRead(microphone);
  if (status_microphone == 0) {
    if (clap == 0) {
      Serial.println("clap = 0");
      detection_range_start = detection_range = millis();
      clap++;
    } else if (clap > 0 && millis() - detection_range >= 100) {
      Serial.println(clap);
      Serial.println("clap > 0 && millis >50");
      detection_range = millis();
      clap++;
    }
  }
  if (millis() - detection_range_start >= 350) {
    if (clap == 2) {
      clap_status = true;
      Serial.println("clap = success");
      if (!status_lights) {
        status_lights = true;
        digitalWrite(13, HIGH);
      } else if (status_lights) {
        status_lights = false;
        digitalWrite(13, LOW);
      }
    }
    clap = 0;
  }
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
