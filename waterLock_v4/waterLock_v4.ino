#include "LowPower.h"
#include <EEPROM.h>
#include <Servo.h>
#include "RTClib.h"
#include <Vcc.h>

// Configurazioni hardware
#define WAKE_UP_PIN 2
#define SERVO_PIN 5
#define LED_PIN 4
#define ANALOG_INPUT 0
#define EEPROM_SIZE 64

// Limiti batteria
#define MIN_VOLTAGE 0.0
#define MAX_VOLTAGE 4.20
const float FIX_VOLTAGE = 4.13 / 4.20;

// Posizioni servo
#define SERVO_MIN_POS 30
#define SERVO_MAX_POS 130

// Variabili globali
Servo myservo;
RTC_DS3231 rtc;
Vcc vcc(FIX_VOLTAGE);

long litri_giornalieri, litri_giornalieri_max, k;
byte stato_valvola, pos;
int rtc_value[5];
byte giorno, mese;
long anno;

// Funzioni ausiliarie per la gestione della EEPROM
void writeLongToEEPROM(int address, long number) {
  for (int i = 0; i < 4; ++i) {
    EEPROM.write(address + i, (number >> (24 - i * 8)) & 0xFF);
  }
}

long readLongFromEEPROM(int address) {
  return ((long)EEPROM.read(address) << 24) |
         ((long)EEPROM.read(address + 1) << 16) |
         ((long)EEPROM.read(address + 2) << 8) |
         (long)EEPROM.read(address + 3);
}

// Funzione per leggere il livello della batteria
float readBatteryVoltage() {
  float vout = (analogRead(ANALOG_INPUT) * vcc.Read_Volts()) / 1024.0;
  float vin = vout / (10000.0 / (100400.0 + 10000.0));
  return vin < 0.09 ? 0.0 : vin;
}

// Funzione per aprire la valvola
void openValve() {
  Serial.println("Apertura valvola...");
  digitalWrite(LED_PIN, HIGH);
  myservo.attach(SERVO_PIN);

  for (int pos = EEPROM.read(16); pos <= SERVO_MAX_POS; ++pos) {
    myservo.write(pos);
    delay(20);
  }

  myservo.detach();
  digitalWrite(LED_PIN, LOW);
  EEPROM.write(16, SERVO_MAX_POS);
  EEPROM.write(15, 1); // Stato valvola: aperta
  litri_giornalieri = 0;
  writeLongToEEPROM(0, litri_giornalieri);

  Serial.println("Valvola aperta e contatore giornaliero azzerato.");
}

// Funzione per chiudere la valvola
void closeValve() {
  Serial.println("Chiusura valvola...");
  digitalWrite(LED_PIN, HIGH);
  myservo.attach(SERVO_PIN);

  for (int pos = EEPROM.read(16); pos >= SERVO_MIN_POS; --pos) {
    myservo.write(pos);
    delay(20);
  }

  myservo.detach();
  digitalWrite(LED_PIN, LOW);
  EEPROM.write(16, SERVO_MIN_POS);
  EEPROM.write(15, 0); // Stato valvola: chiusa

  // Salva la data della chiusura
  DateTime now = rtc.now();
  EEPROM.write(17, now.day());
  EEPROM.write(18, now.month());
  writeLongToEEPROM(20, now.year());

  Serial.println("Valvola chiusa.");
}

// Funzione per gestire il reset giornaliero
void resetDailyCounter() {
  DateTime now = rtc.now();

  if (now.day() > EEPROM.read(30) || now.month() > EEPROM.read(31) || now.year() > readLongFromEEPROM(32)) {
    Serial.println("Reset contatore giornaliero.");
    writeLongToEEPROM(0, 0);
    EEPROM.write(30, now.day());
    EEPROM.write(31, now.month());
    writeLongToEEPROM(32, now.year());
  }
}

// Funzione per la lettura e il debug della memoria
void debugEEPROM() {
  Serial.print("Volume erogato: ");
  Serial.println(readLongFromEEPROM(0));
  Serial.print("Volume massimo giornaliero: ");
  Serial.println(readLongFromEEPROM(5));
  Serial.print("Moltiplicatore K: ");
  Serial.println(readLongFromEEPROM(10));
  Serial.print("Stato valvola: ");
  Serial.println(EEPROM.read(15));
  Serial.print("Posizione valvola: ");
  Serial.println(EEPROM.read(16));
  Serial.print("Ultima chiusura: ");
  Serial.print(EEPROM.read(17));
  Serial.print('/');
  Serial.print(EEPROM.read(18));
  Serial.print('/');
  Serial.println(readLongFromEEPROM(20));
}

// Funzione per gestire l'attività principale della valvola
void manageValve() {
  if (litri_giornalieri >= litri_giornalieri_max) {
    detachInterrupt(digitalPinToInterrupt(WAKE_UP_PIN));
    closeValve();
  } else {
    resetDailyCounter();
    attachInterrupt(digitalPinToInterrupt(WAKE_UP_PIN), []() {
      litri_giornalieri += k;
    }, RISING);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

void setup() {
  pinMode(WAKE_UP_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ANALOG_INPUT, INPUT);
  Serial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("Errore inizializzazione RTC!");
    while (1);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  EEPROM.begin(EEPROM_SIZE);

  litri_giornalieri = readLongFromEEPROM(0);
  litri_giornalieri_max = readLongFromEEPROM(5);
  k = readLongFromEEPROM(10);
  stato_valvola = EEPROM.read(15);
  pos = EEPROM.read(16);

  Serial.println("Setup completato.");
  debugEEPROM();
}

void loop() {
  if (stato_valvola == 1) { // Valvola aperta
    manageValve();
  } else { // Valvola chiusa
    openValve();
  }
}
