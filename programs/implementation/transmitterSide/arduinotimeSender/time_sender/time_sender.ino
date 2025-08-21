// size_t
#include <stddef.h>

// Arduino
#include <Arduino.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>
#include <SPI.h>

#define LORA_INIT_FLAG 0x7A
#define LORA_SEND_FLAG 0x7B
#define LORA_RESET_FLAG 0x7C
#define LORA_RESPONSE_FLAG 0x7D

#define EOL_MARKER '\x00'


int counter = 0;
bool initReceived = false;
bool initFinished = false;
byte deviceId = 0;
char inputBuffer[32];
byte inputCounter =0;
float frequency = 0;
int cycles = 0;
long last_pmic_wd_reset = millis();
const long PMIC_WD_RESET_INTERVAL = 1000;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  if (!PMIC.begin())
    while(1);
  if (!PMIC.disableCharge()) {
    Serial.println("PMIC: error disabling charge");
      while(1);
  }
  Serial.println("PMIC: charge disabled");
}

void print(String text,byte incomingByte){
  Serial.print(deviceId);
  Serial.print(text);
  Serial.println(incomingByte);
}
void print(String text, int incomingByte){
  Serial.print(deviceId);
  Serial.print(text);
  Serial.println(incomingByte);
}

void readInitString(){

  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();
      if (incomingByte == '\n') {
      inputBuffer[inputCounter] = '\0';
      initFinished = true;
    }else if (incomingByte == LORA_RESET_FLAG){
      reset();
    } else {
      inputBuffer[inputCounter] = incomingByte;
      inputCounter++;
    }
  }
  if (initFinished){
    char * strtokIndx; 

    strtokIndx = strtok(inputBuffer, ",");
    frequency = atof(strtokIndx);
    Serial.print("Frequency set to :");
    Serial.println(frequency * 1E6);
    LoRa.begin(frequency * 1E6);

    strtokIndx = strtok(NULL, ",");
    cycles = atoi(strtokIndx);
    Serial.print("Cycles set to :");
    Serial.println(cycles);


    strtokIndx = strtok(NULL, ",");
    int bw = atoi(strtokIndx);
    LoRa.setSignalBandwidth(bw * 1E3);
    Serial.print("Signal bandwidth set to :");
    Serial.println(bw* 1E3);


    strtokIndx = strtok(NULL, ",");
    int sf = atoi(strtokIndx);
    LoRa.setSpreadingFactor(sf);
    Serial.print("Spreading factor set to :");
    Serial.println(sf);


    strtokIndx = strtok(NULL, ",");
    deviceId = atoi(strtokIndx);
    Serial.print("Device id to :");
    Serial.println(deviceId);

    // change the transmission power to 14dB
    LoRa.setTxPower(14);

    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (HIGH is the voltage level)
    Serial.write(LORA_RESPONSE_FLAG);
    
  }
}


void waitForInitFlag() {
  if (Serial.available() == 0) {
    // blinks every seconds to show waiting for initFlag
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
    return;
  }

  byte incomingByte = Serial.read();
  if (incomingByte == EOL_MARKER) {
    return;
  }
  
  if (incomingByte == LORA_INIT_FLAG) {
    Serial.println("Init flag received. setting the parametters");
    initReceived = true;
  } else if (incomingByte == LORA_RESET_FLAG) {
    reset();
  }
}

void reset(){
    initReceived = false;
    counter = 0;
    initFinished =false;
    inputCounter =0;
    print(" Reset Flag received :", LORA_RESET_FLAG);
    // Send end flag to serial monitor
    Serial.write(LORA_RESPONSE_FLAG);
    //loop();
}

void sendLoRaPacket() {
  print(" sent packet", counter);

  LoRa.beginPacket(true);
  LoRa.write(deviceId);
  LoRa.endPacket();
  

  // Check for RESET flag
  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();
    if (incomingByte == LORA_RESET_FLAG) {
      reset();
    }
  }
}

void waitForSendFlag(){
  if (Serial.available() == 0) {
    return;
  }
  byte incomingByte = Serial.read();  
  if (incomingByte == EOL_MARKER) {
    return;
  }
  if (incomingByte == LORA_SEND_FLAG) {

    Serial.println("Send flag received");
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    sendLoRaPacket();
    counter++;
    Serial.write(LORA_RESPONSE_FLAG);
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (HIGH is the voltage level)

  } else if (incomingByte == LORA_RESET_FLAG) {
    reset();
  }
}

void loop(){
  if (!initReceived){
    waitForInitFlag();
  }else if (!initFinished){
    readInitString();
  }else {
    waitForSendFlag();
  }
  if (counter >= cycles && initFinished) {
    reset();
  }
  

  long now = millis();
  if (now > last_pmic_wd_reset) {
    last_pmic_wd_reset = now + PMIC_WD_RESET_INTERVAL;
    PMIC.resetWatchdog();
  }
}