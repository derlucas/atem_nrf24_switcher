#include <SPI.h>
#include <Bounce2.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define LED_YELLOW  5
#define LED_GREEN   4
#define BTN_RED     A0
#define BTN_BLUE    A1

#define ADDRESS     0x00    // change before flashing!
#define RX_TIMEOUT  2000    // in ms

RF24 radio(9,10);

Bounce debouncer1 = Bounce(); 
Bounce debouncer2 = Bounce(); 
byte blinkState = LOW;
long lastTimeDataReceived;
long blinkMillis;
const uint64_t listening_pipes[2] = { 0x4A4A4A4AF0LL, 0x4A4A4A4AE1LL };
uint8_t buttonFell;

void setup() {
  Serial.begin(115200);
  printf_begin();
  printf("booting client id: %d\n", ADDRESS);

  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  pinMode(BTN_RED, INPUT_PULLUP);
  pinMode(BTN_BLUE, INPUT_PULLUP);
  debouncer1.attach(BTN_RED);
  debouncer1.interval(20);
  debouncer2.attach(BTN_BLUE);
  debouncer2.interval(20);
  
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
 
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.setRetries(10,15);
  radio.setPayloadSize(1);  
  radio.openReadingPipe(1,listening_pipes[ADDRESS]);
  radio.startListening();
  radio.printDetails();
}

void loop() {
  
  debouncer1.update();
  debouncer2.update();

  if(debouncer1.fell()) {
    buttonFell = 1;
  }
  if(debouncer2.fell()) {
    buttonFell += 2;
  }

 
  if(radio.available()) {
    uint8_t payload;
    radio.read(&payload, 1);
    //printf("data: %d\n", payload);

    byte ackValue = buttonFell;
    //printf("ack: %d\n", ackValue);
    radio.writeAckPayload(1, &ackValue, 1);
    buttonFell = 0;

    if(bitRead(payload, 0)) {
      digitalWrite(LED_YELLOW, HIGH);
    } else {
      digitalWrite(LED_YELLOW, LOW);
    }
    
    digitalWrite(LED_GREEN, HIGH);
    
    lastTimeDataReceived = millis();;
    
  } else {    
    
    // we have no data, so check for timeout
    if(millis() - lastTimeDataReceived > RX_TIMEOUT) {
      
      buttonFell = 0;
      
      if(millis() - blinkMillis > 100) {
        if(blinkState == LOW) {
          blinkState = HIGH;
        } else {
          blinkState = LOW;
        }
        
        digitalWrite(LED_GREEN, blinkState);
        blinkMillis = millis();
      }    
    }
  }
}


