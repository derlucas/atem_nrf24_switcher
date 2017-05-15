#define USE_ATEM
#define USE_NRF
#define USE_DMX

#include "printf.h"
#include <avr/wdt.h>
#include <Adafruit_NeoPixel.h>
#include "OneButton.h"
#include <SPI.h>

#ifdef USE_NRF
  #include "nRF24L01.h"
  #include "RF24.h"
#endif
#ifdef USE_ATEM
  #include <Ethernet.h>
  #include <ATEMbase.h>
  #include <ATEMext.h>
#endif

#define MAX_NODES      2
#define ATEM_CHANNEL   6
#define PIXELS        30

// IO config
#define ETH_RESET   A5

#define LED_GREEN   2
#define LED_RED     3
#define LED_TALLY1  A0
#define LED_TALLY2  A1
#define LED_TALLY3  A2
#define LED_TALLY4  A3
#define LED_TALLY5  A4

const byte tally_pins[] = { A0, A1, A2, A3, A4 };

#define PWM_RING1   44
#define PWM_RING2   45
#define PWM_RING3   46

#define WSLED_BOX1  22
#define WSLED_BOX2  23
#define WSLED_BOX3  24

#define DIPSW_1     35
#define DIPSW_2     36
#define DIPSW_3     37
#define DIPSW_4     38
#define DIPSW_5     39
#define DIPSW_6     40
#define DIPSW_7     41
#define DIPSW_8     42

#define NRF_CE      8 // 49
#define NRF_CSN     9 // 48

uint32_t ledMillis;
uint32_t nrfMillis;
uint32_t rxMillis[MAX_NODES];
byte blinkState = LOW;
byte tally[ATEM_CHANNEL];
byte nrfLeds[MAX_NODES];
byte nrfButtons[MAX_NODES];
byte ledBlinkState = LOW;
uint32_t ledBlinkMillis;
uint16_t buttons = 0;
uint8_t lastKeyerInput = -1;
bool enabledTestMode = false;

OneButton redButton(43, true);
OneButton btnA1(25, true); OneButton btnA2(26, true); OneButton btnA3(27, true);
OneButton btnB1(28, true); OneButton btnB2(29, true); OneButton btnB3(30, true);
OneButton btnScene1(31, true); OneButton btnScene2(32, true);
OneButton btnScene3(33, true); OneButton btnScene4(34, true);
OneButton btnScene5(47, true);

#ifdef USE_ATEM
  byte mac[] = { 0x90, 0xA2, 0xDB, 0x2A, 0x6A, 0xC4 };
  IPAddress clientIp(192, 168, 10, 45);
  IPAddress atemIp(192, 168, 10, 240);
  EthernetServer server(80);
  ATEMext atem;
  bool isAtemOnline = false;
#endif

#ifdef USE_NRF
  const uint64_t listening_pipes[] = { 0x4A4A4A4AF0LL, 0x4A4A4A4AE1LL };
  RF24 radio(NRF_CE, NRF_CSN);
#endif

#ifdef USE_DMX
  volatile uint32_t dmxMillis;
  volatile uint8_t  dmxRxField[3];      //array of DMX vals (raw)
  volatile uint16_t dmxAddress;         //start address
  enum {IDLE, BREAK, STARTB, STARTADR}; //DMX states
  volatile uint8_t dmxState;
#endif

Adafruit_NeoPixel pixels1 = Adafruit_NeoPixel(PIXELS, WSLED_BOX1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(PIXELS, WSLED_BOX2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels3 = Adafruit_NeoPixel(PIXELS, WSLED_BOX3, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels[] = { pixels1, pixels2, pixels3 };


void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println("\n\ncontroller box");

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  for(uint8_t i = 0; i < sizeof(tally_pins); i++) {
    pinMode(tally_pins[i], OUTPUT);
  }

  pinMode(PWM_RING1, OUTPUT); pinMode(PWM_RING2, OUTPUT); pinMode(PWM_RING3, OUTPUT);

  btnA1.attachClick(btnA1Click); btnA2.attachClick(btnA2Click); btnA3.attachClick(btnA3Click);
  btnB1.attachClick(btnB1Click); btnB2.attachClick(btnB2Click); btnB3.attachClick(btnB3Click);

  btnA1.setClickTicks(100); btnA2.setClickTicks(100); btnA3.setClickTicks(100);
  btnB1.setClickTicks(100); btnB2.setClickTicks(100); btnB3.setClickTicks(100);

  btnScene1.attachClick(btnScene1Click); btnScene2.attachClick(btnScene2Click);
  btnScene3.attachClick(btnScene3Click); 
  //btnScene4.attachClick(btnScene4Click); btnScene5.attachClick(btnScene5Click);

  btnScene1.setClickTicks(100); btnScene2.setClickTicks(100); btnScene3.setClickTicks(100);
  btnScene4.setClickTicks(100); btnScene5.setClickTicks(100);

  redButton.attachClick(redBtnClick); redButton.setClickTicks(100);
  redButton.attachLongPressStart(redBtnLong); redButton.setPressTicks(2000);

  pinMode(DIPSW_1, INPUT_PULLUP); pinMode(DIPSW_2, INPUT_PULLUP); pinMode(DIPSW_3, INPUT_PULLUP);
  pinMode(DIPSW_4, INPUT_PULLUP); pinMode(DIPSW_5, INPUT_PULLUP); pinMode(DIPSW_6, INPUT_PULLUP);
  pinMode(DIPSW_7, INPUT_PULLUP); pinMode(DIPSW_8, INPUT_PULLUP);

  pixels1.begin(); pixels1.show(); pixels2.begin(); pixels2.show(); pixels3.begin(); pixels3.show();

  Serial.begin(115200);
  printf_begin();
  Serial.println("testing leds...");
  ledTest();

  digitalWrite(LED_RED, HIGH);

#ifdef USE_ATEM
  Serial.println("atem begin...");
  pinMode(ETH_RESET, OUTPUT);
  digitalWrite(ETH_RESET, LOW);   // this pin is connected to ethernet shield reset pin
                                  // i cut the connection to the arduino reset to allow me resetting the shield
  delay(100);
  digitalWrite(ETH_RESET, HIGH);
  pinMode(ETH_RESET, INPUT);
  delay(100);
  Ethernet.begin(mac, clientIp);
  atem.begin(atemIp);
  atem.connect();
#endif


#ifdef USE_DMX
  // use hardware serial with registers instead the slow arduino methods
  uint16_t baud_setting = (F_CPU / 4 / 250000 - 1) / 2;
  UCSR1A = (1 << U2X1);
  UBRR1H = (unsigned char)(baud_setting >> 8);
  UBRR1L = (unsigned char)baud_setting;
  UCSR1B = (1 << RXEN1) | (1 << RXCIE1);
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10) | (1 << USBS1); // 8n2
  dmxState = IDLE;
  dmxAddress = readDMXAddress();
  printf("DMX Address: %d\n", dmxAddress);
#endif

#ifdef USE_NRF
  radio.begin(); radio.setAutoAck(1);  radio.enableAckPayload();
  radio.setRetries(0, 15); radio.setPayloadSize(1); radio.printDetails();
#endif

  Serial.println("device ready.\n");
  wdt_enable(WDTO_4S);
}


void ledTest() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  delay(100);

  digitalWrite(LED_TALLY1, HIGH); delay(100);
  digitalWrite(LED_TALLY2, HIGH); delay(100);
  digitalWrite(LED_TALLY3, HIGH); delay(100);
  digitalWrite(LED_TALLY4, HIGH); delay(100);
  digitalWrite(LED_TALLY5, HIGH); delay(100);

  setAllPixels(&pixels1, 20, 0, 0); setAllPixels(&pixels2, 20, 0, 0); setAllPixels(&pixels3, 20, 0, 0);
  delay(100);
  setAllPixels(&pixels1, 0, 20, 0); setAllPixels(&pixels2, 0, 20, 0); setAllPixels(&pixels3, 0, 20, 0);
  delay(100);
  setAllPixels(&pixels1, 0, 0, 20); setAllPixels(&pixels2, 0, 0, 20); setAllPixels(&pixels3, 0, 0, 20);
  delay(100);
  setAllPixels(&pixels1, 0, 0, 0); setAllPixels(&pixels2, 0, 0, 0);  setAllPixels(&pixels3, 0, 0, 0);
  delay(100);

  for(uint8_t i = 0; i < 255; i++) {
    analogWrite(PWM_RING1, i); delay(1);
  }
  delay(100);
  for(uint8_t i = 0; i < 255; i++) {
    analogWrite(PWM_RING2, i); delay(1);
  }
  delay(100);
  for(uint8_t i = 0; i < 255; i++) {
    analogWrite(PWM_RING3, i); delay(1);
  }

  analogWrite(PWM_RING1, 0);
  analogWrite(PWM_RING2, 0);
  analogWrite(PWM_RING3, 0);
  
  digitalWrite(LED_TALLY1, LOW); delay(50);
  digitalWrite(LED_TALLY2, LOW); delay(50);
  digitalWrite(LED_TALLY3, LOW); delay(50);
  digitalWrite(LED_TALLY4, LOW); delay(50);
  digitalWrite(LED_TALLY5, LOW); delay(50);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
}

void outputLEDs() {
  static uint32_t tallyMillis = 0;

  if (millis() - tallyMillis > 200) {
    tallyMillis = millis();
  } else {
    return;
  }

  for(uint8_t i = 0; i<3; i++) {
    if (tally[i] & 0x01 != 0) {
      digitalWrite(tally_pins[i], HIGH);
  
      if(lastKeyerInput == i+1) {
        setAllPixels(&pixels[i], 100, 100, 0);
      } else {
        setAllPixels(&pixels[i], 100, 0, 0);
      }
    } else if(lastKeyerInput == 1) {
      setAllPixels(&pixels[i], 20, 100, 0);
    } else {
      setAllPixels(&pixels[i], 20, 0, 0);
      digitalWrite(tally_pins[i], LOW);
    }
  }

  for(uint8_t i = 0; i<2; i++) {
    if ( tally[i+3] & 0x01 != 0) {
      digitalWrite(tally_pins[i+3], HIGH);
      nrfLeds[i] = 0x01;
    } else {
      digitalWrite(tally_pins[i+3], LOW);
      nrfLeds[i] = 0x00;
    }
  }
}

void setAllPixels(Adafruit_NeoPixel *strip, byte red, byte green, byte blue) {
  for(int i = 0; i < PIXELS; i++) {
    strip->setPixelColor(i, red, green, blue);
  }
  strip->show();
}

uint8_t readDMXAddress() {

  uint8_t addr = 0;
  addr += digitalRead(DIPSW_1) == LOW ? 0x01 : 0;
  addr += digitalRead(DIPSW_2) == LOW ? 0x02 : 0;
  addr += digitalRead(DIPSW_3) == LOW ? 0x04 : 0;
  addr += digitalRead(DIPSW_4) == LOW ? 0x08 : 0;
  addr += digitalRead(DIPSW_5) == LOW ? 0x10 : 0;
  addr += digitalRead(DIPSW_6) == LOW ? 0x20 : 0;
  addr += digitalRead(DIPSW_7) == LOW ? 0x40 : 0;
  addr += digitalRead(DIPSW_8) == LOW ? 0x80 : 0;

  return addr;
}

void redBtnLong() {
  Serial.println("red button long pressed, resetting");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  uint8_t counter;  // endless loop, let WDG trigger reset
  while (true) { counter++; }
}

void redBtnClick() {
#ifdef USE_ATEM
  if (isAtemOnline) {
    if(!enabledTestMode) {
      Serial.println("setting ATEM test mode");
      setInput(1000);
      atem.setProgramInputVideoSource(0, 1000);
      enabledTestMode = true;
    } else {
      Serial.println("disabling ATEM test mode");
      enabledTestMode = false;
      setupKeyer();
      setupSuperSource();
      setInput(1);
      atem.setProgramInputVideoSource(0, 6000);  // SuperSource
    }
  }
#endif
}

void btnA1Click() { setInput(1); }
void btnA2Click() { setInput(2); }
void btnA3Click() { setInput(3); }
void btnB1Click() { setKeyer(1); }
void btnB2Click() { setKeyer(2); }
void btnB3Click() { setKeyer(3); }
void btnScene1Click() { setSuperSource(0); }
void btnScene2Click() { setSuperSource(1); }
void btnScene3Click() { setSuperSource(2); }

void buttonsRemote() {
  static uint16_t lastButton = 0;
  buttons = 0;
  buttons |= (nrfButtons[0] & 0x03) << 3;
  buttons |= (nrfButtons[1] & 0x03) << 5;

  if (buttons != lastButton) {
    printf("button: %d\n", buttons);

#ifdef USE_ATEM
    if (buttons & 0x0008) {
      setInput(4);
    } else if (buttons & 0x0020) {
      setInput(6);
    } 
#endif

    lastButton = buttons;
  }
}

void loop() {

  redButton.tick();
  btnA1.tick(); btnA2.tick(); btnA3.tick();
  btnB1.tick(); btnB2.tick(); btnB3.tick();
  btnScene1.tick(); btnScene2.tick(); 
  btnScene3.tick(); btnScene4.tick();
  btnScene5.tick();
  
#ifdef USE_ATEM
  wdt_reset();
  loopAtem();
  wdt_reset();
#endif

  outputLEDs();

#ifdef USE_NRF
  loopNRF();
#endif

#ifdef USE_DMX
  //printf("dmx: %d\n", readDMXAddress());
  if(millis() - dmxMillis < 500) {
    analogWrite(PWM_RING1, dmxRxField[0]);
    analogWrite(PWM_RING2, dmxRxField[1]);
    analogWrite(PWM_RING3, dmxRxField[2]);
  }
#endif

  buttonsRemote();
  //loopHttp();
  wdt_reset();

  uint32_t timeout = ledBlinkState == LOW ? 50 : 200;
  if (millis() - ledBlinkMillis > timeout) {
    ledBlinkState = ledBlinkState == LOW ? HIGH : LOW;
    digitalWrite(LED_GREEN, ledBlinkState);
    ledBlinkMillis = millis();
  }
}


#ifdef USE_NRF
void loopNRF() {

  if (millis() - nrfMillis > 100) {
    nrfMillis = millis();
  } else {
    return;
  }

  for (uint8_t i = 0; i < MAX_NODES; i++) {
    radio.openWritingPipe(listening_pipes[i]);

    if (!radio.write(&nrfLeds[i], 1)) {
      printf("send failed node: %d\n", i);
    } else {

      if (!radio.available()) {
        //printf("blank payload node: %d\n", i);
      } else {
        //unsigned long tim = micros();
        uint8_t payload;
        radio.read(&payload, 1);
        //        printf("node %d response %d\n", i, payload);

        rxMillis[i] = millis();
        nrfButtons[i] = payload;
      }
    }
    wdt_reset();
    radio.stopListening();
  }
}
#endif

#ifdef USE_ATEM

void setInput(uint16_t program) {
  printf("input: %d\n", program);
  atem.setProgramInputVideoSource(1, program);
  setSuperSource(0);
}

void setKeyer(uint8_t program) {
  
  if(program != lastKeyerInput) {
    // change keyer source
    printf("keyer %d\n", program);
    atem.setKeyerFillSource(0, 0, program);
    atem.setKeyerOnAirEnabled(0, 0, true);
    lastKeyerInput = program;
  } else {
    // source is the same, so disable
    printf("keyer off\n");
    atem.setKeyerOnAirEnabled(0, 0, false);
    lastKeyerInput = -1;
  }
  
}

void setupKeyer() {
  atem.setKeyerType(0, 0, 1);
  atem.setKeyChromaHue(0, 0, 1123);
  atem.setKeyChromaGain(0, 0, 741);
  atem.setKeyChromaYSuppress(0, 0, 1000);
  atem.setKeyChromaLift(0, 0, 0);
  atem.setKeyChromaNarrow(0, 0, false);
}

void setupSuperSource() {
  atem.setSuperSourceFillSource(0);
  atem.setSuperSourcePreMultiplied(false);
  atem.setSuperSourceBorderEnabled(false);
  atem.setSuperSourceBoxParametersEnabled(0, true);
  atem.setSuperSourceBoxParametersEnabled(1, false);
  atem.setSuperSourceBoxParametersEnabled(2, false);
  atem.setSuperSourceBoxParametersEnabled(3, false);
  atem.setSuperSourceBoxParametersInputSource(0, 10020);   //ME 2 Prog
  atem.setSuperSourceBoxParametersInputSource(1, 1);
  atem.setSuperSourceBoxParametersInputSource(2, 2);
  atem.setSuperSourceBoxParametersInputSource(3, 3);
  
  atem.setSuperSourceBoxParametersPositionX(0, 0);
  atem.setSuperSourceBoxParametersPositionY(0, 0);
  atem.setSuperSourceBoxParametersSize(0, 1000);
  
  atem.setSuperSourceBoxParametersPositionX(1, 1100);
  atem.setSuperSourceBoxParametersPositionY(1, 00);
  atem.setSuperSourceBoxParametersSize(1, 300);
  
  atem.setSuperSourceBoxParametersPositionX(2, 0);
  atem.setSuperSourceBoxParametersPositionY(2, 0);
  atem.setSuperSourceBoxParametersSize(2, 300);
  
  atem.setSuperSourceBoxParametersPositionX(3, -1100);
  atem.setSuperSourceBoxParametersPositionY(4, 0);
  atem.setSuperSourceBoxParametersSize(3, 300);
  
  atem.setSuperSourceBoxParametersCropped(0, false);
}

void setSuperSource(uint8_t mode) {

  printf("set ssrc: %d\n", mode);

  switch(mode) {
    case 0:
      atem.setSuperSourceBoxParametersEnabled(0, true);
      atem.setSuperSourceBoxParametersEnabled(1, false);
      atem.setSuperSourceBoxParametersEnabled(2, false);
      atem.setSuperSourceBoxParametersEnabled(3, false);
      break;
    case 1:
      atem.setSuperSourceBoxParametersEnabled(0, false);
      atem.setSuperSourceBoxParametersInputSource(1, 4);
      atem.setSuperSourceBoxParametersInputSource(2, 1);
      atem.setSuperSourceBoxParametersInputSource(3, 6);
      atem.setSuperSourceBoxParametersEnabled(1, true);
      atem.setSuperSourceBoxParametersEnabled(2, true);
      atem.setSuperSourceBoxParametersEnabled(3, true);
      break;
    case 2:
      atem.setSuperSourceBoxParametersEnabled(0, false);
      atem.setSuperSourceBoxParametersInputSource(1, 1);
      atem.setSuperSourceBoxParametersInputSource(2, 4);
      atem.setSuperSourceBoxParametersInputSource(3, 6);
      atem.setSuperSourceBoxParametersEnabled(1, true);
      atem.setSuperSourceBoxParametersEnabled(2, true);
      atem.setSuperSourceBoxParametersEnabled(3, true);
      break;
  }
  
}

void loopAtem() {
  static uint32_t atemMillis = 0;

  if (millis() - atemMillis > 100) {
    atemMillis = millis();
  } else {
    return;
  }

  atem.runLoop();

  if (atem.hasInitialized())  {
    if (!isAtemOnline)  {
      isAtemOnline = true;
      digitalWrite(LED_RED, LOW);
      setupKeyer();
      setupSuperSource();

      // ME 1 is for selecting super source only 
      atem.setProgramInputVideoSource(0, 6000);  // preset SuperSource
      
      // ME 2 is for selecting the input. goes int SuperSource Box 1
      setInput(1); // preset input 1
    }

    tally[0] = atem.getTallyByIndexTallyFlags(0) & 0x03;
    tally[1] = atem.getTallyByIndexTallyFlags(1) & 0x03;
    tally[2] = atem.getTallyByIndexTallyFlags(2) & 0x03;
    tally[3] = atem.getTallyByIndexTallyFlags(3) & 0x03;
    tally[4] = atem.getTallyByIndexTallyFlags(5) & 0x03;

  } else {
    // at this point the ATEM is not connected and initialized anymore

    if (isAtemOnline) {
      isAtemOnline = false;
      digitalWrite(LED_RED, HIGH);

      for (uint8_t i = 0; i < ATEM_CHANNEL; i++) {
        tally[i] = 0;
      }
    }
  }
}
#endif

#ifdef USE_DMX
ISR(USART1_RX_vect) {
  //void serialEvent1() {
  static  uint16_t dmxCount;
  uint8_t  USARTstate = UCSR1A;    //get state before data!
  uint8_t  dmxByte    = UDR1;      //get data
  uint8_t  dmxStateL  = dmxState;  //just load once from SRAM to increase speed

  if (USARTstate & (1 << FE1)) {  //check for break
    dmxCount = dmxAddress;        //reset channel counter (count channels before start address)
    dmxState = BREAK;
  } else if (dmxStateL == BREAK) {

    if (dmxByte == 0) {
      dmxState = STARTB;          //normal start code detected
    } else {
      dmxState = IDLE;
    }
  } else if (dmxStateL == STARTB) {

    if (--dmxCount == 0) {        //start address reached?
      dmxCount = 1;               //set up counter for required channels
      dmxRxField[0] = dmxByte;    //get 1st DMX channel of device
      dmxState = STARTADR;
    }
  } else if (dmxStateL == STARTADR) {

    dmxRxField[dmxCount++] = dmxByte;       //get channel

    if (dmxCount >= sizeof(dmxRxField)) {   //all ch received?
      dmxState = IDLE;            //wait for next break
      dmxMillis = millis();
    }
  }

}
#endif

/*
  void loopHttp() {

  EthernetClient client = server.available();
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK\r\nContent-Type: text/html");
          client.println("Connection: close\r\n");
          client.println("<!DOCTYPE HTML><html>");
          // output the value of each nrf-client voltage
          for (byte i=0; i < MAX_NODES; i++) {
            client.print("node ");
            client.print(i);
            client.print(": buttons: ");
            client.print(buttons);
            client.print(" millis: ");
            client.print(rxMillis[i]);
            client.println("<br />");
          }

          client.print("<br/>Atem connection: ");
          client.println(isAtemOnline ? "Online":"Offline");
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
  }
*/
