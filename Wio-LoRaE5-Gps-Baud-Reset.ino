#include "TFT_eSPI.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial softSerial(3,2); // RX TX (use 4 instead of 3)
#define GPSSerial softSerial
Adafruit_GPS GPS(&GPSSerial);
TFT_eSPI tft;

#define FS9 &FreeSerif9pt7b
#define GFXFF 1
#define TFT_GRAY    0b1010010100010000

#define LINEHEIGHT  20


// ---------------------------------------------------------------
// Custom SoftSerial to support WRITE at 115200bps

uint32_t reg_mask;
uint32_t inv_mask;
volatile PORT_OUT_Type * reg;
double dbit_delay_nano = (1/115200.0)*1000000000;
void SetupFastSerial(uint8_t tx) {
  digitalWrite(tx, HIGH);
  pinMode(tx, OUTPUT);
  reg_mask = digitalPinToBitMask(tx);
  inv_mask = ~digitalPinToBitMask(tx);
  PortGroup * port = digitalPinToPort(tx);
  reg = portOutputReg(port);
}


size_t WriteFastSerial(uint8_t b)
{
  uint32_t ibit_delay_nano = 8680; //(uint32_t) dbit_delay_nano;

  uint32_t start = micros();
  uint32_t stop = start + (ibit_delay_nano)/1000; 
  // Write the start bit
  reg->reg &= inv_mask;
  delayMicroseconds(8);
  //while ( micros() < stop ); 

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(8);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(9);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(8);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(9);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(8);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(9);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(8);
    b >>= 1;

    if (b & 1) // choose bit
      reg->reg |= reg_mask; // send 1
    else
      reg->reg &= inv_mask; // send 0
    delayMicroseconds(8);
    b >>= 1;

    // restore pin to natural state
    reg->reg |= reg_mask;
    ibit_delay_nano += ibit_delay_nano;
    delayMicroseconds(10); 
    return 1;
}

// ---------------------------------------------------------------

boolean gpsLoop() {
  
  char c = GPS.read();  
  if (GPS.newNMEAreceived()) {
    // Since this function change internal library var,
    // avoid multiple call, just once and use results later
    Serial.println(GPS.lastNMEA());
    if (GPS.parse(GPS.lastNMEA())) {
      return true;
    }
  }
  return false;
  
}


void setup() {
  char title[128];
  int y = 10;

  // put your setup code here, to run once:
  Serial.begin(9600);
  uint32_t start = millis();
  while ( !Serial && (millis() - start) < 2000 );

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GRAY);
  tft.setFreeFont(FS9);     // Select the orginal small TomThumb font

  sprintf(title,"Verifying setup");
  tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 

  delay(250);
  GPS.begin(9600);
  GPSSerial.listen();
  delay(1200);

  // try to read a NMEA line for 5 seconds
  start = millis();
  boolean found = false;
  while ( ! found && (millis() - start) < 5000 ) {
    found = gpsLoop();
  }

  if ( found ) {
    // the device is already correctly setup
    sprintf(title,"Setup ok, nothing to do");
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
    //GPS.sendCommand("$PQBAUD,W,115200*43"); // This is to test on a working board
    while(1);
  } else {
    sprintf(title,"Reconfiguration is needed");
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
  }

  // Reconfigure
  
  uint8_t cmd[] = "$PQBAUD,W,9600*4B\r\n";
  SetupFastSerial(2);
  for ( int k = 0 ; k < 2 ; k ++ ) {
    sprintf(title,"Reconfiguration trial %d",k+1);
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
    for ( int i = 0 ; cmd[i] > 0 ; i++) {
      //Serial.print((char)cmd[i]);
      WriteFastSerial(cmd[i]);
    }
    delay(5000);
  }

  sprintf(title,"Reconfiguration done ");
  tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 

  // Verify
  delay(250);
  GPS.begin(9600);
  GPSSerial.listen();
  delay(1200);

  start = millis();
  found = false;
  while ( !found && (millis() - start) < 10000 ) {
    found = gpsLoop();
  }

  if ( found ) {
    // the device is  correctly setup
    sprintf(title,"Device is correctly setup");
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
    sprintf(title,"Reflash WioFieldTester now");
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
    while(1);
  } else {
    sprintf(title,"Reconfiguration failed");
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
    sprintf(title,"Restart Wio to try again");
    tft.drawString(title,5, y, GFXFF); y+= LINEHEIGHT; 
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
