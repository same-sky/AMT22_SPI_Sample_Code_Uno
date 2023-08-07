/*
 * AMT22_Multiurn_SPI_Sample_Code_Uno.ino
 * Company: CUI Devices
 * Author: Jason Kelly, Damon Tarry
 * Version: 2.0.2.0
 * Date: July 18, 2023
 *
 * This sample code can be used with the Arduino Uno to control the AMT22 encoder.
 * It uses SPI to control the encoder and the the Arduino UART to report back to the PC
 * via the Arduino Serial Monitor.
 * For more information or assistance contact CUI Devices for support.
 *
 * After uploading code to Arduino Uno open the open the Serial Monitor under the Tools
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT22.
 *
 * Arduino Pin Connections
 * SPI Chip Select Enc 0:   Pin  2
 * SPI Chip Select Enc 1:   Pin  3
 * SPI MOSI                 Pin 11
 * SPI MISO                 Pin 12
 * SPI SCLK:                Pin 13
 *
 *
 * AMT22 Pin Connections
 * Vdd (5V):                Pin  1
 * SPI SCLK:                Pin  2
 * SPI MOSI:                Pin  3
 * GND:                     Pin  4
 * SPI MISO:                Pin  5
 * SPI Chip Select:         Pin  6
 *
 *
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */


/* Include the SPI library for the arduino boards */
#include <SPI.h>

/* Serial rates for UART */
#define BAUDRATE        115200

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70
#define AMT22_TURNS     0xA0

/* We will use this define macro so we can write code once compatible with 12 or 14 bit encoders */
#define RESOLUTION      12

/* SPI pins */
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13

//create an array containing CS pin numbers for all connected encoders
uint8_t cs_pins[] = {2}; //only one encoder connected, using pin 2 on arduino for CS
//uint8_t cs_pins[] = {2, 3}; //two encoders connected, using pins 2 & 3 on arduino for CS

void setup()
{
  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  for(int encoder = 0; encoder < sizeof(cs_pins); ++encoder)
  {
    pinMode(cs_pins[encoder], OUTPUT);
    
    //Set the CS line high which is the default inactive state
    digitalWrite(cs_pins[encoder], HIGH);
  }

  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);

  //set the clockrate. Uno clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV64);  // 250 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV128); // 125 kHz

  //start SPI bus
  SPI.begin();

  //if you want to set the zero position before beginning uncomment the following function call
  //setZeroSPI(ENC_0);
  //setZeroSPI(ENC_1);
}

void loop()
{
  for(int encoder = 0; encoder < sizeof(cs_pins); ++encoder)
  {
    //set the CS signal to low
    digitalWrite(cs_pins[encoder], LOW);
    delayMicroseconds(3);

    //read the two bytes for position from the encoder, starting with the high byte
    uint16_t encoderPosition = SPI.transfer(AMT22_NOP) << 8; //shift up 8 bits because this is the high byte
    delayMicroseconds(3);
    encoderPosition |= SPI.transfer(AMT22_TURNS); //we send the turns command (0xA0) here, to tell the encoder to send us the turns count after the position

    //wait 40us before reading the turns counter
    delayMicroseconds(40);

    //read the two bytes for turns from the encoder, starting with the high byte
    uint16_t encoderTurns = SPI.transfer(AMT22_NOP) << 8; //shift up 8 bits because this is the high byte
    delayMicroseconds(3);
    encoderTurns |= SPI.transfer(AMT22_NOP);
    delayMicroseconds(3);

    //set the CS signal to high
    digitalWrite(cs_pins[encoder], HIGH);

    if (verifyChecksumSPI(encoderPosition)) //position was good, print to serial stream
    {
      encoderPosition &= 0x3FFF; //discard upper two checksum bits
      if (RESOLUTION == 12) encoderPosition = encoderPosition >> 2; //on a 12-bit encoder, the lower two bits will always be zero

      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" position: ");
      Serial.print(encoderPosition, DEC); //print the position in decimal format
      Serial.write('\n');
    }
    else //position is bad, let the user know how many times we tried
    {
      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" position error.\n");
    }

    if (verifyChecksumSPI(encoderTurns)) //turns was good, print to serial stream
    {
      encoderTurns &= 0x3FFF; //discard upper two checksum bits

      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" turns: ");
      Serial.print(encoderTurns, DEC); //print the turns in decimal format
      Serial.write('\n');
    }
    else //turns is bad, let the user know how many times we tried
    {
      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" turns error.\n");
    }
  }

  //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
  //delay() is in milliseconds0
  delay(500);
}

/*
 * Using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent.
 */
bool verifyChecksumSPI(uint16_t message)
{
  //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  //set CS to low
  digitalWrite(encoder, LOW);
  delayMicroseconds(3);

  //send the first byte of the command
  SPI.transfer(AMT22_NOP);
  delayMicroseconds(3);

  //send the second byte of the command
  SPI.transfer(AMT22_ZERO);
  delayMicroseconds(3);
  
  //set CS to high
  digitalWrite(encoder, HIGH);

  delay(250); //250 millisecond delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  //set CS to low
  digitalWrite(encoder, LOW);
  delayMicroseconds(3);

  //send the first byte of the command
  SPI.transfer(AMT22_NOP);
  delayMicroseconds(3);

  //send the second byte of the command
  SPI.transfer(AMT22_RESET);
  delayMicroseconds(3);
  
  //set CS to high
  digitalWrite(encoder, HIGH);

  delay(250); //250 millisecond delay to allow the encoder to reset
}
