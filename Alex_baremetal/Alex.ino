#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <avr/sleep.h>

#include "packet.h"
#include "constants.h"


#define PRR_TWI_MASK           0b10000000
#define PRR_SPI_MASK           0b00000100
#define ADCSRA_ADC_MASK        0b10000000
#define PRR_ADC_MASK           0b00000001
#define PRR_TIMER2_MASK        0b01000000
#define PRR_TIMER0_MASK        0b00100000
#define PRR_TIMER1_MASK        0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK    0b11110001

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  (1<<6)   // Left forward pin
#define LR                  (1<<5)   // Left reverse pin
#define RF                  (1<<3)  // Right forward pin
#define RR                  (1<<2)  // Right reverse pin

#define S0                  7
#define S1                  8
#define S2                  9
#define S3                  12
#define sensorOut           13

//#define PI                  3.141592654

#define ALEX_LENGTH         17
#define ALEX_BREADTH        11

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;


unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

unsigned long pastMillis;

volatile unsigned long redFrequency = 0;
volatile unsigned long blueFrequency = 0;
volatile unsigned long greenFrequency = 0;
volatile unsigned long whiteFrequency = 0;

/*

   Alex Communication Routines.

*/

void WDT_off(void)
{
  /* Global interrupt should be turned OFF here if not already done so */
  /* Clear WDRF in MCUSR */
  cli();
  MCUSR &= ~(1 << WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei();
  /* Global interrupt should be turned ON here if subsequent operations after calling this function DO NOT require turning off global interrupt */
}

void setupPowerSaving() {
  WDT_off();
  PRR |= PRR_TWI_MASK;
  PRR |= PRR_SPI_MASK;
  ADCSRA |= ADCSRA_ADC_MASK;
  PRR |= PRR_ADC_MASK;
  SMCR |= SMCR_IDLE_MODE_MASK;
  DDRB |= 0b00100000;
  PORTB &= 0b00100000;
}

void putArduinoToIdle() {
  //PRR |= PRR_TIMER0_MASK;
  //PRR |= PRR_TIMER1_MASK;
  //PRR |= PRR_TIMER2_MASK;
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  sleep_cpu();
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  //PRR &= ~PRR_TIMER0_MASK;
  //PRR &= ~PRR_TIMER1_MASK;
  //PRR &= ~PRR_TIMER2_MASK;


}

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

void sendColour() {
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = redFrequency;
  colourPacket.params[1] = blueFrequency;
  colourPacket.params[2] = greenFrequency;
  colourPacket.params[3] = whiteFrequency;

  sendResponse(&colourPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
  //DDRB &= 0b11110011;
  //PORTB |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(TIMER0_COMPA_vect) {
}

ISR(TIMER0_COMPB_vect) {
}

ISR(TIMER1_COMPB_vect) {
}

ISR(TIMER2_COMPA_vect) {
}

// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00100100;
  UCSR0A = 0;
  Serial.begin(9600);

}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
  DDRD |= (LF | LR);
  DDRB |= (RF | RR);
  TCNT0 = 0;
  TCNT1 = 0;
  TCNT2 = 0;
  TCCR0B = 0b00000011; //CLK64
  TCCR1B = 0b00000011; //CLK64
  TCCR2B = 0b00000011; //CLK64
  TIMSK0 |= 0b110;     //ENABLE
  TIMSK1 |= 0b100;
  TIMSK2 |= 0b010;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  TCCR1A = 0b00100001;
  TCCR2A = 0b10000001;
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward()
{
  dir = FORWARD;
  TCCR0A = 0b10000001;  
  OCR0A = 110;          //LF
  OCR1B = 0;            //LR
  OCR0B = 0;            //RR
  OCR2A = 100;          //RF
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse()
{
  dir = BACKWARD;
  TCCR0A = 0b00100001;
  OCR0A = 0;          
  OCR1B = 110;
  OCR0B = 100;
  OCR2A = 0;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left()
{
  dir = LEFT;
  pastMillis = millis();
  TCCR0A = 0b00100001;
  OCR0A = 110;          
  OCR1B = 0;
  OCR0B = 100;
  OCR2A = 0;
  while (millis() - pastMillis < 150);
  stop();
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right()
{
  dir = RIGHT;
  pastMillis = millis();
  TCCR0A = 0b10000001;
  OCR0A = 0;          
  OCR1B = 110;
  OCR0B = 0;
  OCR2A = 100;
  while (millis() - pastMillis < 150);
  forward();
  stop();
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  TCCR0A = 0b00000001;
  OCR0A = 0;          
  OCR1B = 0;
  OCR0B = 0;
  OCR2A = 0;
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{

  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;


  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;

  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void scanColour() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  //redFrequency = map(redFrequency, 50, 160, 255, 0);
  delay(200);

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);
  //greenFrequency = map(greenFrequency, 95, 210, 255, 0);

  delay(200);

  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);
  delay(200);
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward();
      break;
    case COMMAND_REVERSE:
      sendOK();
      reverse();
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left();
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right();
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      break;
    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

    case COMMAND_COLOUR:
      sendOK();
      scanColour();
      sendColour();
      break;

    /*
       Implement code for other commands here.

    */

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  //AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  //AlexCirc = PI * AlexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  sei();

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2


  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  if (dir == STOP) {
    putArduinoToIdle();
  }

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    stop();
    sendBadPacket();
    while (Serial.available() > 0) {
      char t = Serial.read();
    }
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    stop();
    sendBadChecksum();
    while (Serial.available() > 0) {
      char t = Serial.read();
    }
  }


}
