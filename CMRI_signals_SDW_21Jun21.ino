/* This sketch combines servo turnout control and sensor inputs - acknowledge Rob of Little Wicket Railway for bare bones
  of this, and Chris Sharp for discussions on code
  I have added a routine to stop servo chatter by switching power to servos off when not moving
  Sketch is based on an Arduino Mega - a Nano could be used with less inputs but low memory warning!

  Pins 0, 1 and 2 are protected since these are used for communication to the JMRI computer (Raspberry Pi 3B+)
  Pin 3 is used to turn power to the servos on and off
  (Signal lights have a separate power line always on to the signal light boards)
  Pin 13 is the Arduino LED pin and should not be used as an input, but can be used as an output for some applications
  Pins 20 and 21 are reserved for PWM servo control
  64 pins will be used for sensor inputs - these are only good for slow sensors as the delays due to servo movement may not
  pick up a change in time for desired action.  The DCC++EX Command Station is used for most sensors
*/

// The Arduino is set up to behave like a CMRI hardware SUSIC with up to 64 slots
// Each slot has either a 24 or 32 bit input/output card
// 64 x 32 bit cards gives up to 2048 input/outputs!
// However, the SUSIC is set up with the required inputs/outputs to make the process more efficient.
// Cards 0 and 1 are the sensor inputs (up to 64 inputs) and sub-cards 2,3 and 4 support 48 servo outputs on PCA9685 boards
// Cards 5 and 6 support 10 3 aspect signals on PCA9685 boards


// Include libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CMRI.h>

// CMRI Settings
#define CMRI_ADDR 1 //CMRI node address in JMRI

// Define the number of servos and signals connected
#define numServos 48
#define numServo1 16 // Number of servos on PCA9685 board 1
#define numServo2 16 // Number of servos on PCA9685 board 2
#define numServo3 16 // Number of servos on PCA9685 board 3

// The number of signals connected
#define numSignals 32
#define numSignal1 16 // Number of signal Leds on PCA9685 board 4
#define numSignal2 16 // Number of signal Leds on PCA9685 board 5


// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x40); //setup the board address - defaults to 0x40 if not specified
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x42);
Adafruit_PWMServoDriver pwm4 = Adafruit_PWMServoDriver(0x43);
Adafruit_PWMServoDriver pwm5 = Adafruit_PWMServoDriver(0x44);


// Define CMRI connection with 64 inputs and 96 outputs
CMRI cmri(CMRI_ADDR, 64, 96);

// Create tables to hold data about the servo positions
bool Status[(numServos + numSignals)]; //Create a table to hold the status bit of each turnout, signal, etc.
bool Previous[(numServos + numSignals)]; // Create a record of previous status bit so only changes are actioned
int Throw[(numServos + numSignals)]; //Create a table to hold the throw value for each servo / signal
int Close[(numServos + numSignals)]; //Create a table to hold the close value for each servo / signal
int CurrentPosition[numServos]; //Create a table to hold the current value for each servo during movement
int Target[numServos]; //Create a table to hold the target value for each servo during movement
int increment[numServos]; // a variable of either 1 or -1 depending on whether servo moving up or down
int POWERON = 3;      // A relay board attached to pin 3 which switches power to the servos when needed - stops servo chatter
unsigned long PreviousMillis[numServos];   // Time that the loop action was last changed
int ServoDelay;  //  delay in milliseconds to the next move of the servo
int movecounter; // starts at 0 then adds 1 for every move request, subtract 1 when move completed.
bool moving[numServos]; // checks whether a servo is currently still moving when a new request is received

void setup() {

  // Set pins 4-19 and 22-69 as input pins for sensors

  for (int i = 4; i <= 19; i++)  {
    pinMode(i, INPUT_PULLUP);       // set sensor shield pins 3-19 as inputs - if using a Nano start at 6
  }

  //  for (int i = 22; i <= 69; i++)  {  // uncomment these lines if using senors on a Mega
  //    pinMode(i, INPUT_PULLUP);       // set sensor shield pins 22-69 as inputs
  //  }
  ServoDelay = 10;
  movecounter = 0;

  // Start the serial connection to JMRI computer
  Serial.begin(19200);  //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data

  // Initialise PCA9685 boards
  pwm1.begin();
  pwm1.setPWMFreq(50);  // This is the maximum PWM frequency for servos
  pwm2.begin();
  pwm2.setPWMFreq(50);  // 
  pwm3.begin();
  pwm3.setPWMFreq(50);  //
  pwm4.begin();
  pwm4.setPWMFreq(1000);  // This is the maximum PWM frequency for LEDs
  pwm5.begin();
  pwm5.setPWMFreq(1000);  //

  // SET THE THROW AND CLOSE VALUES FOR EACH SERVO BASED ON THE CALIBRATION PROCESS (pulse_width)

  //Servo connection 0 - 47 point motors
  Throw[0] = 240;  // TO8 new servo
  Close[0] = 335;  //
  Throw[1] = 325;  // TO10
  Close[1] = 295;  //
  Throw[2] = 330;  // TO15
  Close[2] = 255;  //
  Throw[3] = 327;  // TO16
  Close[3] = 220;  //
  Throw[4] = 330;  // TO34 new servo
  Close[4] = 410;  //
  Throw[5] = 390;  // TO35
  Close[5] = 350;  //
  Throw[6] = 415;  // TO28
  Close[6] = 230;  //
  Throw[7] = 425;  // TO29
  Close[7] = 270;  //
  Throw[8] = 380;  // TO22
  Close[8] = 300;  //
  Throw[9] = 240;  // TO
  Close[9] = 280;  //
  Throw[10] = 240;  // TO
  Close[10] = 280;  //
  Throw[11] = 240;  // TO
  Close[11] = 280;  //
  Throw[12] = 240;  // TO
  Close[12] = 280;  //
  Throw[13] = 235;  // TO
  Close[13] = 290;  //
  Throw[14] = 240;  // TO
  Close[14] = 280;  //
  Throw[15] = 240;  // TO
  Close[15] = 280;  //
  Throw[16] = 178;  // TO23
  Close[16] = 431;  //
  Throw[17] = 430;  // TO32
  Close[17] = 202;  //
  Throw[18] = 281;  // TO27
  Close[18] = 354;  //
  Throw[19] = 430;  // TO30
  Close[19] = 230;  //
  Throw[20] = 292;  // TO1
  Close[20] = 370;  //
  Throw[21] = 320;  // TO2
  Close[21] = 345;  //
  Throw[22] = 320;  // TO4
  Close[22] = 279;  //
  Throw[23] = 290;  // TO5
  Close[23] = 340;  //
  Throw[24] = 340;  // TO36
  Close[24] = 229;  //
  Throw[25] = 260;  // TO11
  Close[25] = 300;  //
  Throw[26] = 360;  // TO14
  Close[26] = 290;  //
  Throw[27] = 290;  // TO7
  Close[27] = 330;  //
  Throw[28] = 355;  // TO25
  Close[28] = 255;  //
  Throw[29] = 295;  // TO26
  Close[29] = 390;  //
  Throw[30] = 235;  // TO6
  Close[30] = 345;  //
  Throw[31] = 385;  // TO13
  Close[31] = 340;  //
  Throw[32] = 380;  // TO8
  Close[32] = 275;  //
  Throw[33] = 285;  // TO10
  Close[33] = 330;  //
  Throw[34] = 255;  // TO15
  Close[34] = 330;  //
  Throw[35] = 220;  // TO16
  Close[35] = 327;  //
  Throw[36] = 220;  // TO34
  Close[36] = 282;  //
  Throw[37] = 350;  // TO35
  Close[37] = 290;  //
  Throw[38] = 400;  // TO28
  Close[38] = 207;  //
  Throw[39] = 390;  // TO29
  Close[39] = 228;  //
  Throw[40] = 370;  // TO22
  Close[40] = 300;  //
  Throw[41] = 240;  // TO
  Close[41] = 280;  //
  Throw[42] = 240;  // TO
  Close[42] = 280;  //
  Throw[43] = 240;  // TO
  Close[43] = 280;  //
  Throw[44] = 240;  // TO
  Close[44] = 280;  //
  Throw[45] = 235;  // TO
  Close[45] = 290;  //
  Throw[46] = 240;  // TO
  Close[46] = 280;  //
  Throw[47] = 240;  // TO
  Close[47] = 280;  //

  // Signals = all the same values but presented this way to comment which signal is linked in JMRI Turnouts Table
  Throw[48] = 4095;  // CMRI address 1049 Sig1 Platform 1 bypass green
  Throw[49] = 4095;  // yellow
  Throw[50] = 4095;  // red
  Throw[51] = 4095;  // CMRI address 1052 Sig4 Platform 1 green
  Throw[52] = 4095;  // yellow
  Throw[53] = 4095;  // red
  Throw[54] = 4095;  // CMRI address 1055 Sig7 Platform 2 reverse green
  Throw[55] = 4095;  // yellow
  Throw[56] = 4095;  // red
  Throw[57] = 4095;  // CMRI address 1058 Sig10 Platform 2 reverse bypass green
  Throw[58] = 4095;  // yellow
  Throw[59] = 4095;  // red
  Throw[60] = 4095;  //
  Throw[61] = 4095;  //
  Throw[62] = 4095;  //
  Throw[63] = 4095;  //
  Throw[64] = 4095;  // CMRI address 1065 Sig13 Platform 2 bypass green
  Throw[65] = 4095;  // yellow
  Throw[66] = 4095;  // red
  Throw[67] = 4095;  // CMRI address 1068 Sig16 Platform 2 green
  Throw[68] = 4095;  // yellow
  Throw[69] = 4095;  // red
  Throw[70] = 4095;  // CMRI address 1071 Sig19 Platform 1 reverse green
  Throw[71] = 4095;  // yellow
  Throw[72] = 4095;  // red
  Throw[73] = 4095;  // CMRI address 1074 Sig22 Platform 1 bypass reverse green
  Throw[74] = 4095;  // yellow
  Throw[75] = 4095;  // red
  Throw[76] = 4095;  //
  Throw[77] = 4095;  //
  Throw[78] = 4095;  //
  Throw[79] = 4095;  //
  for (int i = numServos; i < (numServos + numSignals); i++) Close[i] = 0;  //  Data for light off
  pinMode(POWERON, OUTPUT);
  digitalWrite(POWERON, HIGH);
  delay(200);
  cmri.process();

  for (int i = 0; i < (numServos + numSignals); i++) {
    Previous[i] = (cmri.get_bit(i));
    Status[i] = Previous[i];
  }
  for ( int i = 0; i < numServos; i++) { // Initialise the current position of all servos
    if (Status[i] == 1) {
      CurrentPosition[i] = Close[i];
      Target[i] = Close[i];  // nothing needs to change yet!
    }
    else {
      CurrentPosition[i] = Throw[i];
      Target[i] = Throw[i];
    }
    increment[i] = 1;
    PreviousMillis[i] = millis();
    moving[i] = false;
  }
}

void loop() {
  cmri.process();

  // PROCESS SERVOS
  // Servos start on bit 0 which corresponds to output address CMRI 1001 in JMRI
  for (int i = 0; i < numServos; i++) {
    Status[i] = (cmri.get_bit(i));
    if (Status[i] != Previous[i]) {      // only turn the power to the servos on if something has changed - once on it stays on until all moves complete
      Previous[i] = Status[i];
      digitalWrite(POWERON, LOW);
      if (moving[i] == false ) {
        movecounter = movecounter + 1;
        moving [i] =true;
      }

      if (Status[i] == 1) {
        CurrentPosition[i] = Close[i];
        Target[i] = Throw[i];
      }
      else {
        CurrentPosition[i] = Throw[i];
        Target[i] = Close[i];
      }
      if (Target[i] > CurrentPosition[i]) increment[i] = 1;
      else increment[i] = -1;
    }

    // Now check whether enough time has elapsed before moving any servos set to move above
    if ((CurrentPosition[i] != Target[i]) && (millis() > (PreviousMillis[i] + ServoDelay))) {
      PreviousMillis[i] = millis();
      CurrentPosition[i] = CurrentPosition[i] + increment[i];
      if (i < numServo1) {
        pwm1.setPWM(i, 0, CurrentPosition[i]);
      }
      else if (i >= numServo1 + numServo2) {
        pwm3.setPWM(i - numServo1 - numServo2, 0, CurrentPosition[i]);
      }
      else {
        pwm2.setPWM(i - numServo1, 0, CurrentPosition[i]);
      }

      if ((CurrentPosition[i] == Target[i])) {  // if this servo has finished moving set the movecounter down 1
        movecounter =  movecounter - 1;
        moving[i] = false;
      }
    }
  }
  if (movecounter < 1) {
    digitalWrite(POWERON, HIGH);
    movecounter = 0;
  }

  //Process the signal boards of 16 lights each
  for (int i = numServos; i < (numServos + numSignals); i++) {
    Status[i] = (cmri.get_bit(i));
    if (Status[i] != Previous[i]) {      // only change if something has changed
      Previous[i] = Status[i];
      if (Status[i] == 1) {
        if (i < numServos + numSignal1) pwm4.setPWM((i - numServos), Close[i], Throw[i]);
        else pwm5.setPWM(i - numServos - numSignal1, Close[i], Throw[i]);
      }
      else {
        if (i < numServos + numSignal1) pwm4.setPWM((i - numServos), Throw[i], Close[i]);
        else pwm5.setPWM(i - numServos - numSignal1, Throw[i], Close[i]);
      }
    }
  }

  // PROCESS SENSORS
  // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required
  // Ensure bit address matches pin, i.e. a sensor attached to pin 17 corresponds to bit 13 (because we've skipped pins 0, 1, 2 and 13) which is address 1014 for this CMRI node

  // Do not read 0, 1, 2, or 3
  //  cmri.set_bit(0, !digitalRead(4));  //Bit 0 = address 1001 in JMRI
  //cmri.set_bit(1, !digitalRead(5));  //Bit 1 = address 1002 in JMRI
  //cmri.set_bit(2, !digitalRead(6));  //Bit 2 = address 1003 in JMRI
  //etc.
  //Do not read 13
  //Do not read 20 or 21
}
