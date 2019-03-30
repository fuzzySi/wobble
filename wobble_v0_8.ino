
/*
    3 (4) EGs / LFOs for arduino, into a modular.
    This controls 4 12bit DAC buffered outputs. These can be set as EG, LFO or direct pot CV.
    DAC is MCP4822 - this is 8pin, dual channel 12 bit DAC (4922 is 14 pin, same spec. MCP4821 is single channel, not pin compatible).
    3 incoming gates trigger EGs, or restarts LFOs.
    Holding a channel button access 2ndry pot functions: attack, release, duty / phase, waveform. these only do something where waveform affected but could modulate something here.

    short press - triggers waveform start
    long press (1 sec or so) toggles reset mode to internal or triggered single shot
    press & move pot changes waveform, set by all 3 pots
    long hold (>2 secs) then let go without tweaking a pot goes into edit mode
      in edit mode:
        top pot does attack, phase,
        middle pot does envelope sustain time.
        lower pot does release, feedback, duty cycle

    options for 4th channel (ch 3):
      mix of 1st 3 (coded)
      delayed average of 1st 3 - a bit smeared
      short envelope signal on trigger from input 3, irrespective of whether in LFO or EG mode.
      one of others inverted (bit dull)

    **********
   version history

    v 0.8 demo version for machina bristonica.
      midi not written, sine wave a bit triangular at lower end.
      pots latch but jumps when next turned, setup option to get back to original point first, more musical
      interpolations between sine & square quite weird - might work.
      midi / CV sequences might be interesting

   share and share alike creative commons licence, fuzzySynths 2019
     some code ideas borrowed from http://abrammorphew.com/notes/2012/01/05/arduino-lfo-generator/

     respect to thonk, music thing, little scale, mutable, dubClub, quiet club.

*/

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // used for writing to DAC
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#include <SPI.h>
#include <TimerOne.h>
#include <ResponsiveAnalogRead.h>
#include <EEPROM.h>
#include <Bounce2.h>
#include <MIDI.h>
#include <elapsedMillis.h>

//editable
const int CHANNELS = 3; // jack outputs
const int TRIGGERS = 3; // inputs
const int OUTPUTS = 4; // includes 4th on back of PCB

const int MIN_PERIOD = 200;  // experiment with this - LED will cut out at high freqs, monitor then reduce this until stops
const int MAX_PERIOD = 30000; // 34000 works out at around 4.8 secs at TIMECALL of 300.
const int HOLD_TIME = 800; // ms button needs to be held to not be a trigger
const int LONG_HOLD_TIME = 2500; // ms, beyond this will cycle around waveforms for that channel
const int MIN_SUST = 15; // stops at top of cycle, coded so can enlong this
const int MAX_FEEDBACK = 6; // number of repeats
const int MIN_AR = 20; // minimum attack & release in ms to prevent glitches - period divided by this
const int STORE_DELAY = 15000; // ms after finished tweaking before settings stored to EEPROM - means not writing changes all the time.
const int MIN_DUTY = 5; // ms
const int DEADSPACE = 20; // either side of midpoint of pot & at end of range so easier to find pure waveforms
const int DEBOUNCE = 50; // ms
const int FLASH_DURATION = 400; // ms between toggle flash state ie one half of cycle

const int TIMECALL = 300; // how often i updates in usecs determines max freq. reduce to get higher Hz, crashes if too low. lengthen period to compensate.

const boolean DEBUG = false; // true;
const boolean EEPROM_DEBUG = false;
const boolean DEBUG_POTS = false;
const boolean DEBUG_BUTTONS = false;

// EEPROM addresses to hold current settings
const int waveStore[3] = {0, 1, 2};
const int waveShapeStore[3] = {4, 6, 8}; // double addresses, as hold int not byte
const int periodStore[3] = {10, 12, 14};
const int tweak0Store[3] = {16, 18, 20};
const int tweak1Store[3] = {22, 24, 26};
const int tweak2Store[3] = {28, 30, 32};
const int maxAmpStore[3] = {34, 36, 38};
const int trigModeStore = 40; // single byte holding 4 bits

const byte EEPROM_CHECK_KEY = 30;     // key - change this to any different number between 0 & 255 if you need to reinitialise the default settings
const byte EEPROM_CHECK_ADD = 99;      // address where eepromID should be stored, if it's not key, initialise settings

const int OCC_CHECK_INTERVAL = 1000; // try 1000 = checks 1 parameter every x ms, updates EEPROM if needed. cycles every 30 secs, spreads processor load
long testTime;

// don't change these
const int QTR_SINE = 256;
const int WHOLE = QTR_SINE * 4; // full period is 1024 (? 4096 gives better resolution - no, this is position along, wave table does give 12 bit values)
const int DAC_MAX = 4095; // 12 bit resolution for MCP4822 DAC (4096 loses important bit)
const int DAC_MIN = 409; // 1v minimum - if we let it go below this, risk of silent module which may not be obvious why
const int SINE_PHASE_FUDGE = 0; // magic number, moves starting phase to 0 preventing glitches on sine wave
const int HALFWAY = 511; // half pot position = 1022/2

// DAC out parameters if needed
const int GAIN_1 = 0x1; // 1 is no gain
const int GAIN_2 = 0x0; //
const int CH_A = 1; // inverted to match PCB
const int CH_B = 0;
const int SHUTDOWN = 1;

const int ESS = 0; // envelope, sine, square
const int RTR = 1; // ramp, triangle, ramp - trigger holds sustain level then falls
const int RPC = 2;  // random, pulse train, CV
const int MAX_WAVEFORMS = 2; //  ie 4 (0-3), beyond this, overflows back to 0

const int MIX = 10; // 4th channel

byte bonusChannelSetting = MIX;



int sineTable[] = {     // one quarter of sine wave
  2048, 2060, 2073, 2085, 2098, 2110, 2123, 2136, 2148, 2161, 2173, 2186, 2198, 2211, 2223, 2236,
  2248, 2261, 2273, 2286, 2298, 2311, 2323, 2336, 2348, 2361, 2373, 2385, 2398, 2410, 2423, 2435,
  2447, 2460, 2472, 2484, 2497, 2509, 2521, 2533, 2545, 2558, 2570, 2582, 2594, 2606, 2618, 2630,
  2642, 2654, 2666, 2678, 2690, 2702, 2714, 2726, 2738, 2750, 2762, 2773, 2785, 2797, 2808, 2820,
  2832, 2843, 2855, 2866, 2878, 2889, 2901, 2912, 2924, 2935, 2946, 2958, 2969, 2980, 2991, 3002,
  3014, 3025, 3036, 3047, 3058, 3069, 3079, 3090, 3101, 3112, 3123, 3133, 3144, 3154, 3165, 3176,
  3186, 3196, 3207, 3217, 3227, 3238, 3248, 3258, 3268, 3278, 3288, 3298, 3308, 3318, 3328, 3338,
  3347, 3357, 3367, 3376, 3386, 3395, 3405, 3414, 3424, 3433, 3442, 3451, 3460, 3470, 3479, 3487,
  3496, 3505, 3514, 3523, 3532, 3540, 3549, 3557, 3566, 3574, 3582, 3591, 3599, 3607, 3615, 3623,
  3631, 3639, 3647, 3655, 3663, 3670, 3678, 3686, 3693, 3701, 3708, 3715, 3723, 3730, 3737, 3744,
  3751, 3758, 3765, 3772, 3778, 3785, 3792, 3798, 3805, 3811, 3818, 3824, 3830, 3836, 3842, 3848,
  3854, 3860, 3866, 3872, 3877, 3883, 3889, 3894, 3899, 3905, 3910, 3915, 3920, 3925, 3930, 3935,
  3940, 3945, 3949, 3954, 3959, 3963, 3968, 3972, 3976, 3980, 3984, 3988, 3992, 3996, 4000, 4004,
  4008, 4011, 4015, 4018, 4022, 4025, 4028, 4031, 4034, 4037, 4040, 4043, 4046, 4049, 4051, 4054,
  4056, 4059, 4061, 4063, 4065, 4067, 4069, 4071, 4073, 4075, 4077, 4078, 4080, 4081, 4083, 4084,
  4085, 4087, 4088, 4089, 4090, 4091, 4091, 4092, 4093, 4093, 4094, 4094, 4094, 4095, 4095, 4095,
  4095
};


// pins
const int greenPin[] = {3, 6, 5};
const int redPin[] = {4, 7, A3}; // check this // wired reversed in prototype, should be red4 A2, green4 9
const int gateIn[] = {A2, A1, A0};
const int potPin[] = {A5, A6, A7}; //
const int buttonPin[] = {2, 8, A4};

const byte DAC0_CS = 10; // only used in setup, direct port manipulation used in main loop instead
const byte DAC1_CS = 9;  //
const byte MIDI_IN_PIN = 0; const byte MIDI_OUT_PIN = 1; // note also used in Serial communication

Bounce button0 = Bounce();
Bounce button1 = Bounce();
Bounce button2 = Bounce();

ResponsiveAnalogRead analog0(potPin[0], true); // sleepEnabled or not, easing. more twichty with true, smoother with disabled
ResponsiveAnalogRead analog1(potPin[1], true);
ResponsiveAnalogRead analog2(potPin[2], true);

elapsedMillis timeSinceStore; // holds time since last check for changes and EEPROM write if so
elapsedMillis flashTime[CHANNELS]; // hold time for flashing LED


// central vars
volatile unsigned int i = 0; // clock, updates on timer
byte takeTurns = 0; // reads each pot in order
long occCheckTime = 0;
int currentChannel = 9; // off
boolean anyPotMoved = false;
boolean storeFlag = false; // if set, store EEPROM
byte param = 0; // current setting to store in EEPROM, start at 0 as it updates first
byte hiByte, loByte, retrigs; // retrig stores all channels in same byte, use bitSet


// LEDs
volatile byte channelRedLED_on = 0; // if on by button pressed - anded for final write so gate doesn't turn off when button pressed, holds LED status on bits 0-3,
boolean editModeLEDs = false; // turns on LEDs when long press
byte channelLEDpressed = 9;
boolean flashOn[CHANNELS] = {1, 1, 1}; // start with flash on


// buttons
byte buttonPressed[CHANNELS] = {false, false, false};
byte firstButton = 10; // holds first button pushed (to allow multiple combinations), 10 being off
boolean newPress = {false}; // true from start of button until button processed, or turned off
boolean buttonHeld = {false};
long buttonStartTime;


// channel vars
volatile unsigned int potValue[CHANNELS];
volatile unsigned int value[OUTPUTS]; // = {1000, 2512, 2512, 2512}; // output value (12 bit 0-4096) use volatile if timer may change value
unsigned int lastPotValue[CHANNELS];
unsigned int lastPlace[CHANNELS]; // hold where left off waveform in envelope ?? used

int tweak0[CHANNELS]; // = {512, 512, 512}; // duty cycle / phase / EG sustain level
int tweak1[CHANNELS]; // = {512, 512, 512}; // {0, 0, 0, 0}; // 10 bit (0-1024) used for attack, or fine tuning LFO, portamento for noise, duty cycle for pulse,
int tweak2[CHANNELS]; // = {512, 512, 512}; // {1024, 1024, 1024, 1024}; // 10 bit (0-1024), release, or max amplitude for LFOs, (or randomness)
// int sustainLevel[CHANNELS] = {512, 512, 512};
int dutyCycle[CHANNELS];
int maxAmp[CHANNELS]; // = {DAC_MAX, DAC_MAX, DAC_MAX};
volatile unsigned int attackTime[OUTPUTS], releaseTime[OUTPUTS], sustainTime[OUTPUTS];
unsigned int randVal[CHANNELS];
int duty[OUTPUTS]; // calculated from tweak1 anyway
int repeats[CHANNELS]; // number of pulse repeats, up to MAX_REPEATS


boolean retrigMode[] = {true, true, true, false}; // starting mode
byte waveForm[OUTPUTS]; // holds which of 3 main waveforms is used (set by pot) /
int waveShape[OUTPUTS] = {0, 0, 0, 0}; // initial pot value, sets which waveShape is used
int sineValue, pulseValue, triValue, rampValue, envValue ;

int storedWaveForm[OUTPUTS], storedWaveShape[OUTPUTS], storedPeriod[OUTPUTS];
int storedTweak0[OUTPUTS], storedTweak1[OUTPUTS], storedTweak2[OUTPUTS];
int storedMaxAmp[OUTPUTS];

boolean tweak0Moved[CHANNELS] = {true, true, true};
boolean tweak1Moved[CHANNELS] = {true, true, true};
boolean tweak2Moved[CHANNELS] = {true, true, true};

int phase[CHANNELS] = {512, 512, 512};
// boolean pulseState[CHANNELS] = {1, 1, 1}; // start with pulse on

volatile unsigned long place[CHANNELS] = {0, 0, 0}; // place along cycle
volatile unsigned long period[OUTPUTS]; // cycle length

boolean gate[CHANNELS];
boolean oldGate[CHANNELS] = {false, false, false};
volatile boolean trigger[CHANNELS] = {true, true, true}; // 4 channels, 3rd duplicated into 4th at the time



// ********************  setup ********************************

void setup() {
  Serial.begin(9600);
  SPI.begin();
  Serial.println("Hi. Welcome to wobble, a module by fuzzysynths.co.uk");
  Serial.println("This is version 0.8 of software. Early version, demo for Machina Bristonica 2019");
  Serial.println("Check for software updates at www.fuzzysynths.co.uk/wobble");
  Serial.println("MIDI support still to add. Output 4 set to mix of other 3");
  Serial.println("***********");
  Serial.println("");

  Timer1.initialize(TIMECALL); // updates every x usecs - reduce to get higher Hz
  Timer1.attachInterrupt(updateClock); //

  pinMode(greenPin[0], OUTPUT); pinMode(greenPin[1], OUTPUT); pinMode(greenPin[2], OUTPUT);
  pinMode(redPin[0], OUTPUT); pinMode(redPin[1], OUTPUT); pinMode(redPin[2], OUTPUT);
  pinMode(buttonPin[0], INPUT); // don't ask me why input pullup doesn't work on this one, just write high later. might be just the test board.
  pinMode(buttonPin[1], INPUT_PULLUP); pinMode(buttonPin[2], INPUT_PULLUP);

  pinMode(DAC0_CS, OUTPUT); pinMode(DAC1_CS, OUTPUT);
  pinMode(gateIn[0], INPUT); pinMode(gateIn[1], INPUT); pinMode(gateIn[2], INPUT);
  pinMode(potPin[0], INPUT); pinMode(potPin[1], INPUT); pinMode(potPin[2], INPUT);

  digitalWrite(buttonPin[0], HIGH);

  button0.attach(buttonPin[0], INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  button0.interval(DEBOUNCE); // Use a debounce interval of DEBOUNCE (~50) milliseconds
  button1.attach(buttonPin[1], INPUT_PULLUP);
  button1.interval(DEBOUNCE);
  button2.attach(buttonPin[2], INPUT_PULLUP);
  button2.interval(DEBOUNCE);


  //  DDRB = B11101110; // pins 13-8,  0 is input, 1 is out
  //  DDRC = B00010000; // pins A7-0
  //  DDRD = B11111010; // ports pins 7-0

  digitalWrite(DAC0_CS, HIGH); digitalWrite(DAC1_CS, HIGH);

  checkEEPROM();

  testTime = micros();    // for testing cycle time

}


// ********************  loop  start ********************************
void loop() {

  // test loop time - microseconds between each cycle
  //  int lag = micros() - testTime;
  //    Serial.println(lag);
  //  testTime = micros();
  // 560 best yet
  // now 1056 - 1500

  // update timer
  // more complicated than updating each time, but allows interpolation esp. when cycles might be slower (housekeeping)
  if (i) {               // if timer has updated
    noInterrupts();      // read v quick
    unsigned int incr = i;        // how many have there been since last checked?
    interrupts();

    place[0] += incr;    // update place along wave
    place[1] += incr;
    place[2] += incr;
    place[3] += incr;
    i = 0;
  }

  // read interface one pot each time around to save processing time for waveform generation
  readPots(takeTurns);
  takeTurns ++;
  if (takeTurns >= CHANNELS) {
    takeTurns = 0;
  }

  checkButtons();
  checkGates();
  updateLEDs();
  houseKeeping();
  // deepOptions(); // system settings, not yet coded

  updateOutputs(); // update values

  // write values to DAC // ** really don't touch this ***
  setOutput(1, CH_A, 0, value[3]); // chip, channel, gain (0 is 2x), value YES
  setOutput(1, CH_B, 0, value[1]); // just the way it's wired
  setOutput(0, CH_A, 0, value[0]); // setOutput(1, CH_A, 0, value[3]); goes to output 4
  setOutput(0, CH_B, 0, value[2]); //

}

// ********************  loop  end ********************************



// ********************* functions **************************

void checkButtons() { // sets firstButton (if multiple) and buttonPressed[] true or false
  button0.update(); // Update the Bounce instance
  button1.update();
  button2.update();

  buttonPressed[0] = !button0.read(); // inverts, so pressed is true when it is pressed (library does true voltage - input pullup so goes low when pressed)
  buttonPressed[1] = !button1.read();
  buttonPressed[2] = !button2.read();

  if (buttonHeld == false) { // (millis() > buttonStartTime + 10) &&                                                          // newly pressed
    if (buttonPressed[0] == true) {     //    if (!digitalRead(buttonPin[0])) {           // read inverted
      firstButton = 0;
      buttonHeld = true;
      buttonStartTime = millis();
      //      Serial.print(" pressed on channel :");  Serial.println(firstButton);// first button pressed
    } else if (buttonPressed[1] == true) {
      firstButton = 1;
      buttonHeld = true;
      buttonStartTime = millis();
      //      Serial.print(" pressed on channel :"); Serial.println(firstButton);// first button pressed
    } else if (buttonPressed[2] == true) {
      firstButton = 2;
      buttonHeld = true;
      buttonStartTime = millis();
      //      Serial.print(" pressed on channel :"); Serial.println(firstButton);// first button pressed
    }
  }

  if (buttonHeld) {
    if ((millis() > buttonStartTime + HOLD_TIME) && (millis() < buttonStartTime + LONG_HOLD_TIME)) {                               // button held, not too long yet

      channelLEDpressed = firstButton;  // turns on this LED

    } else if (millis() > buttonStartTime + LONG_HOLD_TIME) {                                                                      // button held long time

      if ((currentChannel > 8) && (!anyPotMoved)) {
        editModeLEDs = true;              // turns on all red LEDs
      } else {
        editModeLEDs = false;
      }
      if (anyPotMoved) {
        editModeLEDs = false;
      }
    }
  }

  if ((buttonHeld) && (buttonPressed[0] == false) && (buttonPressed[1] == false) && (buttonPressed[2] == false)) {           // newly released
    channelLEDpressed = 9;
    if (millis() < (buttonStartTime + DEBOUNCE)) {                // just a bounce
      buttonHeld = false;
      firstButton = 9;
      // anyPotMoved = false;

    } else if ( (millis() >= (buttonStartTime + DEBOUNCE)) && (millis() < (buttonStartTime + HOLD_TIME))) {                       // after short hold
      //      Serial.print(" released on channel :"); Serial.println(firstButton);// first button pressed
      trigger[firstButton] = true;
      buttonHeld = false;
      firstButton = 9;
      anyPotMoved = false;

    } else if ((millis() >= (buttonStartTime + HOLD_TIME)) && (millis() < (buttonStartTime + LONG_HOLD_TIME))) {

      if (!anyPotMoved) {                                                                                                          // after medium hold
        retrigMode[firstButton] = !retrigMode[firstButton];                                                                                // released after medium hold - toggles retrig mode

        if (currentChannel < 8) {   // cancel active channel
          currentChannel = 9;
        }
      }

      //      Serial.print(anyPotMoved); Serial.print(" <- anyPotMoved, finished medium hold on channel :"); Serial.print(firstButton); Serial.print(", retrig Mode is :"); Serial.print(retrigMode[firstButton]); Serial.print(" on channel "); Serial.println(firstButton);
      buttonHeld = false;
      firstButton = 9;
      anyPotMoved = false;


    } else if (millis() >= buttonStartTime + LONG_HOLD_TIME) {                                                                  // released after very long hold
      if (!anyPotMoved) {
        if (currentChannel > 8) {
          currentChannel = firstButton;
          //          editModeLEDs = true;
          //         Serial.print("released on "); Serial.print(currentChannel); Serial.print(" edit mode is "); Serial.println(editModeLEDs);
        } else {
          currentChannel = 9;
          //          editModeLEDs = false;
          //        Serial.print("released on "); Serial.print(currentChannel); Serial.print(" edit mode is "); Serial.println(editModeLEDs);
        }
      } else {                                                                                                                // to turn off edit mode if any pot moved (changing waveform)
        currentChannel = 9;
        //       editModeLEDs = false;
        if (DEBUG_BUTTONS) {
          Serial.print(" edit mode is :"); Serial.print(editModeLEDs); Serial.print(", current channel now:"); Serial.println(currentChannel);
        }
      }
      anyPotMoved = false;
      if (DEBUG_BUTTONS) {
        Serial.print(" edit mode is :"); Serial.print(editModeLEDs); Serial.print(", channelLEDpressed is :"); Serial.print(channelLEDpressed); Serial.print(", current channel now:"); Serial.print(currentChannel);
        Serial.print(" very long hold on channel :"); Serial.print(firstButton); Serial.print(" current channel now:"); Serial.println(currentChannel);
      }
    }
    buttonHeld = false;
    firstButton = 9;
  }

  // if nothing pressed for ages, current channel turns off? - no, too complicated?

  if ((buttonPressed[0] == false) && (buttonPressed[1] == false) && (buttonPressed[2] == false)) {                              // still nothing pressed
    buttonHeld = false;
    buttonStartTime = millis();
    //   anyPotMoved = false;
    firstButton = 9;
  }
}

void readPots(byte pot) { // reads one pot at a time
  long temp; // MUST be a long
  switch (pot) {
    case 0:
      analog0.update();
      potValue[0] = analog0.getValue(); //     temp = analogRead(potPin[0]);
      break;
    case 1:
      analog1.update();
      potValue[1] = analog1.getValue();  // temp = analogRead(potPin[0]);
      break;
    case 2:
      analog2.update();
      potValue[2] = analog2.getValue();  // temp = analogRead(potPin[2]);
      break;
  }
  //  potValue[pot] = temp;

  // potValue[pot] = sqrt(temp * 1023) changes it to positive exponential curve, so pot changes lower at top end of readings  sqrt(potValue[pot] * 1023) which is it?  sqrt(potValue[pot]) * 1023

  if ((buttonHeld == false) && (currentChannel > 8)) {                                                                                      // no button, not in edit mode, so change cycle length (period)

    if (lastPotValue[pot] != potValue[pot]) {                                                                                             // pot has changed, do something
      lastPotValue[pot] = potValue[pot];
      anyPotMoved = true;

      period[pot] = map(potValue[pot], 0, 1022, MAX_PERIOD, MIN_PERIOD); // responsive analog read out of 1022, not 1023!        //    period[pot] = map(sqrt(temp * 1023), 0, 1022, MAX_PERIOD, MIN_PERIOD); // responsive analog read out of 1022, not 1023!
      if (DEBUG_POTS) {
        Serial.print("Moved pot "); Serial.print(pot);  Serial.print(", pot value is "); Serial.print(potValue[pot]); Serial.print(", period now: "); Serial.println(period[pot]);
      }
      // period is exponential curve: sqrt(temp * 1023), rest of pot mvts linear
      //  anyPotMoved = true; // used so button doesn't change retrigMode, if held to edit parameters rather than retrigMode
    }
  }

  if ((buttonHeld) && (lastPotValue[pot] != potValue[pot])) {                                                                                             // pot has changed, do something
    lastPotValue[pot] = potValue[pot];
    waveShape[firstButton] = potValue[pot];
    waveForm[firstButton] = pot;
    if (DEBUG_POTS) {
      Serial.print(firstButton); Serial.print(" channel, waveform is "); Serial.print(waveForm[firstButton]); Serial.print(" wave select is "); Serial.println(waveShape[firstButton]);
    }
  }
  if ((!buttonHeld) && (currentChannel < 8) && (lastPotValue[pot] != potValue[pot])) {                                                                                           // if current edit channel, nothing held
    lastPotValue[pot] = potValue[pot];
    if (pot == 0) {
      tweak0[currentChannel] = analog0.getValue();                                                                                               // change tweak value
      tweak0Moved[currentChannel] = true;
      if (DEBUG_POTS) {
        Serial.print("Moved pot "); Serial.print(pot);  Serial.print(", pot value is "); Serial.print(potValue[pot]); Serial.print(", tweak 0 is : "); Serial.println(tweak0[currentChannel]);
      }
    } else if (pot == 1) {
      tweak1[currentChannel] = analog1.getValue();
      tweak1Moved[currentChannel] = true;
      if (DEBUG_POTS) {
        Serial.print("Moved pot "); Serial.print(pot);  Serial.print(", pot value is "); Serial.print(potValue[pot]); Serial.print(", tweak 1 is : "); Serial.println(tweak1[currentChannel]);
      }
    } else if (pot == 2) {
      tweak2[currentChannel] = analog2.getValue();
      tweak2Moved[currentChannel] = true;
      if (DEBUG_POTS) {
        Serial.print("Moved pot "); Serial.print(pot);  Serial.print(", pot value is "); Serial.print(potValue[pot]); Serial.print(", tweak 2 is : "); Serial.println(tweak2[currentChannel]);
      }
    }

  } // end of pot changed


}


void updateLEDs() {
  for (int i = 0; i < CHANNELS; i ++) {

    if (retrigMode[i]) {                                                             // first choose colour for retrig mode
      bitClear(channelRedLED_on, i);
    } else {
      bitSet(channelRedLED_on, i);
    }

    if (gate[i]) {                                                                      //  inverts colour with gate
      if (bitRead(channelRedLED_on, i)) {
        bitClear(channelRedLED_on, i);
      } else {
        bitSet(channelRedLED_on, i);
      }
    }

    if (channelLEDpressed == i) {                                                     // turn red on after button held
      if (bitRead(channelRedLED_on, i)) {                                                             // first choose colour for retrig mode
        bitClear(channelRedLED_on, i);
      } else {
        bitSet(channelRedLED_on, i);
      }
      //           Serial.println(channelLEDpressed );
    }

    if (editModeLEDs) {                            // edit mode, any channel
      bitSet(channelRedLED_on, i);
      //      Serial.print(channelRedLED_on, BIN); Serial.println(" <- channelREDLed_on, we're in editMode ");
    }

    if (currentChannel == i) {                                                            // channel edit mode overwrites all this
      if (flashTime[i] >= FLASH_DURATION) {                                                       // update flash time
        flashOn[i] = !flashOn[i];
        flashTime[i] = 0;
        //        Serial.print("flash value is "); Serial.print(flashOn[i]);  Serial.print(" on channel "); Serial.println(i);
      }
      //     analogWrite(greenPin[i], 255);
      if (flashOn[i]) {
        bitSet(channelRedLED_on, i);
      } else if (!flashOn[i]) {
        bitClear(channelRedLED_on, i);
      }
      //     Serial.println(channelRedLED_on, BIN);
    }

    if ((bitRead(channelRedLED_on, i))) {                                                 // set the actual pins
      turnRedOn(i); //       digitalWrite(redPin[i], HIGH);

    } else if (!bitRead(channelRedLED_on, i)) {
      turnRedOff(i); //     digitalWrite(redPin[i], LOW);
    }
    if (currentChannel > 8) {     // not in edit mode
      analogWrite(greenPin[i], 255 - (value[i] / 16) ); // /16 brings down to 8 bit resolution for PWM, inverted
    } else {
      analogWrite(greenPin[i], 255);
    }
  } // end each channel
}

void turnRedOn(byte i) { // inverted due to PCB
  if (i == 0) {
    // easier in binary - goes backwards from right though? ie pins 7-0
    PORTD = PORTD & 0b11101111; // D4  high, rest unchanged, PORTD on the Uno.
  } else if (i == 1) {
    PORTD = PORTD & 0b01111111; // turn off PD7
  } else if (i == 2) {
    PORTC = PORTC & 0b11110111; // turn off PC3
  }
}


void turnRedOff(byte i) { // inverted due to PCB
  if (i == 0) {
    PORTD = PORTD | 0b00010000;   // turn on PD4
  } else if (i == 1) {
    PORTD = PORTD | 0b10000000; // turn on PD7,
  } else if (i == 2) {
    PORTC = PORTC | 0b00001000;   // turn on PC3

  }
}


void updateOutputs() {
  for (int ch = 0; ch < CHANNELS; ch ++) {

    if (retrigMode[ch]) {
      if (place[ch] >= period[ch]) { // got to end of cycle
        if (ch == 0) {
          //         Serial.print(retrigMode[ch]); Serial.print("retrig happened at  "); Serial.println(place[ch]);
        }
        place[ch] = 0;
        trigger[ch] = true;
      }
    }

    maxAmp[ch] = int(map(tweak1[ch], 0 , 1022, DAC_MIN, DAC_MAX));

    // phase needs to update each cycle

    if (tweak0Moved[currentChannel]) {  // only updates this parameter when actively moved, so inherits previous setting if different meaning
      phase[ch] = int(map(tweak0[ch], 0 , 1022, 0, DAC_MAX));
      tweak1Moved[currentChannel] = false;
    }

    if (tweak1Moved[currentChannel]) {  // only updates this parameter when actively moved, so inherits previous setting if different meaning
      //     maxAmp[ch] = int(map(tweak1[ch], 0 , 1022, DAC_MIN, DAC_MAX));
      tweak1Moved[currentChannel] = false;
    }



    switch (waveForm[ch]) {

      case ESS:                                               // envelope - sine - square, always LFO mode
        if (trigger[ch] == true) {                            // reset done by trigger in
          period[ch] = map(lastPotValue[ch], 0, 1022, MAX_PERIOD, MIN_PERIOD); //  recalculated at start of envelope. last pot value holds any changed, might be wrong
          attackTime[ch] =  int(map(tweak0[ch], 0, 1022, MIN_AR, int(period[ch] / 4)));
          sustainTime[ch] = int(map(tweak0[ch], 0, 1022, MIN_SUST, int(period[ch] / 4)));
          maxAmp[ch] = int(map(tweak1[ch], 0 , 1022, DAC_MIN, DAC_MAX));
          releaseTime[ch] = int(map(tweak2[ch], 0, 1022, (period[ch] / 4), MIN_AR));
          place[ch] = 0;
          trigger[ch] = false;
        }
        //        Serial.print("period: "); Serial.print(period[ch]); Serial.print(", attack: "); Serial.print(attackTime[ch]);  Serial.print(", release: "); Serial.println(releaseTime[ch]);

        if (place[ch] < attackTime[ch]) {                                                                             // attack phase
          envValue = int(map(place[ch], 0, attackTime[ch], 0, maxAmp[ch]));
        } else if ((place[ch] >= attackTime[ch]) && (place[ch] <= (attackTime[ch] + sustainTime[ch]))) {              // sustain phase
          envValue = maxAmp[ch];
          //         lastPlace[ch] = place[ch];
        } else if ((place[ch] > (attackTime[ch] + sustainTime[ch])) && (place[ch] <= (attackTime[ch] + sustainTime[ch] + releaseTime[ch]))) {   // release phase
          envValue = int(map(place[ch], (attackTime[ch] + sustainTime[ch]), (attackTime[ch] + sustainTime[ch] + releaseTime[ch]), maxAmp[ch], 0));
        } else if ((place[ch] > (attackTime[ch] + sustainTime[ch] + releaseTime[ch]))) { // ends at end of release time
          envValue = 0;
        }

        // sine
        sineValue = int(sine(map(place[ch], 0, period[ch], 0, maxAmp[ch] / 4), phase[ch]));

        // square
        duty[ch] = MIN_DUTY + (period[ch] * tweak0[ch] / 1200);
        if (place[ch] < duty[ch]) { // on for first half of waveform
          // pulseState[ch] = 1;
          pulseValue = maxAmp[ch];
        } else {
          //  pulseState[ch] = 0;
          pulseValue = 0;
        }
        //     if (pulseState[ch]) {
        //          pulseValue = maxAmp[ch];
        //     } else {
        //          pulseValue = 0;
        //     }
        if (waveShape[ch] < DEADSPACE) { // turned to left
          value[ch] = envValue;
        } else if ((waveShape[ch] >= DEADSPACE) && (waveShape[ch] < (HALFWAY - DEADSPACE))) {
          value[ch] = int(map(waveShape[ch], DEADSPACE, (HALFWAY - DEADSPACE), envValue, sineValue)); // interpolate tri to sine
        } else if ((waveShape[ch] >= (HALFWAY - DEADSPACE)) && (waveShape[ch] < (HALFWAY + DEADSPACE))) { // near midpoint
          value[ch] = sineValue;
        } else if ((waveShape[ch] >= (HALFWAY + DEADSPACE)) && (waveShape[ch] < (1022 - DEADSPACE))) {
          value[ch] = int(map(waveShape[ch], (HALFWAY + DEADSPACE), 1022, sineValue, pulseValue)); // interpolate sine to pulse
        } else if (waveShape[ch] >= (1022 - DEADSPACE)) {
          value[ch] = pulseValue;
        }

        break;

      case RTR: // ramp down - triangle - ramp up
        if (trigger[ch] == true) {            // reset done by trigger in
          // should change with phase
          trigger[ch] = false;
          place[ch] = 0;
        }

        if (place[ch] < (period[ch] / 2) ) { // first half of waveform                                    //  triangle
          triValue = int(map(place[ch], 0, period[ch] / 2, 0, maxAmp[ch]));
        } else {
          triValue = int(map(place[ch], period[ch] / 2, period[ch], maxAmp[ch], 0));
        }

        rampValue = int(map(place[ch], 0, period[ch], maxAmp[ch], 0));                                    // ramp

        if (waveShape[ch] < DEADSPACE) {                                                                  // all the way left
          value[ch] = rampValue;                                                                            // ramp down
        } else if ((waveShape[ch] >= DEADSPACE) && (waveShape[ch] < (HALFWAY - DEADSPACE))) {             // turned to left
          value[ch] = int(map(waveShape[ch], DEADSPACE, (HALFWAY - DEADSPACE), rampValue, triValue));       // interpolate ramp down to tri
        } else if ((waveShape[ch] >= (HALFWAY - DEADSPACE)) && (waveShape[ch] < (HALFWAY + DEADSPACE))) { // near midpoint
          value[ch] = triValue;                                                                             // stick with triangle
        } else if ((waveShape[ch] >= (HALFWAY + DEADSPACE)) && (waveShape[ch] < (1022 - DEADSPACE))) {    // turned to right
          value[ch] = int(map(waveShape[ch], (HALFWAY + DEADSPACE), 1022, triValue, (maxAmp[ch] - rampValue))); // interpolate tri to ramp up
        } else if (waveShape[ch] >= (1022 - DEADSPACE)) {                                                 // all the way right
          value[ch] = maxAmp[ch] - rampValue;                                                               // ramp up
        }
        break;


      case RPC: // random, pulse train, direct CV

        period[ch] = map(lastPotValue[ch], 0, 1023, MAX_PERIOD, MIN_PERIOD); //  recalculated constantly

        if (tweak1Moved[currentChannel]) {  // only updates this parameter when actively moved, so inherits previous setting if different meaning
          tweak1Moved[currentChannel] = false;
          // not used
        }

        if (trigger[ch] == true) {                                                                         // new cycle
          trigger[ch] = false; // reset done by trigger in
          if (ch == 0) {
            //            Serial.print("trigger at place: "); Serial.println(place[ch]);
          }
          place[ch] = 0;
          randVal[ch] = random(maxAmp[ch]);
          int feedback = int(map(waveShape[ch], DEADSPACE, (1022 - DEADSPACE), 1, MAX_FEEDBACK)); // sets repeats from waveshape[]
          repeats[ch] = feedback;
          int triplets = int(period[ch] / MAX_FEEDBACK);
          attackTime[ch] =  int(map(tweak0[ch], 0, 1022, MIN_AR, int(triplets / 3)));
          //      sustainTime[ch] = int(map(tweak0[ch], 0, 1022, MIN_SUST, int(triplets / 3)));
          sustainTime[ch] = MIN_SUST;
          releaseTime[ch] = int(map(tweak2[ch], 0, 1022, MIN_AR, int(triplets / 3)));
          maxAmp[ch] = int(map(tweak1[ch], 0 , 1022, DAC_MIN, DAC_MAX));
          // should change with phase

        }

        // triplets of the period, can go from triangle to envelope, if repeats >3 ie overlaps goes into next period ie period doubles, repeats up to 3 periods would be about right
        envValue = 0;

        if (repeats[ch] >= 1) {
          if (place[ch] <= attackTime[ch]) {                                                                             // attack phase
            envValue = int(map(place[ch], 0, attackTime[ch], 0, maxAmp[ch]));
          } else if ((place[ch] > attackTime[ch]) && (place[ch] <= (attackTime[ch] + sustainTime[ch]))) {              // sustain phase
            envValue = maxAmp[ch];
          } else if ((place[ch] > (attackTime[ch] + sustainTime[ch])) && (place[ch] <= (attackTime[ch] + sustainTime[ch] + releaseTime[ch]))) {   // release phase
            envValue = int(map(place[ch], (attackTime[ch] + sustainTime[ch]), (attackTime[ch] + sustainTime[ch] + releaseTime[ch]), maxAmp[ch], 0));
          } else if ((place[ch] > (attackTime[ch] + sustainTime[ch] + releaseTime[ch]))) { // ends at end of release time
            envValue = 0;
          }

          if (place[ch] > int(period[ch] / MAX_FEEDBACK)) { // overruns
            //       Serial.print("place "); Serial.print(place[ch]); Serial.print(", end "); Serial.println((attackTime[ch] + sustainTime[ch] + releaseTime[ch]));
            envValue = 0;
            place[ch] = 0;
            repeats[ch] --; // one fewer repeat to do
          }
        }

        if (waveShape[ch] < DEADSPACE) {                                                                  // all the way left
          value[ch] = randVal[ch];                                                                            // ramp down
        } else if ((waveShape[ch] >= DEADSPACE) && (waveShape[ch] < (HALFWAY - DEADSPACE))) {             // turned to left
          value[ch] = envValue;
        } else if ((waveShape[ch] >= (HALFWAY - DEADSPACE)) && (waveShape[ch] < (HALFWAY + DEADSPACE))) { // near midpoint
          value[ch] = envValue;                                                                              // stick with triangle
        } else if ((waveShape[ch] >= (HALFWAY + DEADSPACE)) && (waveShape[ch] < (1022 - DEADSPACE))) {    // turned to right
          value[ch] = envValue;
        } else if (waveShape[ch] >= (1022 - DEADSPACE)) {                                                 // all the way right
          value[ch] = map(potValue[ch], 0, 1022, 0, DAC_MAX);                                                               // ramp up
        }
        break;
    }

  } // go on to next channel
  if (bonusChannelSetting == MIX) {   // set 4th channel output
    value[3] = int((value[0] + value[1] + value[2]) / 3);
  }
} // end update outputs



int sine(int i, int phase) {    // sine wave look up table dose middle to top, ie top left bit of cirlce
  int place = i - phase + SINE_PHASE_FUDGE; // magic number, to get sine to start at 0 with duty, (tweak0) at half pot, 512
  while (place < 0) {
    place += 1024;
  }
  if (place <= 255) {
    return sineTable[place];
  } else if ((place <= 511) && (place >= 256)) {
    return sineTable[512 - place];
  } else if ((place <= 767) && (place >= 512)) {
    return (4095 - sineTable[place - 512]);
  } else if (place >= 768) {
    return (4095 - sineTable[1024 - place]);
  }
}


void setOutput(byte chip, byte channel, byte gain, unsigned int val) // bit 12 shutdown 1 is active, 0 is no output
{ // you're brave to mess with this bit
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | channel << 7 | gain << 5 | SHUTDOWN << 4; // 7 is bit 15, 5 bit 13. gain 0 is 2x

  if (chip == 0) { // upper IC, outputs 1 & 2
    PORTB &= 0x11111011; //  PORTB &= 0xfb;   // 0xfb = 0x1111 1011   should be equiv to digitalWrite(DAC0_CS, LOW);
    //    PORTD &= B00000011; // turns off 2..7, but leaves pins 0 and 1 alone
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
    PORTB = PORTB | 0b00000100;
    //   PORTB |= 0x0000100; // 0x4;   //  equiv to digitalWrite(DAC0_CS, HIGH);
  } else if (chip == 1) { // the lower IC, outputs 3 & 4
    PORTB &= 0x11111101; // PORTB &= 0xfe; // sets PB1 low,  // equiv to digitalWrite(DAC1_CS, LOW);
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
    PORTB = PORTB | 0b00000010;
    // PORTB |= 0x00000010;  // 0x1; // sets PB1 high, equiv to // digitalWrite(DAC1_CS, HIGH);
  }
}

void updateClock() { // called by interrupt, so timing moves on whatever else is happening
  i ++;
}

void checkGates() {
  for (int i = 0; i < TRIGGERS; i ++) {
    gate[i] = !(digitalRead(gateIn[i])); // inverts gate in as hardware so now gate is true when signal received
    if (gate[i] == HIGH) {
      if (oldGate[i] == false) { // from now, gate is 1 when signal present at jack
        trigger[i] = true; // reset after processing
        oldGate[i] = true; // don't trigger again
        if (trigger[2]) {
          trigger[3] = true; // ch 3 inherits trigger from channel 2
        }
      }
    } else if (gate[i] == LOW) {
      oldGate[i] = false;
    }
  }
}



void deepOptions() { // to edit deep options from fronta panel ? by pressing multiple button combo? eg change MIDI channel?
  // nothing to see here yet
  // system reset - change EEPROM_CHECK_KEY so it reloads defaults
}

void houseKeeping() {                   // scrolls through parameters & checks if updated, if so writes to EEPROM so saves settings for next time.
  // write cycle completed every 25 secs at default rate. change OCC_CHECK_INTERVAL if need it faster

  if (timeSinceStore >= OCC_CHECK_INTERVAL) {        // only do occasionally
    //  if ((storeFlag) && (millis() > timeSinceEdit)) {
    byte ch = param % 3;
    //   Serial.print(param); Serial.print(" <- param, ch is: "); Serial.println(ch);

    if (param < 3) { // wave forms
      if (storedWaveForm[ch] != waveForm[ch]) {
        EEPROM.write(waveStore[ch], waveForm[ch]);
        storedWaveForm[ch] = waveForm[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing waveform ");
          Serial.print(storedWaveForm[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if ((param >= 3) && (param < 6)) {
      if (storedWaveShape[ch] != waveShape[ch]) {
        hiByte = highByte(waveShape[ch]);
        loByte = lowByte(waveShape[ch]);
        EEPROM.write(waveShapeStore[ch], hiByte);
        EEPROM.write(waveShapeStore[ch] + 1, loByte);
        storedWaveShape[ch] = waveShape[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing waveShape ");
          Serial.print(storedWaveShape[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if ((param >= 6) && (param < 9)) {
      if (storedPeriod[ch] != period[ch]) {
        hiByte = highByte(period[ch]);
        loByte = lowByte(period[ch]);
        EEPROM.write(periodStore[ch], hiByte);
        EEPROM.write(periodStore[ch] + 1, loByte);
        storedPeriod[ch] = period[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing period ");
          Serial.print(period[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if ((param >= 9) && (param < 12)) {
      if (storedTweak0[ch] != tweak0[ch]) {
        hiByte = highByte(tweak0[ch]);
        loByte = lowByte(tweak0[ch]);
        EEPROM.write(tweak0Store[i], hiByte);
        EEPROM.write(tweak0Store[i] + 1, loByte);
        storedTweak0[ch] = tweak0[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing tweak 0: ");
          Serial.print(storedTweak0[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if ((param >= 12) && (param < 15)) {
      if (storedTweak1[ch] != tweak1[ch]) {
        hiByte = highByte(tweak1[ch]);
        loByte = lowByte(tweak1[ch]);
        EEPROM.write(tweak1Store[ch], hiByte);
        EEPROM.write(tweak1Store[ch] + 1, loByte);
        storedTweak1[ch] = tweak1[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing tweak 1: ");
          Serial.print(storedTweak1[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if ((param >= 15) && (param < 18)) {
      if (storedTweak2[ch] != tweak2[ch]) {
        hiByte = highByte(tweak2[ch]);
        loByte = lowByte(tweak2[ch]);
        EEPROM.write(tweak2Store[ch], hiByte);
        EEPROM.write(tweak2Store[ch] + 1, loByte);
        storedTweak2[ch] = tweak2[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing tweak 2: ");
          Serial.print(storedTweak2[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if ((param >= 18) && (param < 21)) {
      if (storedMaxAmp[ch] != maxAmp[ch]) {
        hiByte = highByte(maxAmp[ch]);
        loByte = lowByte(maxAmp[ch]);
        EEPROM.write(maxAmpStore[ch], hiByte);
        EEPROM.write(maxAmpStore[ch] + 1, loByte);
        storedMaxAmp[ch] = maxAmp[ch];
        if (EEPROM_DEBUG) {
          Serial.print("storing max amp: ");
          Serial.print(maxAmp[ch]);
          Serial.print(" on channel ");
          Serial.println(ch);
        }
      }
    } else if (param == 21) {
      if (bitRead(retrigs, ch) != retrigMode[ch]) {   // retrig mode
        if (retrigMode[ch] == true) {
          bitSet(retrigs, ch);
        } else if (retrigMode[ch] == false) {
          bitClear(retrigs, ch);
        }
        EEPROM.write(trigModeStore, retrigs);
        if (EEPROM_DEBUG) {
          Serial.print("storing trig mode byte: ");
          Serial.println(retrigs);
        }
      }
    }
    timeSinceStore = 0;
    param ++;             // update which channel housekeeping is storing
    if (param > 21) {
      param = 0;
      //     storeFlag = false; // finished
    }
  }
}

void checkEEPROM() {

  byte id = EEPROM.read(EEPROM_CHECK_ADD);
  //  Serial.print("check ID is : ");
  //  Serial.println(id);
  if (id != EEPROM_CHECK_KEY) {  // first time around so initialise variables
    Serial.println("initiated, writing default values "); // change EEPROM_CHECK_KEY to another number if you want to initialise
    hiByte = highByte(512); loByte = lowByte(512); // half values
    EEPROM.write(waveStore[0], ESS);
    EEPROM.write(waveStore[1], RTR);
    EEPROM.write(waveStore[2], RPC);
    EEPROM.write(waveShapeStore[0], hiByte); // sine wave by default
    EEPROM.write(waveShapeStore[0] + 1, loByte);
    EEPROM.write(waveShapeStore[1], hiByte); // tri wave
    EEPROM.write(waveShapeStore[1] + 1, loByte);
    hiByte = highByte(2054); loByte = lowByte(2054);
    EEPROM.write(waveShapeStore[2], hiByte); // direct CV output 2
    EEPROM.write(waveShapeStore[2] + 1, loByte);
    EEPROM.write(trigModeStore, 7); // all set to retrig ie xxxxx111

    for (byte i = 0; i < CHANNELS; i ++) {
      hiByte = highByte(512); loByte = lowByte(512); // halfway (pots are 10 bit values)
      EEPROM.write(periodStore[i], hiByte); // full output by default
      EEPROM.write(periodStore[i] + 1, loByte);
      EEPROM.write(tweak0Store[i], hiByte);
      EEPROM.write(tweak0Store[i] + 1, loByte);
      EEPROM.write(tweak1Store[i], hiByte);
      EEPROM.write(tweak1Store[i] + 1, loByte);
      EEPROM.write(tweak2Store[i], hiByte);
      EEPROM.write(tweak2Store[i] + 1, loByte);
      hiByte = highByte(DAC_MAX);  loByte = lowByte(DAC_MAX);
      EEPROM.write(maxAmpStore[i], hiByte); // full output by default
      EEPROM.write(maxAmpStore[i] + 1, loByte);
    }
    Serial.println("EEPROM initialisation finished");
    EEPROM.write(EEPROM_CHECK_ADD, EEPROM_CHECK_KEY);    // has been initialised now thanks
  }

  for (byte i = 0; i < CHANNELS; i ++) {                       // load up last values
    waveForm[i] = EEPROM.read(waveStore[i]); // single byte
    retrigs = EEPROM.read(trigModeStore);

    hiByte = EEPROM.read(waveShapeStore[i]);
    loByte = EEPROM.read(waveShapeStore[i] + 1);
    waveShape[i] = word(hiByte, loByte);

    hiByte = EEPROM.read(tweak0Store[i]);
    loByte = EEPROM.read(tweak0Store[i] + 1);
    tweak0[i] = word(hiByte, loByte);
    hiByte = EEPROM.read(tweak1Store[i]);
    loByte = EEPROM.read(tweak1Store[i] + 1);
    tweak1[i] = word(hiByte, loByte);
    hiByte = EEPROM.read(tweak2Store[i]);
    loByte = EEPROM.read(tweak2Store[i] + 1);
    tweak2[i] = word(hiByte, loByte);
    hiByte = EEPROM.read(maxAmpStore[i]);
    loByte = EEPROM.read(maxAmpStore[i] + 1);
    maxAmp[i] = word(hiByte, loByte);

    storedWaveShape[i] = waveShape[i];
    storedWaveForm[i] = waveForm[i];
    storedTweak0[i] = tweak1[i];
    storedTweak1[i] = tweak1[i];
    storedTweak2[i] = tweak2[i];
    storedMaxAmp[i] = maxAmp[i];

    if (bitRead(retrigs, i)) {
      retrigMode[i] = true;
    } else {
      retrigMode[i] = false;
    }

    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(", waveForm is ");
    Serial.print(waveForm[i]);
    Serial.print(", waveShape is ");
    Serial.print(waveShape[i]);
    Serial.print(", tweak 0 is  ");
    Serial.print(tweak0[i]);
    Serial.print(", tweak 1 is  ");
    Serial.print(tweak1[i]);
    Serial.print(", tweak 2 is  ");
    Serial.print(tweak2[i]);
    Serial.print(", maxAmp value is  ");
    Serial.print(maxAmp[i]);
    Serial.print(", maxAmp is  ");
    Serial.print(maxAmp[i] / 409.6);
    Serial.println("v ");
  }

  Serial.print("retrig mode is (chs 3 2 1:) ");
  Serial.println(retrigs, BIN);
  Serial.println("end of load from EEPROM");
}  // end EEPROM
