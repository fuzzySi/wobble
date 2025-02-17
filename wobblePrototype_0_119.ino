/*
 * 
 *  wobble module by fuzzySynths
 *  code for Arduino Nano
 *  
    3 (? 4) EGs / LFOs in 4u eurorack format
    This controls 4 x 12bit DAC buffered outputs. These can be set as EG, LFO or direct pot control of CV.
    3 incoming gates trigger EGs, or restarts LFOs.
    prototype PCBs have a 4th output available on the PCB but not front panel [is this buffered?]
    
    UI:
        hold button to change waveform
        change knob that channel to change shape - envelope / sine / triangle / square / random / CV control
        press button to set that waveform
        knob changes frequency
        can change extra parameter by holding button & turning knob (env midpoint, sine phase, triangle midpoint, pulse wave duty cycle). 
        the final two (random and CV) control min and max levels - these persist across waveforms, so can attenuate the range for all waves, or invert outputs by setting min to 5v and max to 0v
            warning - if min & max are set to the same value, it'll spring 1v apart to save feeling like modules is borked

    TO DO:
        4th channel (ch 3) sends random signals changing on signals from all 3 inputs

  possible developments for future?
    learn mode: time between incoming gate clocks becomes next cycle of LFO - really interesting when driven by random source
    store last values in EEPROM ? including pot values? but not maximum amplitude - only changed for that time.

  licence
    share and share alike creative commons licence, fuzzySynths 2018-2025
    some code ideas borrowed from http://abrammorphew.com/notes/2012/01/05/arduino-lfo-generator/

*/

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#include <SPI.h>
#include <TimerOne.h>
#include <ResponsiveAnalogRead.h>
// #include <EEPROM.h>
#include <Bounce2.h>

// #include <elapsedMillis.h>

// editable
const int CHANNELS = 3; // 
const int OUTPUTS = 4; // includes 4th on back of PCB
const int POT_HYSTERESIS = 10; // keep low, but if getting weird errors with 1 or more channels, increase this (try 8)
const unsigned int MIN_PERIOD = 20; // is this ms? // experiment with this - LED will cut out at high freqs, monitor then reduce this until stops
const unsigned int MAX_PERIOD = 30000; // overflows much past 30 secs. sorry. // 34000 works out at around 4.8 secs at TIMECALL of 300.
const int HOLD_TIME = 800; // ms button needs to be held to not be a trigger
const int LONG_HOLD_TIME = 2000; // ms, beyond this will cycle around waveforms for that channel
const int MIN_ENV = 200; // minimum attack & release to prevent glitches
const int MAX_ENV = 8000;
const int MIN_DUTY = 5; // % of cycle
const int MAX_DUTY = 95; // % 
const int MIN_RAMP = 0; // in ms
const int TIMECALL = 900; // how often i updates ie max freq, crashes if too low. lengthen period to compensate. 10 for unoptimised v27 code
const int DEBOUNCE = 50; // ms
const unsigned long POT_POWER = 2;           // exponent to derive period from, 3 gives nice curve. a linear pot to period equation is clumsy, all the range at one end
     // https://docs.google.com/spreadsheets/d/1pRY8PRPjB4lb8PoqfdWUBQF9YMBjLJbPhRVFbxrdJSU/edit?usp=sharing for working out
long occCheckInterval = 200; // try 2000 = 1 every 2 secs secs, check next channel, update EEPROM if needed
long testTime;

/*
// EEPROM addresses to hold current settings if needed
int waveStore[4] = {0, 1, 2, 3};
int tweakStore[4] = {44, 46, 48, 50};
const byte EEPROM_CHECK_KEY = 67;     // key - change this to anything else between 0 & 255 if you need to reinitialise the default settings
const byte EEPROM_CHECK_ADD = 99;      // address where eepromID should be stored, if it's not key, initialise settings
*/

// don't change these
const int QTR_SINE = 256;
const int WHOLE = QTR_SINE * 4; // full period is 1024 (? 4096 gives better resolution - no, this is position along, wave table does give 12 bit values)
const int DAC_MAX = 4095; // 12 bit resolution for MCP4822 DAC (4096 loses important bit)
const int SINE_PHASE_FUDGE = 0; // magic number, moves starting phase to 0 preventing glitches on sine wave - 225 works
const int HALFWAY = 511; // half pot position = 1022/2

// DAC out parameters if needed - best not to change these
const int GAIN_1 = 0x1; // 1 is no gain
const int GAIN_2 = 0x0; //
const int CH_A = 1; // inverted to match PCB
const int CH_B = 0;
const int SHUTDOWN = 1;

// waveforms
const int MAX_WAVEFORMS = 5; //  ie 6 (0-5), beyond this, overflows back to 0

const int ENV = 0;
const int SINE = 1;
const int TRI = 2;
const int PULSE = 3;
const int RANDOM = 4;
const int CV = 5;

int defaultTweak[MAX_WAVEFORMS + 1] = {256, 0, 512, 512, 1024, 1024};  // this is secondary function for each waveform
                                    // ENV, attack:release ratio; SINE, phase; TRI, ramp shape; PULSE, duty; RANDOM, max; CV, max

// LED colours
const int OFF = 0;
const int GREEN = 1;
const int RED = 2;


// one quarter of
int sineTable[] = {
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
};  // much quicker to use table than calculate


// pins
const int greenPin[] = {3, 6, 5};
const int redPin[] = {4, 7, A3}; // check this // wired reversed in prototype, should be red4 A2, green4 9
const int gateIn[] = {A2, A1, A0};
const int potPin[] = {A5, A6, A7}; //
const int buttonPin[] = {2, 8, A4};

// for 4 channel first prototype
// const int greenPin[] = {3, 5, 6, 9};
// const int redPin[] = {2, 4, 7, A2}; // check this // wired reversed in prototype, should be red4 A2, green4 9
// const int gateIn[] = {12, A1, A0}; // change this from prototype?
// const int potPin[] = {A4, A5, A6, A7}; //
// const int buttonPin = A3; // 3 buttons via different resistors, into 1 pin

const byte DAC0_CS = 10; // only used in setup, direct port manipulation used in main loop instead
const byte DAC1_CS = 9;  //
const byte MIDI_IN_PIN = 0;
const byte MIDI_OUT_PIN = 1;

Bounce button0 = Bounce();
Bounce button1 = Bounce();
Bounce button2 = Bounce();

ResponsiveAnalogRead analog0(potPin[0], true); // sleepEnabled or not. more twitchy with true, smoother with disabled
ResponsiveAnalogRead analog1(potPin[1], true);
ResponsiveAnalogRead analog2(potPin[2], true);

// central vars
volatile unsigned int i = 0; // clock, updates on timer
int takeTurns = 0; // reads each pot in order, only checks one pot each cycle to save time
long occCheckTime = 0;
byte param = 0; // current setting to store in EEPROM, start at 0 as it updates first
byte hiByte, loByte;

// buttons
byte buttonPressed[CHANNELS] = {false, false, false};
byte firstButton = 9; // holds first button pushed (to allow multiple combinations), 9 being off
boolean newPress = false; // true from start of button until button processed, or turned off
boolean buttonHeld = true; // true forces it to update first run
boolean buttonReleased = true;
long buttonStartTime;
long buttonHeldTime;

// channel vars
volatile int potValue[CHANNELS];
volatile signed int value[CHANNELS]; // = {1000, 2512, 2512, 2512}; // output value (12 bit 0-4096) use volatile if timer may change value
volatile int lastPotValue[CHANNELS];
volatile boolean potNotCorrect[CHANNELS] = {false, false, false};
int rememberPotFreq[CHANNELS];
volatile int LEDcolour[CHANNELS]; // 0 is off, 1 is green, 2 is red
volatile int tweak[CHANNELS]; // duty cycle / phase / EG sustain level
volatile boolean pulseOn[CHANNELS] = {false, false, false};
// volatile boolean envFinished[CHANNELS] = {true, true, true}; // stops it retriggering if envelope changed to shorter
volatile boolean tweakMode;
volatile boolean tie_1_2 = false; // stores whether trigger on channel 1 works on channel 2 etc
volatile boolean tie_1_3 = false;
volatile boolean tie_2_3 = false;
volatile boolean tieDone = false;

int waveForm[CHANNELS]; // holds which of 3 main waveforms is used (set by pot) // 4 channels // doh! declaring it to [3] (ie 0-2) gave errors, as no waveform[3]
int newWaveForm[CHANNELS];
int maxLevel[CHANNELS] = {DAC_MAX, DAC_MAX, DAC_MAX};                                             // can change max level using tweak for random or CV
int minLevel[CHANNELS] = {0, 0, 0};

volatile unsigned long place[CHANNELS] = {0, 0, 0}; // place along cycle
volatile unsigned long period[OUTPUTS]; // cycle length

volatile boolean gate[CHANNELS] = {false, false, false};
volatile boolean oldGate[CHANNELS] = {false, false, false};
volatile boolean retrigger[CHANNELS] = {false, false, false}; // 4 channels, 3rd duplicated into 4th at the time

int counter[CHANNELS] = {0, 0, 0};       // how many times we have seen new value
int reading[CHANNELS];           // the current value read from the input pin
unsigned long lastCheck[CHANNELS] = {0, 0, 0};         // the last time the output pin was sampled
boolean current_state[CHANNELS];
volatile int tweakChannel = 9;
volatile int editChannel = 9; // 9 is off


const boolean DEBUG = false; // true;


// ********************  setup ********************************

void setup() {
  Serial.begin(115200);

  SPI.begin();
  Timer1.initialize(TIMECALL); // updates every x usecs - reduce to get higher Hz
  Timer1.attachInterrupt(updateClock); //

  pinMode(greenPin[0], OUTPUT);
  pinMode(greenPin[1], OUTPUT);
  pinMode(greenPin[2], OUTPUT);

  pinMode(redPin[0], OUTPUT);
  pinMode(redPin[1], OUTPUT);
  pinMode(redPin[2], OUTPUT);

  pinMode(buttonPin[0], INPUT_PULLUP); // INPUT); // don't ask me why input pullup doesn't work on this one, write high later
  pinMode(buttonPin[1], INPUT_PULLUP);
  pinMode(buttonPin[2], INPUT_PULLUP);


  pinMode(DAC0_CS, OUTPUT);
  pinMode(DAC1_CS, OUTPUT);

  pinMode(gateIn[0], INPUT);
  pinMode(gateIn[1], INPUT);
  pinMode(gateIn[2], INPUT);


  pinMode(potPin[0], INPUT);
  pinMode(potPin[1], INPUT);
  pinMode(potPin[2], INPUT);

  button0.attach(buttonPin[0], INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  button0.interval(DEBOUNCE); // Use a debounce interval of DEBOUNCE (~50) milliseconds
  button1.attach(buttonPin[1], INPUT_PULLUP);  
  button1.interval(DEBOUNCE);
  button2.attach(buttonPin[2], INPUT_PULLUP);
  button2.interval(DEBOUNCE);


  //  DDRB = B11101110; // pins 13-8,  0 is input, 1 is out
  //  DDRC = B00010000; // pins A7-0
  //  DDRD = B11111010; // ports pins 7-0

  // pinMode(redPin[2], OUTPUT);
  // pinMode(greenPin[2], OUTPUT); // this should be covered in above ports, but doesn't work without declaring. don't ask me why

  for (int i = 0; i < CHANNELS; i ++) {
        tweak[i] = defaultTweak[waveForm[i]];
  }
  
  digitalWrite(DAC0_CS, HIGH);
  digitalWrite(DAC1_CS, HIGH);

  // for testing
  testTime = micros();

  // checkEEPROM(); // not used

  // starting waveforms
  waveForm[0] = PULSE;                                                                                                          // set initial waves
  waveForm[1] = SINE; 
  waveForm[2] = RANDOM; 
  waveForm[3] = RANDOM;

  LEDcolour[0] = GREEN;
  LEDcolour[1] = GREEN;
  LEDcolour[2] = GREEN;

  gate[0] = false;
  gate[1] = false;
  gate[2] = false;

}

// ********************  loop  start ********************************
void loop() {                                                                                                                   // *** MAIN LOOP ***

  // int lag = micros() - testTime;                                                                                                // test loop time
  //  Serial.println(lag);
  // testTime = micros();
  // 560 best yet

                                                                                                                                // update timer
  // more complicated than updating each time, but allows interpolation esp. when cycles might be slower (housekeeping)
  if (i) {                                                                                                                      // if timer has updated
    noInterrupts();                                                                                                             // read v quick
    unsigned int incr = i;                                                                                                      // how many have there been since last checked?
    //    Serial.println(i);
    interrupts();

    place[0] += incr;    // update place along wave
    place[1] += incr;
    place[2] += incr;
    place[3] += incr;
    i = 0;
  }


  readPots(takeTurns);                                                                                                           // read interface one pot each time around to save processing time for waveform generation
  takeTurns ++;
  if (takeTurns >= CHANNELS) {
    takeTurns = 0;
  }

  checkButtons();
  checkGates();
  updateLEDs();


    for (int ch = 0; ch < CHANNELS; ch ++) {
      value[ch] = updateOutputs(ch, waveForm[ch], rememberPotFreq[ch], tweak[ch]);  
      if (value[ch] < 0) {
        value[ch] = 0;
      }
    }

  // write values to DAC
  setOutput(1, CH_A, 0, value[3]); // chip, channel, gain (0 is 2x), value
  setOutput(1, CH_B, 0, value[1]); // just the way it's wired
  setOutput(0, CH_A, 0, value[0]); // setOutput(1, CH_A, 0, value[3]); goes to output 4
  setOutput(0, CH_B, 0, value[2]); //

  //  houseKeeping(); // occ backups to EEPROM only

}

// ********************  loop  end ********************************



// ********************* functions **************************

void checkButtons() { // sets firstButton (if multiple) and buttonPressed[] true or false                                       // *** READ BUTTONS ***
  button0.update(); // Update the Bounce instance
  button1.update();
  button2.update();

  buttonPressed[0] = !button0.read(); // inverts, so pressed is true when it is pressed (library does true voltage - input pullup so goes low when pressed)
  buttonPressed[1] = !button1.read();
  buttonPressed[2] = !button2.read();
  
  if ((buttonHeld == false)) { // (millis() > buttonStartTime + 10) &&                                                          // newly pressed, which button was it? stored in firstButton
    if ((buttonPressed[0] == true)) {
      firstButton = 0;
      buttonHeld = true;
      buttonStartTime = millis();
      buttonHeldTime = buttonStartTime;
    } else if (buttonPressed[1] == true) {
      firstButton = 1;
      buttonHeld = true;
      buttonStartTime = millis();
      buttonHeldTime = buttonStartTime;
    } else if (buttonPressed[2] == true) {
      firstButton = 2;
      buttonHeld = true;
      buttonStartTime = millis();
      buttonHeldTime = buttonStartTime;
    }
    if (DEBUG) { Serial.print(" pressed on channel :"); Serial.println(firstButton); } // which button pressed  
  }
  

  if (buttonHeld) {
    if ((millis() > buttonStartTime + HOLD_TIME)) {                                                                                // button pressed, not too long yet                                                                                      
     LEDcolour[firstButton] = RED; // turn on red LED if not being tweaked
    } 
    if ((firstButton == 0) && (buttonPressed[1] == true)) {
      tie_1_2 = true;
      tieDone = true;
     // Serial.println("tie 1-2");
      LEDcolour[1] = RED; // not working in prototype, sorry
    }
    if ((firstButton == 0) && (buttonPressed[2] == true)) {
      tie_1_3 = true;
      LEDcolour[2] = RED;
       //     Serial.println("tie 1-3");
    }
    if ((firstButton == 1) && (buttonPressed[2] == true)) {
      tie_2_3 = true;
      LEDcolour[2] = RED;
         //   Serial.println("tie 2-3");
    }
    if ((firstButton == 1) && (buttonPressed[0] == true)) {
      tie_1_2 = false;
      LEDcolour[1] = GREEN;
       //     Serial.println("untied 1-2");
    }
    if ((firstButton == 2) && (buttonPressed[0] == true)) {
      tie_1_3 = false;
      LEDcolour[2] = GREEN;
        //          Serial.println("untied 1-3");
    }
    if ((firstButton == 2) && (buttonPressed[1] == true)) {
      tie_2_3 = false;
      LEDcolour[2] = GREEN;
        //          Serial.println("untied 2-3");
    }
  }

  if ((buttonHeld) && (buttonPressed[0] == false) && (buttonPressed[1] == false) && (buttonPressed[2] == false)) {                  // newly released
        buttonHeld = false;

    if (millis() < (buttonStartTime + DEBOUNCE)) {                                                                                  // just a bounce, do nothing
      buttonStartTime = millis();
      firstButton = 9;
      editChannel = 9;
    } else if (millis() < (buttonStartTime + HOLD_TIME)) {                                                                            // released after quick press, triggers waveform
      if (editChannel > 8) {                                                                                                          // only retrig if not in edit mode
      retrigger[firstButton] = true;
      }
    if (DEBUG) {Serial.print("retrigger, quickly release on channel :"); Serial.println(firstButton);}
    LEDcolour[firstButton] = GREEN;
      firstButton = 9;
      buttonStartTime = millis();
      
      if ((editChannel < 9) && (!tieDone)) {                                                                                           // toggle out of edit mode when button pressed
        waveForm[editChannel] = newWaveForm[editChannel];                                                                              // update waveform

     if (DEBUG) {Serial.print("channel "); Serial.print(editChannel); Serial.print(", selected waveform is "); Serial.println(waveForm[editChannel]);}
       LEDcolour[editChannel] = GREEN;
      // Serial.print("LEDs:"); Serial.print(LEDcolour[0]); Serial.print(LEDcolour[1]); Serial.println(LEDcolour[2]);
        editChannel = 9;    
        }
    } else if (millis() >= (buttonHeldTime + HOLD_TIME)) {                                                                              // toggle into edit mode if not  
        if (editChannel > 8 && (!tweakMode) && (!tieDone)) {                                                                                            // don't if pot moved while button held, that's tweak mode instead
      editChannel = firstButton;
      LEDcolour[editChannel] = RED;
      } 

      if (DEBUG) {Serial.print(" edit mode on channel :"); Serial.print(editChannel); Serial.print(", waveform is :"); Serial.println(newWaveForm[editChannel]); }
    //  buttonHeld = false;
      firstButton = 9;
      buttonStartTime = millis();
    } 
     tweakMode = false;
     tieDone = false;
  } // end of released

}

void readPots(byte pot) { // reads one pot at a time                                                                              // *** READ POTS ***
  long temp; // MUST be a long
  switch (pot) {
    case 0:
      analog0.update();
      temp = analog0.getValue();
      break;
    case 1:
      analog1.update();
      temp = analog1.getValue();
      break;
    case 2:
      analog2.update();
      temp = analog2.getValue();
      break;
  }
  potValue[pot] = temp;

    if (lastPotValue[pot] != potValue[pot]) { // changed     
      
      if (editChannel < 9) { // edit mode                                                                                       // tweaking pot in edit mode changes waveform   
          newWaveForm[editChannel] = map (potValue[editChannel], 0, 1000, 0, MAX_WAVEFORMS);  // 1000 to ensure final one gets space
        if (DEBUG) {Serial.print("channel "); Serial.print(editChannel); Serial.print(", possible waveform is "); Serial.println(newWaveForm[editChannel]);} 
        if (newWaveForm[editChannel] % 2 == 1) {                                                  // odd number, so change colour
          LEDcolour[editChannel] = RED;                                                                                            // alternating colours to feedback change
        } else {
          LEDcolour[editChannel] = GREEN;
        }
        if (DEBUG) {Serial.print("LEDs:"); Serial.print(LEDcolour[0]); Serial.print(LEDcolour[1]); Serial.println(LEDcolour[2]);}
      }                                                                                 // pot has changed
      if (buttonHeld && firstButton == pot) {                                                                                     // button held, so changing tweak value instead
        tweakMode = true; 
        if (waveForm[pot] == RANDOM) {
          minLevel[pot] = map(potValue[pot], 0, 1024, 0, 1024); 
        } else if (waveForm[pot] == CV) {
          maxLevel[pot] = map(potValue[pot], 0, 1024, 0, 1024); 
        } else {
        tweak[firstButton] = potValue[pot];
        }
        if (abs(maxLevel[pot] - minLevel[pot]) < 205) { // if within 1v 0f each other, will move apart to 1v, raising max level if needed
          if (maxLevel[pot] < 205) {
            maxLevel[pot] = 205;
          }
          minLevel[pot] = maxLevel[pot] - 205;
        }

        if (DEBUG) {  Serial.print("channel "); Serial.print(firstButton); Serial.print("tweak mode "); Serial.println(potValue[pot]); }
      } 
        if (!tweakMode && editChannel > 8) {  // not edit or tweak mode, update frequency                                                      // update pot value if changed not in edit mode, changes frequency
      rememberPotFreq[pot] = potValue[pot];   // rememberPotFreq stores pot value when not being edited
    }  
          lastPotValue[pot] = potValue[pot];  // reset  
  } // end pot changed

}



int updateOutputs(int ch, int waveform, int potFr, int tweakValue) {                                     // *** UPDATE OUTPUT VALUE ***

      bool trig = false; 
      unsigned int midCycle; 
      unsigned int outValue; 
      int duty;
      long period;

      period = derivePeriod(potFr); 

      if (retrigger[ch]) {                             // trigger received - from button, & from gate
        trig = true;
        retrigger[ch] = false;
      }


    switch (waveform) {

      case ENV:
        period = map(potFr, 0, 1022, MIN_ENV, MAX_ENV); 
        midCycle = int(map(tweakValue, 0, 1024, 0, period));                          // tweak controls mid shape of AR, so potValue controls overall period
        if (place[ch] < midCycle ) {   // first half of waveform
          outValue = int(map(place[ch], 0, midCycle, minLevel[ch], maxLevel[ch]));
        } else {
          outValue = int(map(place[ch], midCycle, period, maxLevel[ch], minLevel[ch]));
        }
        if (trig) {
          place[ch] = 0;
        }
        if (place[ch] > period) {
          place[ch] = period; // needs to stop as otherwise would overflow around eventually
        }
        if (outValue < 0) {
          outValue = 0;
        }
       // outValue = constrain(outValue, 0, maxLevel[ch]);
      break;    

      case SINE:
        outValue = int(sine(map(place[ch], 0, period, minLevel[ch], maxLevel[ch] / 4), tweakValue)); 
        if ((place[ch] >= period) || (trig)) { // loop if time to or triggered
          place[ch] = 0;
        }
      break;

      case TRI:
        midCycle = int(map(tweakValue, 0, 1024, MIN_RAMP, period - MIN_RAMP));
        if (place[ch] < midCycle ) { // first half of waveform
          outValue = int(map(place[ch], 0, midCycle, minLevel[ch], maxLevel[ch]));
        } else {
          outValue = int(map(place[ch], midCycle, period, maxLevel[ch], minLevel[ch]));
        }
        if ((place[ch] >= period) || (trig)) {
          place[ch] = 0;
        }
      break;

      case PULSE:
        duty = int(map(tweakValue, 0, 1024, MIN_DUTY * period / 100, MAX_DUTY * period / 100));
        if (trig) {                                       // toggle pulse state
          pulseOn[ch] = !pulseOn[ch];
        }
        if (potFr > 1) {                               // only LFO if rate not turned off
          if (place[ch] < duty) {                         // on for first half of waveform
            outValue = maxLevel[ch];
          } else {
            outValue = minLevel[ch];
          }
        } 
        if (potFr <= 1) {
          if (pulseOn[ch]) {                              // otherwise trigger toggles pulse on & off
            outValue = maxLevel[ch];
          } else {
            outValue = minLevel[ch]; 
          }
        }
        if ((place[ch] >= period) || (trig)) {
          place[ch] = 0;
        }
        break;

      case RANDOM:
        if ((place[ch] >= period)) {
          if (potValue[ch] > 1) {                 // don't generate if rate turned off, wait for triggger
          outValue = random(minLevel[ch], maxLevel[ch]);    // TODO make random quicker
          }
          place[ch] = 0;
        } else {
          outValue = value[ch];               // otherwise only gives a value on trig & doesn't sustain
        }
        if (trig) {
          outValue = random(minLevel[ch], maxLevel[ch]);
        }
      break;

      case CV:
          outValue = map(potValue[ch], 0, 1022, 0, DAC_MAX); 
      break;


    } // end switch

  return outValue;

} // end update outputs


long derivePeriod(int potValue) {
    // receives 0-1024 value, returns from 30000 to 20 but with a part curve so more log than lin
    int out;
    if (potValue <= 200) {
      out = map(potValue, 0, 200, 30000, 9000);
    } else if (potValue > 200 && potValue <= 400) {
      out = map(potValue, 200, 400, 9000, 3000);
    } else if (potValue > 401 && potValue <= 600) {
      out = map(potValue, 400, 600, 3000, 1000);
    } else if (potValue > 601 && potValue <= 850) {
      out = map(potValue, 600, 850, 1000, 250);
    } else if (potValue > 850) {
      out = map(potValue, 850, 1024, 250, 20);
    }
  return out;
}



void checkGates() {                                                                                                         // *** CHECK GATES ***
                                              // start of gate on should send retrigger[ch] boolean, and keep gate[ch] positive  
  for (int i = 0; i < CHANNELS; i ++) {
    gate[i] = !(digitalRead(gateIn[i])); // hardware inverts gate in, so now gate is true when signal received
    if (gate[i] == HIGH) {
      if (oldGate[i] == false) {         // new trigger 
        retrigger[i] = true; 
        if ((i == 0) && (tie_1_2 == true)) { // check ties & trigger extra channels if ties true
          retrigger[1] = true;
        }
        if ((i == 0) && (tie_1_3 == true)) {
          retrigger[2] = true;
        }
        if ((i == 1) && (tie_2_3 == true)) {
          retrigger[2] = true;
        }        
        oldGate[i] = true; // don't trigger again
      //  Serial.print("gates: "); Serial.print(gate[0]); Serial.print(", "); Serial.print(gate[1]); Serial.print(", "); Serial.println(gate[2]);
      }
    } else if (gate[i] == LOW) {
      oldGate[i] = false;
    }
  }
}



void updateLEDs() {                                                                                                                  // *** UPDATE LEDs ***
  for (int i = 0; i < CHANNELS; i ++) {

    if ((LEDcolour[i] == RED) || (gate[i])) {
      turnRedOn(i); //       digitalWrite(redPin[i], HIGH);
    } else if (LEDcolour[i] != RED) {
      turnRedOff(i); //     digitalWrite(redPin[i], LOW);
    }  
    if ((i == editChannel) && (LEDcolour[i] == GREEN)) {
      analogWrite(greenPin[i], 0); // mid range - is this high or low?                                                                                 // turn on if in edit mode & green
    } else if ((i == editChannel) && (LEDcolour[i] == RED)) {
      turnRedOn(i);
      analogWrite(greenPin[i], 255);
    } else {
      turnRedOff(i);
    analogWrite(greenPin[i], 255 - (value[i] / 16) ); // /16 brings down to 8 bit resolution for PWM, inverted
    }
  }
}



int sine(int i, int phase) {    // sine wave look up table dose middle to top, ie top left bit of cirlce
  int place = i - phase + SINE_PHASE_FUDGE; // magic number, to get sine to start at 0 with duty, tweak[ch] at half pot, 512
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


void turnRedOn(byte i) { // inverted due to PCB
  if (i == 0) {
    //   PORTD = PORTD & 0xFB; // turn off PD2
    // PORTD = PORTD & 0xEF; // turn off PD4
    // easier in binary - goes backwards from right though?
    PORTD = PORTD & 0b11101111; // D4  high, rest unchanged, PORTD on the Uno.
  } else if (i == 1) {
    PORTD = PORTD & 0b01111111; // turn off PD7
  } else if (i == 2) {
    PORTC = PORTC & 0b11110111; // turn off PC3
  }
}


void turnRedOff(byte i) { // inverted due to PCB
  if (i == 0) {
    //    PORTD = PORTD | 0x10; // turn on PD4
    PORTD = PORTD | 0b00010000;
  } else if (i == 1) {
    PORTD = PORTD | 0b10000000; // turn on PD7
    // PORTD = PORTD | 0x80;
  } else if (i == 2) {
    PORTC = PORTC | 0b00001000;
    //    PORTC = PORTC | 0x08; // turn on PC3
  }

}



void setOutput(byte chip, byte channel, byte gain, unsigned int val) // bit 12 shutdown 1 is active, 0 is no output
{
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | channel << 7 | gain << 5 | SHUTDOWN << 4; // 7 is bit 15, 5 bit 13. gain 0 is 2x
  // DAC0_CS = 10 (PB2),  DAC1_CS = 9 (PB1) ;
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
