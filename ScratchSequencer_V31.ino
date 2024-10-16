/* Jon Pascone | https://www.audioalchemyinc.com | https://github.com/sfxredditor/CVStepSequencer |

CV Step Outputs read pitch & decay; CV3 reads pitch in ZERO_MODE only
CV1 = DronePitch; CV2 = PostDecayTrigger; CV3 = Keyboard; CV4 = GateOutput

Design based on LookMumNoComputer's 8 step arduino sequencer:
https://www.lookmumnocomputer.com/sequencer-keyboard
*/

#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADC 0-3 setup
#define TEMPO_ANALOG_CHANNEL 0
#define START_ANALOG_CHANNEL 2  
#define STOP_ANALOG_CHANNEL 3   

// Teensy External Clock
#define EXTERNAL_CLOCK_PIN 9   
#define CLOCK_SWITCH_INT_PIN 8  
#define CLOCK_SWITCH_EXT_PIN 10 

// Teensy Direction
#define FORWARDS 11
#define BACK 12
#define RESET 29
#define ZERO 28

Adafruit_MCP4728 mcp1, mcp2, mcp3;
Adafruit_ADS1015 ads1015;

// FW, BW, RESET, ZERO logic
int val1 = HIGH, old_val1 = HIGH;
int val2 = HIGH, old_val2 = HIGH;
int val3 = HIGH, old_val3 = HIGH;
int val4 = HIGH, old_val4 = HIGH;

//init
int pitchVolt[8] = {0};
int decayVolt[8] = {0};

int ledPins[8] = {0, 1, 2, 3, 4, 5, 6, 7};
int buttonPins[8] = {37, 36, 35, 34, 33, 32, 31, 30};
int decayPots[8] = {A10, A11, A12, A13, A14, A15, A16, A17};
int pitchPots[8] = {A0, A1, A2, A3, A6, A7, A8, A9};

int currentStep = 0;
uint32_t tempo = 1000;
const int totalSteps = 8;
unsigned long lastStepTime = 0; // needed for decayValue tracking
unsigned long lastDecayTime[totalSteps]; //tracks last decay time for each step

// direction flags
bool cvForwards = false;
bool cvBackwards = false;
bool zeroState = false;

// clock mode state
enum ClockMode {
    INTERNAL_CLOCK,
    EXTERNAL_CLOCK
};
ClockMode currentClockMode = INTERNAL_CLOCK; // default for loop to function with switch logic

// seq state machine
enum SequencerState {
    STOPPED,
    RUNNING,
    ZERO_MODE
};
SequencerState currentState = STOPPED;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mcp1.begin(0x64)) { Serial.println("No DAC 01 @ 0x64"); }
  if (!mcp2.begin(0x63)) { Serial.println("No DAC 02 @ 0x63"); }
  if (!mcp3.begin(0x61)) { Serial.println("No DAC 03 @ 0x61"); }

  for (int i = 0; i < totalSteps; i++) {
    pinMode(ledPins[i], OUTPUT);
    pinMode(buttonPins[i], INPUT_PULLUP);
    lastDecayTime[i] = 0; // initialize at 0 before read value in advanceStep
  }

  // switch setup
  pinMode(FORWARDS, INPUT_PULLUP);
  pinMode(BACK, INPUT_PULLUP);
  pinMode(RESET, INPUT_PULLUP);
  pinMode(ZERO, INPUT_PULLUP);

  pinMode(EXTERNAL_CLOCK_PIN, INPUT_PULLUP);  
  pinMode(CLOCK_SWITCH_INT_PIN, INPUT_PULLUP); 
  pinMode(CLOCK_SWITCH_EXT_PIN, INPUT_PULLUP); 

  ads1015.begin();

  initializeSequencerOnPowerUp(); // force LOW before read if in sequencer mode
  initializeZeroState(); //
  updateHorizontalCV();
}

void initializeSequencerOnPowerUp() {
    bool forwardsPressed = digitalRead(FORWARDS) == LOW; // force low for regular power up behaviour 
    bool backwardsPressed = digitalRead(BACK) == LOW;

    if (forwardsPressed) {
        currentStep = 0;
        cvForwards = true;
        cvBackwards = false;
    } else if (backwardsPressed) {
        currentStep = totalSteps - 1; // force start step 8
        cvForwards = false;
        cvBackwards = true;
    } else {
        currentStep = 0; // safety
        cvForwards = true;
        cvBackwards = false;
    }

    setDACOutput(currentStep, pitchVolt[currentStep]); //pitchVolt output of currentStep analog read
    digitalWrite(ledPins[currentStep], HIGH); 

    for (int i = 0; i < totalSteps; i++) {
        if (i != currentStep) {
            setDACOutput(i, 0);
            digitalWrite(ledPins[i], LOW); // force off for PowerUp
        }
    }

    Serial.print("Zoop Zoop Zoop! Current Step: ");
    Serial.println(currentStep);
}

void initializeZeroState() {
  for (int i = 0; i < totalSteps; i++) {
    pitchVolt[i] = map(analogRead(pitchPots[i]), 0, 1023, 0, 4095);
    decayVolt[i] = map(analogRead(decayPots[i]), 0, 1023, 0, 4095); // still read decay pots
  }
}

void setDACOutput(int step, int value) {
    if (step < 0 || step >= totalSteps) return; // edge condition for safety
    if (value < 0) value = 0; // min clamp
    if (value > 4095) value = 4095; // max clamp

    if (step < 4) {
        mcp1.setChannelValue(static_cast<MCP4728_channel_t>(step), value); // need to cast for enum type
    } else {
        mcp2.setChannelValue(static_cast<MCP4728_channel_t>(step - 4), value);
    }
}

void loop() {
  checkClockMode(); // check clock

  if (currentClockMode == EXTERNAL_CLOCK) {
    Serial.println("Using external clock");
  } else {
    Serial.println("Using internal clock");
  }

  readCVInputs(); //for tempo read & START/STOP
  processSwitches(); //determine states & direction

  if (currentState == RUNNING) {
    if (!zeroState) {
      if (currentClockMode == EXTERNAL_CLOCK) {
        updateSequencerWithClock();  // advanceStep external
      } else {
        updateSequencer();  // advanceStep internal
      }
    }
  }

  updateDACAndLeds(); //lock step sequence

  if (currentClockMode == INTERNAL_CLOCK) {
    tempoUpdate();  // internal clock determined by temp
  }
}

void checkClockMode() {
  bool intPinState = digitalRead(CLOCK_SWITCH_INT_PIN); 
  bool extPinState = digitalRead(CLOCK_SWITCH_EXT_PIN);

  // state debug
  Serial.print("Int pin: "); Serial.println(intPinState);
  Serial.print("Ext pin: "); Serial.println(extPinState);

  if (intPinState == LOW) {
    currentClockMode = INTERNAL_CLOCK;  // set internal
    Serial.println("Switched to INTERNAL CLOCK");
  } else if (extPinState == LOW) {
    currentClockMode = EXTERNAL_CLOCK;  // set external
    Serial.println("Switched to EXTERNAL CLOCK");
  }
}

void readCVInputs() {
    int tempoVal = ads1015.readADC_SingleEnded(TEMPO_ANALOG_CHANNEL);
    tempo = map(tempoVal, 0, 1023, 1875, 9);

    // start stop switch function
    int startValue = ads1015.readADC_SingleEnded(START_ANALOG_CHANNEL);
    int stopValue = ads1015.readADC_SingleEnded(STOP_ANALOG_CHANNEL);

    // basic threshold to surpass
    if (startValue > 1000) {
        currentState = RUNNING;
    }
    if (stopValue > 1000) {
        currentState = STOPPED;
    }
}

void tempoUpdate() {
  int adcValue = ads1015.readADC_SingleEnded(TEMPO_ANALOG_CHANNEL);
  tempo = map(adcValue, 0, 1023, 1875, 9); // super slow and reasonably fast, reads at 2s/qurtNote
}

void updateSequencer() {
  if (millis() - lastStepTime >= (uint32_t)tempo) {
    lastStepTime = millis();
    advanceStep();

    /* when total time - lastStepTime is greater 
       then or equal to current tempo count, only then advanceStep */

    for (int i = 0; i < totalSteps; i++) {
      digitalWrite(ledPins[i], (i == currentStep) ? HIGH : LOW); // LED on for currentStep else off
    }
  }
}

void updateSequencerWithClock() {
  bool currentClockState = digitalRead(EXTERNAL_CLOCK_PIN) == LOW;
  // flag for secured functionality of EXTERNAL clockState
  static bool lastClockState = HIGH;

  /* advacnceStep according to EXTERNAL only if 
  currentClockState and LOW lastClockState */

  if (currentClockState && !lastClockState) {
    advanceStep();  
    
    for (int i = 0; i < totalSteps; i++) {
      digitalWrite(ledPins[i], (i == currentStep) ? HIGH : LOW); // LED on for currentStep else off
    }
  }

  lastClockState = currentClockState;  // reset lastClockState to setup for next EXTERNAL clock detection
}


void triggerGateOutput() {
    mcp3.setChannelValue(MCP4728_CHANNEL_D, 4095);
    delay(tempo / 10); // safe for EXT and INT
    mcp3.setChannelValue(MCP4728_CHANNEL_D, 0);
}

void advanceStep() {
    setDACOutput(currentStep, 0);

    if (cvBackwards) {
        currentStep = (currentStep == 0) ? totalSteps - 1 : currentStep - 1;
    } else {
        currentStep = (currentStep + 1) % totalSteps;
    }

    int currentPitch = analogRead(pitchPots[currentStep]);
    pitchVolt[currentStep] = map(currentPitch, 0, 1023, 0, 4095);
    
    setDACOutput(currentStep, pitchVolt[currentStep]); //pitchPot read determines output Voltage of CV1

    lastDecayTime[currentStep] = millis(); // used to determine decayValue of each step
    triggerGateOutput(); // gate trigger at beggining of each step
}

// for CV1[PitchDrone], CV2[PostDecayTrigger], CV3[Keyboard], and CV4[GateOutput]
void updateHorizontalCV() {
  if (currentState == ZERO_MODE) {
    int buttonCV = 0; // init at 0

    for (int i = 0; i < totalSteps; i++) {
      if (digitalRead(buttonPins[i]) == LOW) {
        buttonCV = pitchVolt[i]; // read pitchPot value continuously while buttonPressed
        break;
      }
    }
    mcp3.setChannelValue(MCP4728_CHANNEL_C, buttonCV);
    return;
  }

  mcp3.setChannelValue(MCP4728_CHANNEL_A, pitchVolt[currentStep]); //pitchVolt = pitchPot read

  /*  millis() • ms count since program start, lastDecayTime is sorta like an int object
      lastDecayTime[currentStep] • holds the previous time that the currentStep was advanced, then updates in the advanceStep() function
      
      subtracting lastDecayTime[currentStep] from the current time (millis()), gets elapsedTime since that step started 
      allows control of timing for decay and when to trigger the next step despite various controlling speeds.
  */


  // measure how long it's been since a step was advanced or initiated to control decay of CV Outs
  unsigned long elapsedTime = millis() - lastDecayTime[currentStep];
  int decayValue = analogRead(decayPots[currentStep]);
  unsigned long decayLength = map(decayValue, 0, 1023, tempo / 8, tempo); //decayLength are eigth subdivisions of tempo

  // if elapsedTime is greater or equal to decay legnth currentStep is HIGH
  if (elapsedTime >= decayLength) {
    mcp3.setChannelValue(MCP4728_CHANNEL_B, 4095);
  } else {
    mcp3.setChannelValue(MCP4728_CHANNEL_B, 0); // else new step is currentStep so LOW
  }
}

void updateDACAndLeds() {
    // flag for safety in zero mode, keeps LEDS LOW
    static bool buttonPressed[totalSteps] = {false};

    if (zeroState) {
        for (int i = 0; i < totalSteps; i++) {
            if (digitalRead(buttonPins[i]) == LOW) {
                pitchVolt[i] = map(analogRead(pitchPots[i]), 0, 1023, 0, 4095); // update while playing
                mcp3.setChannelValue(MCP4728_CHANNEL_C, pitchVolt[i]);
                setDACOutput(i, pitchVolt[i]); // set after update

                if (!buttonPressed[i]) {
                    triggerGateOutput(); // gate output trigger on button press
                    buttonPressed[i] = true;
                }

                digitalWrite(ledPins[i], HIGH);
                return;
            } else {
                setDACOutput(i, 0);
                digitalWrite(ledPins[i], LOW);
                buttonPressed[i] = false;
            }
        }

        mcp3.setChannelValue(MCP4728_CHANNEL_C, 0); // clamp zero if no buttons pressed
        return;
    } else {
        mcp3.setChannelValue(MCP4728_CHANNEL_C, 0);

        /*
        
        • read decayPot value for the currentStep
        • map to a time range between tempo / 8 and tempo 
        • renders decayLength per step when RUNNING state

        */

        if (currentState == RUNNING) {
            unsigned long elapsedTime = millis() - lastDecayTime[currentStep];
            int decayValue = analogRead(decayPots[currentStep]);
            unsigned long decayLength = map(decayValue, 0, 1023, tempo / 8, tempo);

            /* 
            • when elapsedTime < decayLength --> DAC continues to output the pitch for the currentStep, ergo, PostDecayOutput not triggered [CV2] 
            • if/when elapsedTime > decayLength --> DAC output for the current step is set to 0, and decay is completed, & triggers PostDecayOuput [CV2]
            */

            if (elapsedTime < decayLength) {
                setDACOutput(currentStep, pitchVolt[currentStep]);
                mcp3.setChannelValue(MCP4728_CHANNEL_A, pitchVolt[currentStep]);
                mcp3.setChannelValue(MCP4728_CHANNEL_B, 0);
            } else {
                setDACOutput(currentStep, 0); // DAC CV Step Ouput LOW
                mcp3.setChannelValue(MCP4728_CHANNEL_B, 4095); // CV2 PostDecayTrigger HIGH
            }
        } else {
            mcp3.setChannelValue(MCP4728_CHANNEL_A, 0); // safety
            mcp3.setChannelValue(MCP4728_CHANNEL_B, 0);
        }
    }
}

void processSwitches() {
    val1 = digitalRead(FORWARDS);
    val2 = digitalRead(BACK);
    val4 = digitalRead(RESET);
    val3 = digitalRead(ZERO);

    // same as INTERNAL / EXTERNAL switch, but with additional logic for resetting and direction control

    if (val3 == LOW && old_val3 == HIGH) {
        zeroState = true;
        currentState = ZERO_MODE;
        resetOutputsForZeroMode();
        Serial.println("ZERO_MODE");
    }

    if (zeroState && val3 == HIGH) {
        zeroState = false;

        if (val1 == LOW) {
            currentStep = 0;
            cvForwards = true;
            cvBackwards = false;
            currentState = STOPPED;

            digitalWrite(ledPins[currentStep], HIGH);
            setDACOutput(currentStep, pitchVolt[currentStep]);

            Serial.println("Exited ZERO_MODE, RESET to Step 0, FORWARDS");
        } else if (val2 == LOW) {
            currentStep = totalSteps - 1;
            cvForwards = false;
            cvBackwards = true;
            currentState = STOPPED;

            digitalWrite(ledPins[currentStep], HIGH);
            setDACOutput(currentStep, pitchVolt[currentStep]);

            Serial.println("Exited ZERO_MODE, RESET to Step 7, BACKWARDS");
        } else {
            currentState = STOPPED;
            Serial.println("ZERO_MODE, Sequnecer STOPPED");
        }
    }

    if (!zeroState) {
        if (val1 == LOW && old_val1 == HIGH) {
            cvForwards = true;
            cvBackwards = false;
            if (currentState != STOPPED) {
                currentState = RUNNING;
            }
            Serial.println("FORWARDS");
        }
        old_val1 = val1;

        if (val2 == LOW && old_val2 == HIGH) {
            cvBackwards = true;
            cvForwards = false;
            if (currentState != STOPPED) {
                currentState = RUNNING;
            }
            Serial.println("BACKWARDS");
        }
        old_val2 = val2;

        if (val4 == LOW && old_val4 == HIGH) {
            if (cvForwards) {
                currentStep = 0;
            } else if (cvBackwards) {
                currentStep = totalSteps - 1;
            }

            digitalWrite(ledPins[currentStep], HIGH);
            setDACOutput(currentStep, pitchVolt[currentStep]);

            if (currentState != STOPPED) {
                currentState = RUNNING;
            }
            Serial.println("RESET");
        }
        old_val4 = val4;
    }

    old_val3 = val3;
}

// extra safety for proper function of ZERO_MODE
void resetOutputsForZeroMode() {
    mcp3.setChannelValue(MCP4728_CHANNEL_A, 0);
    mcp3.setChannelValue(MCP4728_CHANNEL_B, 0);
    setDACOutput(currentStep, 0);
    digitalWrite(ledPins[currentStep], LOW);
    mcp3.setChannelValue(MCP4728_CHANNEL_D, 0);
}

