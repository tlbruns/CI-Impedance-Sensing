#include <ADC.h>
#include <IntervalTimer.h>

ADC *adc = new ADC();
int adcCount = 0;
int adcSum = 0;
bool adcFlag = false;
double voltagePositive = 0.0;
double voltageNegative = 0.0;
double voltageShorted = 0.0;
double adc2Voltage = 0.0;

const int SWP1 = 17;
const int SWN1 = 8;
const int SWP2 = 16;
const int SWN2 = 9;
const int pulseTime = 35; // [us] time for each current pulse (+/-); total time for biphasic pulse is 2*pulseTime
const int interPulseDelay = 20; // multiples of pulseTime between pulses ([us] = pulseTime * interPulseDelay)

IntervalTimer timerPulse;
bool timerFlag = false;
int timerCount = 0;
bool runFlag = false;

bool buttonFlag = false;
const int buttonPin = 3; // momentary tactile button

char cmd = 0;

int loopCount = 0;

//********************************************************************
// FUNCTIONS
//********************************************************************

void pulseISR() {
  if (runFlag) { // do nothing if not running
    timerFlag = true;
  }
}

void buttonISR() {
  static unsigned long debounceTime = 0;
  unsigned long currentTime = millis();
  
  // If interrupts come faster than N ms, assume it's a bounce and ignore
  if (currentTime - debounceTime > 250) {
    buttonFlag = true;
  }
  
  debounceTime = currentTime;
}

void setStateOff() {
  // all gates LOW
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
  digitalWriteFast(SWN1, LOW);
  digitalWriteFast(SWN2, LOW);
}

void setStateCurrentPositive() {
  // Allow positive current from P1 to N1
  digitalWriteFast(SWN2, LOW); // ensure off
  digitalWriteFast(SWP2, LOW);
  digitalWriteFast(SWN1, HIGH); // turn on N1 before P1
  digitalWriteFast(SWP1, HIGH);
}

void setStateCurrentNegative() {
  // Allow positive current from P2 to N2
  digitalWriteFast(SWN1, LOW); // ensure off
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWN2, HIGH); // turn on N2 before P2
  digitalWriteFast(SWP2, HIGH);
}

void setStateShorted() {
  // Short load through N1/N2
  digitalWriteFast(SWN1, HIGH); // ensure off
  digitalWriteFast(SWN2, HIGH);
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
}

double computeAvgVoltage(int sum, int count){
  double adcAvg = ((double)sum) / ((double)count);
  double volt = adcAvg * adc2Voltage;
  return volt;
}

//********************************************************************
// SETUP
//********************************************************************

void setup() {
  
  // set up control pins for MOSFET switches
  pinMode(SWP1, OUTPUT);
  pinMode(SWN1, OUTPUT);
  pinMode(SWP2, OUTPUT);
  pinMode(SWN2, OUTPUT);
  setStateOff(); // sets all to LOW

  pinMode(A10, INPUT); // Diff Channel 0 +
  pinMode(A11, INPUT); // Diff Channel 0 -
  adc->setAveraging(8); // average 4 samples for each measurement
  adc->setResolution(16); // 16 bit resolution
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // HIGH_SPEED adds +6 ADCK; MED_SPEED adds +10 ADCK
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // sets ADCK to highest speed within spec for all resolutions
  adc2Voltage = 3.3/adc->getPGA()/adc->getMaxValue(); // conversion factor for adc values
  
  Serial.begin(115200);
  Serial.println("Type 'a' or press button to begin");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT);
  attachInterrupt(buttonPin, buttonISR, RISING);
  
  while ((cmd != 'a') && (!buttonFlag)) {
    if (Serial.available() > 0) {
      cmd = Serial.read();
    }
  }
  
  Serial.print("+100 uA pulse for ");
  Serial.print(pulseTime);
  Serial.print(" us, followed immediately by -100 uA pulse for ");
  Serial.print(pulseTime);
  Serial.println(" us");
  Serial.print("Inter-pulse delay is ");
  Serial.print(interPulseDelay * pulseTime);
  Serial.println(" us\n");
  Serial.println("\nType 's' to start pulses");

  while ((cmd != 's') && (!buttonFlag)) {
    if (Serial.available() > 0) {
      cmd = Serial.read();
    }
  }

  runFlag = true;
  buttonFlag = false;
    
  Serial.println("\nStarting Pulses");
  timerPulse.begin(pulseISR, pulseTime);
  //runFlag = true;
}

//********************************************************************
// MAIN LOOP
//********************************************************************

void loop() {

  if(buttonFlag) {
    buttonFlag = false;
    timerFlag = false;
    timerCount = 0;
    runFlag = !runFlag;
    digitalWriteFast(LED_BUILTIN, runFlag);
    delay(300);
    Serial.print("\n\nrunFlag = ");
    Serial.println(runFlag);
  }

  if(runFlag) {
    // timerFlag is set every pulseTime [us], triggering state machine to step forward
    if (timerFlag) {
      timerFlag = false; // reset flag
      timerCount++; // increment
      
      switch (timerCount) {
        case 0 : // should never actually hit this
          setStateOff();
          break;
          
        case 1 : // +100 uA pulse
          voltageShorted = adc2Voltage * adc->adc0->analogReadDifferential(A10,A11);
          setStateCurrentPositive();
          delayMicroseconds(1);
          voltagePositive = adc2Voltage * adc->adc0->analogReadDifferential(A10,A11);
          adcCount = 0; // reset
          adcSum = 0; // reset
          //adcFlag = true; // start measuring voltage
          break;
          
        case 2 : // -100 uA pulse
//          voltagePositive = adc2Voltage * adc->adc0->analogReadDifferential(A10,A11);
//          setStateOff(); // ensure off first
          //voltagePositive = computeAvgVoltage(adcSum, adcCount);
          adcCount = 0;
          adcSum = 0;
          setStateCurrentNegative();
          delayMicroseconds(1);
          voltageNegative = adc2Voltage * adc->adc0->analogReadDifferential(A10,A11);
          break;
  
        case 3 : // turn all off and short the load
//          voltageNegative = adc2Voltage * adc->adc0->analogReadDifferential(A10,A11);
          setStateOff();
          //voltageNegative = computeAvgVoltage(adcSum, adcCount);
          adcCount = 0;
          adcSum = 0;
          setStateShorted();
          break;
  
        case (2 + interPulseDelay): // prepare for next pulse
          timerCount = 0; // will increment to 1 on next interrupt
//          voltageShorted = adc2Voltage * adc->adc0->analogReadDifferential(A10,A11);
          //voltageShorted = computeAvgVoltage(adcSum, adcCount);
          adcCount = 0;
          adcSum = 0;
          adcFlag = false;
          loopCount++;
          break;

        default: // do nothing in between pulses
          break;
      }
    }
  
    // measure voltage across load
    if (adcFlag){
      adcSum += adc->adc0->analogReadDifferential(A10,A11); // read current voltage
      adcCount++;
      delayMicroseconds(1);
    }
  
    if (loopCount == 1000){
      Serial.print("\nV+ = ");
      Serial.print(voltagePositive, 5);
      Serial.print(", V- = ");
      Serial.print(voltageNegative, 5);
      Serial.print(", V0 = ");
      Serial.print(voltageShorted, 5);
      Serial.print(", Vnet = ");
      Serial.println(voltagePositive - voltageShorted, 5);
  
      loopCount = 0;
    }
  }
}
