#include <ADC.h>
#include "RingBuffer.h"
#include <IntervalTimer.h>
#include <Eigen.h>
#include <Eigen/Dense>

using namespace Eigen;

#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

const int debugPin = 14;
const int debug2Pin = 15;

const int SWP1 = 17;
const int SWN1 = 8;
const int SWP2 = 16;
const int SWN2 = 9;

const int FETdelay = 2; // [us] time to wait for MOSFET to fully turn on/off (prevents shoot-through)

long int loopTime = 0;

IntervalTimer timerPulse;
const int pulseTime = 80; // [us] time for each current pulse (+/-); total time for biphasic pulse is 2*pulseTime
const int interPulseDelay = 1; // multiples of pulseTime between pulse trains ([us] = pulseTime * interPulseDelay)
volatile bool timerFlag = false;
volatile int timerCount = 0;

IntervalTimer timerAdc; // timer for triggering ADC samples
const float adcTime = 7.5; // [us] time between each adc trigger
const int adcDelay = 0; // number of adc samples to discard before saving (while waiting to current pulse to stabalize)
const int nSamples = 7; // number of ADC samples to take during positive pulse
const int nPulses = 128; // number of pulse trains which will be sampled and averaged together for each single output measurement
const float filterSigma = 2.5; // samples more than this many std deviations from the mean will be filtered out
const float filterVariance = filterSigma * filterSigma; // variance is actually used since it is faster to compute 
const int nSamplesShorted = 64; // number of samples to average for shorted voltage

ADC *adc = new ADC(); // ADC object
RingBuffer *adcRingBuffer = new RingBuffer; // buffer to capture adc samples (changed size in .h to 16 elements)
double vShorted = 0.0; // voltage across load when shorted
MatrixXi adcRaw(nSamples, nPulses); // stores raw (integer) values from adc
volatile int adcCount = 0; // current sample number (row index of adcRaw)
volatile int pulseCount = 0; // current pulse number (column index of adcRaw)
bool adcFlag = false;
double adc2Voltage = 0.0;

MatrixXd Alinfit(nSamples, 2); // linear regression matrix for line fitting
double resistance = 0.0; // resistive component of measured impedance
double capacitance = 0.0;// capacitive component of measured impedance

volatile bool runFlag = false;
const int buttonPin = 3; // momentary tactile button

char cmd = 0;

int loopCount = 0;

//********************************************************************
// STATE MACHINE
//********************************************************************

enum fsmState {statePowerUp, stateNotRunning, statePulsePositive, statePulseNegative, stateInterPulse, stateComputeZ};
fsmState stateNext;
fsmState stateCurrent;

//***********************
// ****** PowerUp *******

fsmState PowerUp(void) {
  stateCurrent = statePowerUp;
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("State: PowerUp");
  
  // set up control pins for MOSFET switches
  pinMode(SWP1, OUTPUT);
  pinMode(SWN1, OUTPUT);
  pinMode(SWP2, OUTPUT);
  pinMode(SWN2, OUTPUT);
  setOutputOff(); // sets all to LOW

  pinMode (debugPin, OUTPUT);
  pinMode (debug2Pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT);
  attachInterrupt(buttonPin, button_isr, RISING);
  
  // set up ADC
  pinMode(A10, INPUT); // Diff Channel 0 +
  pinMode(A11, INPUT); // Diff Channel 0 -
  adc->setAveraging(0); // no averaging; take single samples
  adc->setResolution(16); // 13 bit resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // sets ADCK to highest speed within spec for all resolutions
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // HIGH_SPEED adds +6 ADCK; MED_SPEED adds +10 ADCK
//  adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // use 1.2V internal reference
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0); // use 3.3V internal reference
  adc->adc0->analogReadDifferential(A10,A11); // call once to setup
//  adc2Voltage = 3.3/adc->getPGA()/adc->getMaxValue(); // conversion factor for adc values
  adc2Voltage = 3.3/32768.0; // conversion factor for 16-bit adc values
//  adc2Voltage = 1.2/32768.0; // conversion factor for 16-bit adc values
  adc->enableInterrupts(ADC_0);
  
  Serial.print("adc2Voltage = ");
  Serial.println(adc2Voltage, 6);
  
  // set up linear regression matrix
  Alinfit.col(0) = VectorXd::Ones(nSamples);
  Alinfit.col(1) = VectorXd::LinSpaced(nSamples, 1+adcDelay, nSamples+adcDelay);
//  print_mtxf(Alinfit);

  Serial.println("\nInitialization Complete");
 
  return stateNotRunning;
}

//*************************
// ****** NotRunning ******

fsmState NotRunning(void) {
  stateCurrent = stateNotRunning;

  // ensure outputs are shorted to zero voltage
  setOutputShorted();

  // stop IntervalTimers
  timerPulse.end();
  timerAdc.end();

  // reset counts
  pulseCount = 0;
  timerCount = 0;

  Serial.println("\nState: NotRunning\n");
  
  Serial.print("+100 uA pulse for ");
  Serial.print(pulseTime);
  Serial.print(" us, followed immediately by -100 uA pulse for ");
  Serial.print(pulseTime);
  Serial.println(" us");
  Serial.print("Inter-pulse delay is ");
  Serial.print(interPulseDelay * pulseTime);
  Serial.println(" us\n");
  Serial.println("\nType 's' or press button to start pulses");

  while ((cmd != 's') && (!runFlag)) {
    if (Serial.available() > 0) {
      cmd = Serial.read();
    }
  }

  runFlag = true;
    
  Serial.println("\nStarting Pulses");
  
  return statePulsePositive;
}

//*****************************
// ****** PulsePositive *******

fsmState PulsePositive(void) {
  stateCurrent = statePulsePositive;

  // reset adcCount
  adcCount = 0;

  // prepare for positive pulse by ensuring N2/P2 are off
  digitalWriteFast(SWN2, LOW);
  digitalWriteFast(SWP2, LOW);
  delayMicroseconds(FETdelay); // brief delay to ensure fully off
  
  // begin pulse IntervalTimer
  timerCount = 0;
  timerPulse.begin(timerPulse_isr, pulseTime);

  // wait for first pulse to sync up timing
  while (timerCount == 0);

  // start positive pulse (current from P1 to N1)
  digitalWriteFast(SWN1, HIGH);
  digitalWriteFast(SWP1, HIGH); // takes ~4 us to turn on and establish current 

  // start triggering ADC
  // takes around (1.6 + adcTime + 2.4) [us] to set up IntervalTimer,  wait for first trigger, and start first adc measurement
  timerAdc.begin(timerAdc_isr, adcTime); 
  
  return statePulseNegative;
}

//*****************************
// ****** PulseNegative *******

fsmState PulseNegative(void) {
  stateCurrent = statePulseNegative;

  // wait for positive pulse to finish
  while (timerCount < 2) {
    // store adc samples
    if (!adcRingBuffer->isEmpty()) {
      adcCount++; // increment count
      
      noInterrupts(); // ensure ring buffer isn't modified during read
      int adcLast = adcRingBuffer->read(); // read sample from buffer
      interrupts();
      
      if (adcCount > (nSamples+adcDelay)) {
        timerAdc.end(); // halt triggering
      }
      else if (adcCount > adcDelay) {
        digitalWriteFast(debug2Pin, HIGH);
        adcRaw(adcCount-adcDelay-1, pulseCount) = adcLast; // store
        digitalWriteFast(debug2Pin, LOW);
      }  
    }
  }

  // start negative pulse (current from P2 to N2)
  digitalWriteFast(SWP1, LOW); // turn off P1 first to drain charge from REF200
  delayMicroseconds(FETdelay);
  digitalWriteFast(SWN1, LOW); 
  delayMicroseconds(FETdelay/2); // brief delay to ensure fully off
  digitalWriteFast(SWN2, HIGH); // turn on N2 before P2
  digitalWriteFast(SWP2, HIGH);
  
  // ensure ADC triggering has been stopped
  timerAdc.end();

  // ensure buffer is cleared
  while(!adcRingBuffer->isEmpty()) {
    digitalWriteFast(debug2Pin, HIGH);
    adcRingBuffer->read();
    digitalWriteFast(debug2Pin, LOW);
  }
  
  // increment pulse count
  pulseCount++;

  return stateInterPulse;
}

//*****************************
// ****** InterPulse **********

fsmState InterPulse(void) {
  stateCurrent = stateInterPulse;
  
  // wait for negative pulse to finish
  while (timerCount < 3);

  // short load through N1/N2 to zero the voltage
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
  delayMicroseconds(FETdelay);
  digitalWriteFast(SWN1, HIGH);
  digitalWriteFast(SWN2, HIGH);

//  // after brief delay, turn off all
//  delayMicroseconds(20);
//  digitalWriteFast(SWN1, LOW);
//  digitalWriteFast(SWN2, LOW);
  
  // check whether this is the last pulse
  fsmState next = (pulseCount < nPulses ? statePulsePositive : stateComputeZ);

  // wait for interPulseDelay to finish
  while (timerCount < (3+interPulseDelay));

  return next;
}

//****************************
// ******** ComputeZ *********

fsmState ComputeZ(void) {
  stateCurrent = stateComputeZ;
  
  // stop IntervalTimer
  timerPulse.end();

  // reset pulse count
  pulseCount = 0;
  
  // average together samples taken at same time during pulses (i.e. rows of adcRaw) and convert to voltage
  VectorXd adcMean(nSamples);
  adcMean = adc2Voltage * adcRaw.rowwise().mean().cast<double>();

  // measure shorted voltage
  double adcBufferShorted = 0.0;
  adcCount = 0;
  timerAdc.begin(timerAdc_isr, adcTime);
  while (adcCount < nSamplesShorted) 
  {
    if (!adcRingBuffer->isEmpty()) {
      adcCount++;
      adcBufferShorted += (adc2Voltage * adcRingBuffer->read());
    }
  }
  vShorted = adcBufferShorted / (double)nSamplesShorted; // compute average
//  Serial.print("Vshorted = ");
//  Serial.print(vShorted, 5);
//  Serial.println(" V");

  // subtract shorted voltage
  adcMean = adcMean - (VectorXd::Ones(nSamples)*vShorted);
  
  // stop ADC triggering and empty buffer
  timerAdc.end();
  while(!adcRingBuffer->isEmpty()) {
    adcRingBuffer->read();
  }
    
  
  
  // fit least-squares line to data
  Vector2d fit;
  long int startTime = micros();
//  fit = Alinfit.householderQr().solve(adcMean);
  fit = Alinfit.colPivHouseholderQr().solve(adcMean);
  long int totalTime = micros() - startTime;
  
  // compute resistive component (via intercept of fit)
  // I = 100E-6 [A] ==> 1/I = 1E4 [A]
  resistance = fit(0) * 1.0E4;
  
  // compute capacitive component (via slope of fit)
  // C[nF] = (0.1[ma] * adcTime[us/sample]) / (slope[V/sample])
  capacitance = 0.1 * (double)adcTime / fit(1);

  long int totalLoopTime = micros() - loopTime;
  
  // print results
//  Serial.print("Linear Fit Computation Time = ");
//  Serial.print(totalTime);
//  Serial.println(" us");
//  Serial.print("Loop Time = ");
//  Serial.print(totalLoopTime);
//  Serial.println(" us");
//  Serial.print("R = ");
//  Serial.print(resistance);
//  Serial.print(" ohms,  C = ");
//  if (capacitance > 500) {
//    // too large to cause enough voltage rise over the pulse length
//    Serial.println(" N/A");
//  }
//  else {
//    Serial.print(capacitance);
//    Serial.println(" nF");
//  }

//  Serial.print(vShorted,6);
//  Serial.print(",");

  Serial.print(resistance);
  Serial.print(",");
  if ((capacitance > 500)||(capacitance<0)) {
    // too large to cause enough voltage rise over the pulse length
    Serial.println("0");
  }
  else {
    Serial.println(capacitance,4);
  }
  
//  Serial.println(fit(1), 7);

//  MatrixXf adcV(nSamples, nPulses);
//  adcV = adc2Voltage * adcRaw.cast<float>();
//print_mtxf(adcMean.cast<float>());
//  delay(1000);

  loopTime = micros();
  
  // begin next pulse train
  return statePulsePositive;
}

//********************************
// ****** StepStateMachine *******

fsmState stepStateMachine(fsmState stateNext) 
{
  switch (stateNext)
  {
  case statePowerUp:
    return PowerUp();
    
  case statePulsePositive:
    return PulsePositive();

  case statePulseNegative:
    return PulseNegative();

  case stateInterPulse:
    return InterPulse();

  case stateNotRunning:
    return NotRunning();

  case stateComputeZ:
    return ComputeZ();

  default:
    Serial.println("Error: Unrecognized State");
        while (1) {
          digitalWriteFast(LED_BUILTIN, HIGH);
          delay(500);
          digitalWriteFast(LED_BUILTIN, LOW);
          delay(500);
        }
  }
}

//************************************
//**** Interrupt Service Routines ****
//************************************

void timerPulse_isr() { // IntervalTimer
  // increment counter
  timerCount++;
}

void timerAdc_isr() {
  digitalWriteFast(debugPin, HIGH);
  adc->adc0->startSingleDifferential(A10,A11);
  digitalWriteFast(debugPin, LOW);
}

// called as soon as adc measurement is ready
void adc0_isr() {
  digitalWriteFast(debugPin, HIGH);
  adcRingBuffer->write(adc->adc0->readSingle());
  digitalWriteFast(debugPin, LOW);
}

void button_isr() {
  static unsigned long debounceTime = 0;
  unsigned long currentTime = millis();
  
  // If interrupts come faster than N ms, assume it's a bounce and ignore
  if (currentTime - debounceTime > 250) {
    runFlag = !runFlag;

    if (!runFlag) {
      WRITE_RESTART(0x5FA0004);
    }
  }
  
  debounceTime = currentTime;
}

//************************************
//********* Other Functions **********

void setOutputOff() {
  // all gates LOW
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
  delayMicroseconds(FETdelay);
  digitalWriteFast(SWN1, LOW);
  digitalWriteFast(SWN2, LOW);
}

void setOutputPositive() {
  // Allow positive current from P1 to N1
  digitalWriteFast(SWN2, LOW); // ensure off
  digitalWriteFast(SWP2, LOW);
  delayMicroseconds(FETdelay); // brief delay to ensure fully off
  digitalWriteFast(SWN1, HIGH);
  digitalWriteFast(SWP1, HIGH);
}

void setOutputNegative() {
  // Allow positive current from P2 to N2
  digitalWriteFast(SWP1, LOW); // turn off P1 first to drain charge from REF200
  delayMicroseconds(4);
  digitalWriteFast(SWN1, LOW); 
  delayMicroseconds(FETdelay); // brief delay to ensure fully off
  digitalWriteFast(SWN2, HIGH); // turn on N2 before P2
  digitalWriteFast(SWP2, HIGH);
}

void setOutputShorted() {
  // Short load through N1/N2
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
  delayMicroseconds(FETdelay);
  digitalWriteFast(SWN1, HIGH);
  digitalWriteFast(SWN2, HIGH); 
}

// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

//   Serial.print("nrow: "); Serial.println(nrow);
//   Serial.print("ncol: "); Serial.println(ncol);       
//   Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

// PRINT MATRIX (double type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXd& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

//   Serial.print("nrow: "); Serial.println(nrow);
//   Serial.print("ncol: "); Serial.println(ncol);       
//   Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

//********************************************************************
// SETUP
//********************************************************************

void setup() {
  // initialize finite state machine
  stateNext = stepStateMachine(statePowerUp);
}

//********************************************************************
// MAIN LOOP
//********************************************************************

void loop() {
  // run state machine
  stateNext = stepStateMachine(stateNext);
}
