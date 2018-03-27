#include <ADC.h>
#include <IntervalTimer.h>
#include <Eigen.h>
#include <Eigen/Dense>

using namespace Eigen;

const int debugPin = 14;

const int SWP1 = 17;
const int SWN1 = 8;
const int SWP2 = 16;
const int SWN2 = 9;

const int pulseTime = 75; // [us] time for each current pulse (+/-); total time for biphasic pulse is 2*pulseTime
const int interPulseDelay = 4; // multiples of pulseTime between pulses ([us] = pulseTime * interPulseDelay)
IntervalTimer timerPulse;
volatile bool timerFlag = false;
volatile int timerCount = 0;

const float adcTime = 5.0; // [us] time between each adc trigger
const float adcFrequency = 1.0E6/adcTime; // [Hz] frequency used by pdb to trigger adc samples
const int nSamples = 8; // number of ADC samples to take during positive pulse
const int nPulses = 256; // number of pulse trains which will be sampled and averaged together for each single output measurement
const float filterSigma = 2.5; // samples more than this many std deviations from the mean will be filtered out
const float filterVariance = filterSigma * filterSigma; // variance is actually used since it is faster to compute 
const int nSamplesShorted = 16; // number of samples to average for shorted voltage

ADC *adc = new ADC(); // ADC object
float adcBufferShorted; // ADC buffer for samples when measuring shorted voltage
float vShorted = 0.0; // voltage across load when shorted
MatrixXi adcBuffer(nSamples, nPulses); // ADC buffer for storing raw (integer) values
volatile int adcCount = 0; // current sample number (row index of adcBuffer)
volatile int pulseCount = 0; // current pulse number (column index of adcBuffer)
bool adcFlag = false;
double adc2Voltage = 0.0;

MatrixXf Alinfit(nSamples, 2); // linear regression matrix for line fitting
float resistance = 0.0; // resistive component of measured impedance
float capacitance = 0.0;// capacitive component of measured impedance

volatile bool buttonFlag = false;
const int buttonPin = 3; // momentary tactile button

volatile bool runFlag = false;

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
  delay(500);
  Serial.println("State: PowerUp");
  
  // set up control pins for MOSFET switches
  pinMode(SWP1, OUTPUT);
  pinMode(SWN1, OUTPUT);
  pinMode(SWP2, OUTPUT);
  pinMode(SWN2, OUTPUT);
  setOutputOff(); // sets all to LOW

  // set up ADC
  pinMode(A10, INPUT); // Diff Channel 0 +
  pinMode(A11, INPUT); // Diff Channel 0 -
  adc->setAveraging(1); // no averaging; take single samples
  adc->setResolution(16); // 16 bit resolution
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // HIGH_SPEED adds +6 ADCK; MED_SPEED adds +10 ADCK
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // sets ADCK to highest speed within spec for all resolutions
  adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2
  adc2Voltage = 1.2/adc->getPGA()/adc->getMaxValue(); // conversion factor for adc values
  adc->adc0->analogReadDifferential(A10,A11); // call once to setup
  adc->enableInterrupts(ADC_0); // it's necessary to enable interrupts for PDB to work

  // set up linear regression matrix
  Alinfit.col(0) = VectorXf::Ones(nSamples);
  Alinfit.col(1) = VectorXf::LinSpaced(nSamples, 0, nSamples-1);

  pinMode (debugPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT);
  attachInterrupt(buttonPin, button_isr, RISING);

  Serial.println("\nInitialization Complete");
 
  return stateNotRunning;
}

//*************************
// ****** NotRunning ******

fsmState NotRunning(void) {
  stateCurrent = stateNotRunning;

  // ensure outputs are shorted to zero voltage
  setOutputShorted();

  // stop pdb
  adc->adc0->stopPDB();

  // stop IntervalTimer
  timerPulse.end();

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

  while ((cmd != 's') && (!buttonFlag)) {
    if (Serial.available() > 0) {
      cmd = Serial.read();
    }
  }

  buttonFlag = false;
    
  Serial.println("\nStarting Pulses");
  
  return statePulsePositive;
}

//*****************************
// ****** PulsePositive *******

fsmState PulsePositive(void) {
  stateCurrent = statePulsePositive;
    
  // measure shorted voltage
//  for (int ii=0; ii<nSamplesShorted; ii++){
//    adcBufferShorted(ii) = adc->adc0->analogReadDifferential(A10,A11);
//  }
//  vShorted = adc2Voltage * (float)adcBufferShorted.mean();
  
  // reset adcCount
  adcCount = 0;
  
  // begin IntervalTimer
  timerCount = 0;
  timerPulse.begin(pulse_isr, pulseTime);

  // wait for first pulse to sync up timing
  while (timerCount == 0);

  // start positive pulse
  setOutputPositive();

  // start pdb for triggering ADC
  adc->adc0->startPDB(adcFrequency);
  
  return statePulseNegative;
}

//*****************************
// ****** PulseNegative *******

fsmState PulseNegative(void) {
  stateCurrent = statePulseNegative;

  // wait for positive pulse to finish
  while (timerCount < 2);

  // stop ADC
  adc->adc0->stopPDB();
  
  // start negative pulse
  setOutputNegative();

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

  // short to zero voltage across load
  setOutputShorted();

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

  // measure shorted voltage
  adc->disableInterrupts(ADC_0);
  adcBufferShorted = 0.0;
  for (int ii=0; ii<nSamplesShorted; ii++){
    adcBufferShorted += (adc2Voltage * adc->adc0->analogReadDifferential(A10,A11));
  }
  vShorted = adcBufferShorted / (float)nSamplesShorted;
  adc->enableInterrupts(ADC_0);
  
  Serial.print("Vshorted = ");
  Serial.println(vShorted, 7);
  
  // average together samples taken at same time during pulses (i.e. rows of adcBuffer) and convert to voltage
  VectorXf adcMean(nSamples);
  adcMean = adc2Voltage * adcBuffer.rowwise().mean().cast<float>();

  // fit least-squares line to data
  Vector2f fit;
  fit = Alinfit.householderQr().solve(adcMean);

  // compute resistive component (via intercept of fit)
  // I = 100E-6 [A] ==> 1/I = 1E4 [A]
//  resistance = fit(0) * 1.0E4;
  resistance = adcMean(0) * 1.0E4;
  
  // compute capacitive component (via slope of fit)
  // C[nF] = (0.1[ma] * adcTime[us/sample]) / (slope[V/sample])
  capacitance = 0.1 * adcTime / fit(1);

  // print results
  Serial.print("\nV = ");
//  Serial.print(resistance);
  Serial.print(adcMean(0), 6);
  Serial.print(" V,  C = ");
//  Serial.print(" ohms,  C = ");
//  Serial.print(capacitance);
  Serial.print(fit(1));
  Serial.println(" nF");

//  MatrixXf adcV(nSamples, nPulses);
//  adcV = adc2Voltage * adcBuffer.cast<float>();
  print_mtxf(adcBuffer);
//  delay(1000);
  
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

void pulse_isr() { // IntervalTimer
  // increment counter
  timerCount++;
}

void button_isr() {
  static unsigned long debounceTime = 0;
  unsigned long currentTime = millis();
  
  // If interrupts come faster than N ms, assume it's a bounce and ignore
  if (currentTime - debounceTime > 250) {
    buttonFlag = true;
  }
  
  debounceTime = currentTime;
}

void adc0_isr() {
  if(adc->adc0->fail_flag) {
    adc->adc0->stopPDB();
    Serial.print("ADC0 error flags: 0x");
    Serial.println(adc->adc0->fail_flag, HEX);
    if(adc->adc0->fail_flag == ADC_ERROR_COMPARISON) {
      adc->adc0->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
      Serial.println("Comparison error in ADC0");
    }
    while(1){
      digitalWriteFast(LED_BUILTIN, HIGH);
      delay(500);
      digitalWriteFast(LED_BUILTIN, LOW);
      delay(500);
    }
  }
  
  if(adcCount < nSamples) {
    // reads value and clears interrupt
    adcBuffer(adcCount, pulseCount) = adc->adc0->readSingle();
    adcCount++; // increment count
  }
  else{
    adc->adc0->stopPDB(); // stop pdb once desired number of samples have been taken
    adc->adc0->readSingle(); // throw away sample and clear interrupt
  }

  digitalWriteFast(debugPin, LOW);
}

void pdb_isr(void) { // triggers ADC sample
  PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt
  digitalWriteFast(debugPin, HIGH);
}

//************************************
//********* Other Functions **********

void setOutputOff() {
  // all gates LOW
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
  digitalWriteFast(SWN1, LOW);
  digitalWriteFast(SWN2, LOW);
}

void setOutputPositive() {
  // Allow positive current from P1 to N1
  digitalWriteFast(SWN2, LOW); // ensure off
  digitalWriteFast(SWP2, LOW);
  digitalWriteFast(SWN1, HIGH); // turn on N1 before P1
  digitalWriteFast(SWP1, HIGH);
}

void setOutputNegative() {
  // Allow positive current from P2 to N2
  digitalWriteFast(SWN1, LOW); // ensure off
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWN2, HIGH); // turn on N2 before P2
  digitalWriteFast(SWP2, HIGH);
}

void setOutputShorted() {
  // Short load through N1/N2
  digitalWriteFast(SWN1, HIGH); // ensure off
  digitalWriteFast(SWN2, HIGH);
  digitalWriteFast(SWP1, LOW);
  digitalWriteFast(SWP2, LOW);
}

// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   
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
  
//  if(buttonFlag) {
//    buttonFlag = false;
//    timerFlag = false;
//    timerCount = 0;
//    runFlag = !runFlag;
//    digitalWriteFast(LED_BUILTIN, runFlag);
//    delay(300);
//    Serial.print("\n\nrunFlag = ");
//    Serial.println(runFlag);
//  }
}
