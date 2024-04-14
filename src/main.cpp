#include <Arduino.h>

#include <IRremote.hpp>
#include <math.h>
#define IR_RECEIVE_PIN 8

// UP = Forward ()
// OK = Brake = Hard Deceleration (reverse RC )
// DOWN = Backward
// LEFT RIGHT = Steer (We don't have steering system , so we can only employ unequating two motor speed)
// holding any button = Acceleration (Modeled by RC charge equation)
// Let the button go = Deceleration (Modeled by RC discharge equation)

byte ena = 10; // PWM
byte in1 = 7;
byte in2 = 6;
byte in3 = 5;
byte in4 = 4;
byte enb = 9; // PWM

byte speedControlA = 0;
byte speedControlB = 0;
unsigned int counter = 0; 
// to count pass 255 , because my acceleeation function need an ample size of input counter to reach max speedControl
 
#define RPMpinLT 2
#define RPMpinRT 3

volatile int revLeft, revRight = 0; // Count of pulses from left counter
volatile unsigned long lastTime = 0; // Time of last pulse

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN,ENABLE_LED_FEEDBACK); // Start the receiver
  // (is this a class variable? , I don't even need to )

  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(RPMpinRT, INPUT); // RPM counter Module already has pull up resistor
  pinMode(RPMpinLT, INPUT);
  resetM();

  //Left Counter
  attachInterrupt(digitalPinToInterrupt(RPMpinLT), countPulseL, RISING); // for Left Wheel
  //Right Counter
  attachInterrupt(digitalPinToInterrupt(RPMpinRT), countPulseR, RISING); // for Right wheel
}

void loop() {

  //1. Decode IR Remote button
  
  if (IrReceiver.decode()) {
    //2. Execute Command Accordingly
    RemoteCommand( IrReceiver.decodedIRData.decodedRawData ); // Listen to Remote Command and execute function
    
    IrReceiver.resume(); // Enable receiving of the next value or from next IR frame (Next Pulse set)

  } 
  delayMicroseconds(10000); // separate condition and buffer with small delay to allow the next decode to registered before falling into this condition
  if(!IrReceiver.decode()) {
    // IF WE LET GO OF THE BUTTON
    // decelerate()
    Serial.println("***********EXIT"); 
    // if(speedControl =! 0){    
    //   speedControl = 0;
    // } 
    return;
  }
  /*
  but the counter needs to reset after we letting go of the button , so that the whole accel or decel process will starts from 0
  which doesn't seems to work by this code , because somehow even in the middle of waiting for next IR pulse , 
  the condition just fucking met, and this shit resets all the time -> fuck 
  */

  /***** Data Acquisition of Sensors*****/

  //RPM from optical counter (Poor accuracy but anyway)
  // int leftWheelRPM = getRPM(revLeft , revRight)[0];
  // int rightWheelRPM = getRPM(revLeft,revRight)[1];
  // Other sensors that I will add

}

// Interrupt Service Routine
  void countPulseL() { 
    revLeft++; // Increment pulse count for attach interrupt void function
  }
  void countPulseR() {
    revRight++; // Increment pulse count for attach interrupt void function
  }

//

void resetM(){
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}

int getRPM(volatile int pulseCountL , volatile int pulseCountR)[2]{
  unsigned long currentTime = millis();
  // keep current time millis() after MCU is active in currentTime
  if(currentTime- lastTime >= 1000){
    detachInterrupt(digitalPinToInterrupt(RPMpinLT));
    detachInterrupt(digitalPinToInterrupt(RPMpinRT));
    int rpmL = (pulseCountL / 20.0) * (60000.0 / (currentTime - lastTime));
    int rpmR = (pulseCountR / 20.0) * (60000.0 / (currentTime - lastTime));
    revLeft,revRight = 0; // Reset the count
    attachInterrupt(digitalPinToInterrupt(RPMpinLT), countPulseL, RISING); // RISING from LOW to HIGH
    attachInterrupt(digitalPinToInterrupt(RPMpinRT), countPulseR, RISING);  
    lastTime = currentTime;
    int array[2] = {rpmL,rpmR};
    return array;
  }
}

void RemoteCommand(int IRrawdata){
  switch(IRrawdata){
    case 0xF708FF00:
      // Turn Left
      tLeft();
      break; 
    case 0xE718FF00:
      // Up -> fwd
      fwd();
      break;
    case 0xA55AFF00:
      // Turn Right
      tRight();
      break;
    case 0xAD52FF00:
      // Down -> bwd
      bwd();
      break;
    case 0xE31CFF00:
      hardbreak();
      break;
    case 0: 
      counter += 3;
      accelerate();
      // Limit speed to exact 255 (100% Duty cycle) regardless of how much we accelerate , 
      // to simulate a car terminal speed & Acceleration being primary dictated by engine power & gear box
      break;
  }
} 

/*** Movement function ***/

  void hardbreak(){
        // Serial.println("Reset Speed & Counter");
        speedControlA = 0;
        speedControlB = 0;
        counter = 0;
  }

  void fwd(){
    resetM();
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);

  }

  void bwd(){
    resetM();
    digitalWrite(in1,HIGH);
    digitalWrite(in4,HIGH);
  }

  void tLeft(){ // if Left is Motor A 
    speedControlB = 0.3 * SpeedControlA;
    // Left needs to rotate slower than Right
  }

  void tRight(){ // if Right is Motor B
    speedControlA = 0.3 * SpeedControlB;
    // Left needs to rotate slower than Right
  }
  // This can potentially leads to chained speed reduction if we concecutively alternate left and right
  // but I think this behavior is similar to how a real car would response to an extream steering as well.


  //Acceleration (RC charge)
  void accelerate(byte initialSpeed , float konst, unsigned int count) {
    // initial speed is there to accelerate up from that point
    byte speedControl = 255 *(1-exp((-konst)*(count))) + initialSpeed;
    if (speedControl >= 255) {
        speedControl = 255;
      }
    speedControlA , speedControlB = speedControl;    
  }


  //Deceleration (RC discharge)
  void decelerate(byte currentSpeed ,float konst, unsigned int count){ 
    // currentSpeed is there to decelerate down from that point
    byte speedControl = currentSpeed *(exp((-konst)*(count)));
    if (speedControl <= 0) {
        speedControl = 0;
      }
    speedControlA , speedControlB = speedControl;    
  }

