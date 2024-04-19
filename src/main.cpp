#include <Arduino.h>
#include <IRremote.hpp>
#include <math.h>
#include <time.h>
#define IR_RECEIVE_PIN 8

// UP = Forward (RC)
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

void resetM();
int* getRPM(volatile int, volatile int);
void RemoteCommand();
void hardbreak();
void fwd(); void bwd(); void tLeft(); void tRight();
void accelerate(byte , float, unsigned int);
void decelerate(byte,float, unsigned int);
void countPulseL();
void countPulseR();

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN,ENABLE_LED_FEEDBACK); 
  // in IRremote 4.0 we don't need to instantiate Receiver or sender object, it is already instantiate in the library
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(RPMpinRT, INPUT); // photointerrupter Module seems to have on board pull up resister
  pinMode(RPMpinLT, INPUT);
  resetM();

  //Left Counter
  attachInterrupt(digitalPinToInterrupt(RPMpinLT), countPulseL, RISING); // for Left Wheel
  //Right Counter
  attachInterrupt(digitalPinToInterrupt(RPMpinRT), countPulseR, RISING); // for Right wheel
}

/*Let's use timer interrupt here*/
// bool isbuttonPressed;
void loop() {

// Flag setting mehtod
// Let's use Timer Interrupt method

  //1. Decode IR Remote button
  //2. Execute Command Accordingly
  // RemoteCommand();
 

  // separate condition and buffer with small delay to allow the next decode to registered before falling into this condition
  // delay(100);

  
/*
but the counter needs to reset after we letting go of the button , so that the whole accel or decel process will starts from 0
which doesn't seems to work by this code , because somehow even in the middle of waiting for next IR pulse , 
the condition just met, if we were to resetMotor speed it would resets at some time during acceleration => which made it stop all time
*/
  digitalWrite(in2,1);
  digitalWrite(in3,1);
  analogWrite(ena,255);
  analogWrite(enb,255);

  /***** Data Acquisition of Sensors*****/
    Serial.print("Rev Left : ");  
    Serial.println(revLeft); 
    Serial.print("Rev Right : ");  
    Serial.println(revRight); 
    //RPM from optical counter (Poor accuracy but anyway)
    Serial.println(getRPM(revLeft , revRight)[0]);  
    Serial.println(getRPM(revLeft,  revRight)[1]);  
   
    delay(150);
    // ถือว่าใช้งานได้ละ , RPM ถูก reset ทุกครั้งที่ล้อหยุดหมุน
    /* Other sensors that I will add */

}

// Interrupt Service Routine
  void countPulseL() { 
    revLeft++; // Increment pulse count for attach interrupt void function
  }
  void countPulseR() {
    revRight++; // Increment pulse count for attach interrupt void function
  }

void resetM(){
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}


// To create a function that return an array -> we need to create a function that returns a pointer to the array instead
int* getRPM(volatile int pulseCountL , volatile int pulseCountR){
  unsigned long currentTime = millis();

  // Begin the measurement only if the duration from last measurement is more than one sec.
  if(currentTime- lastTime >= 1000){

    // detach interrupt , to stop counting from photo interrupter
    detachInterrupt(digitalPinToInterrupt(RPMpinLT));
    detachInterrupt(digitalPinToInterrupt(RPMpinRT));

    
    // Once the function return the pointer to 1st element of this array , all the local variable mem. will be delocated to save space
    // we can not dereference back the mem of 1st element of this array, anymore 
    // declare static array to prevent deallocation of this variable mem address
    static int array[2];
    array[0] = pulseCountL / 20.0 * (60000.0 / (currentTime - lastTime)); // first address (for Left wheel)
    array[1] = pulseCountR / 20.0 * (60000.0 / (currentTime - lastTime)); // second address (for Right wheel)
    if(currentTime - lastTime >= 500){
      revLeft = revRight = 0; // Reset the count
    }
    
    // the reset is too fast, not enogh time for reading
    attachInterrupt(digitalPinToInterrupt(RPMpinLT), countPulseL, RISING); // RISING from LOW to HIGH
    attachInterrupt(digitalPinToInterrupt(RPMpinRT), countPulseR, RISING);  
    lastTime = currentTime;
    // return rpm_arr;
    return array;
  }
  return 0 ;
  // if the RPM doesn't get calculated
}

//Remote Command function -> it uses switch case, which is evaluate at compile time ,, the value uses to check different case
// should be known and pretty much constant (Not a passed argument) , so yeah this will be void
void RemoteCommand() {
   if (IrReceiver.decode()) {
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case 0xF708FF00:  // Turn Left
        tLeft();
        break;
      case 0xE718FF00:  // Up -> fwd
        fwd();
        break;
      case 0xA55AFF00:  // Turn Right
        tRight();
        break;
      case 0xAD52FF00:  // Down -> bwd
        bwd();
        break;
      case 0xE31CFF00:  // Hard Break
        hardbreak();
        break;
      case 0:  // Handle 0 (no leading zero)
        counter += 3;
        accelerate(255, 0.02, counter);
        // Limit speed to exact 255 (100% Duty cycle) regardless of how much we accelerate , 
        // to simulate a car terminal speed & Acceleration being primary dictated by engine power & gear box
        break;
    }

    // Enable receiving of the next value or from next IR frame (Next Pulse set)
    IrReceiver.resume(); 
  } 
  
  // IF WE LET GO OF THE BUTTON
  if(!IrReceiver.decode()) {
    
    // decelerate(255,0.02,counter);
    // Serial.println("***********EXIT"); 
    // if(speedControl =! 0){    
    //   speedControl = 0;
    // } 
    return;
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
    speedControlB = 0.3 * speedControlA;
    // Left needs to rotate slower than Right
  }

  void tRight(){ // if Right is Motor B
    speedControlA = 0.3 * speedControlB;
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
    speedControlA = speedControlB = speedControl;  
  }

  // there needs to be a way to save the current speed (PWM duty we are in right now)

  //Deceleration (RC discharge)
  void decelerate(byte currentSpeed ,float konst, unsigned int count){ 
    // currentSpeed is there to decelerate down from that point
    byte speedControl = currentSpeed *(exp((-konst)*(count)));
    if (speedControl <= 0) {
        speedControl = 0;
      }
    speedControlA = speedControlB = speedControl;    
  }

