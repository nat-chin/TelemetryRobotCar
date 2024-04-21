/*
This branch is to figure out realistic Robot car movement system, and movement control system in my own way 
and I would suggest to change from IRremote controller to a more refined input (Like analog) , like Joy Stick module , Rotary encoder,
Maybe incorporate Radio , Serial Bluetooth , WiFi, LoRa , whatever IoT protocol to do wireless control

The control system should be like FPV drone or a car acceleration that will reset zero after letting go of the control , basically
no self-holding kind of stuff

ALL FUNCTION IS PRETTY MUCH UNFINISH , AND SHOULD BE REVIEW FIRST BEFORE EDIT

*/

// INCLUDE custom header file here (to separate and organize functionality)

// INCLUDE Libraries
#include <Arduino.h>
#include <IRremote.hpp> // Consider changing this one
#include <math.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>


#define IR_RECEIVE_PIN 8
//PWM control pin of l298n
#define ena 10 
#define enb 9
//Input enable pin of l298n
#define in1 7
#define in2 6
#define in3 5
#define in4 4
// Hardware Interrupt pin (attach to left & Right Photo interrupter)
#define RPMpinLT 2
#define RPMpinRT 3

// Problem :Tachometer can't read absolute 0 RPM due to motor stop or Stall, can be solved by TimerInterrupt (about 1.5-2 sec)
// But I would like to use method presented here: https://deepbluembedded.com/arduino-rpm-sensor-motor-rpm-meter-counter-encoder/
short speedControlA = 0;
short speedControlB = 0;


/*Flag and Counter*/
unsigned int pwmcounter = 0; // used as a parameter in motor acceleration function
volatile unsigned int pulseCountL, pulseCountR; // Count of pulses from both photo interrupter counter
volatile unsigned long lastPulseTime = 0; // Time of last pulse
volatile unsigned long lastIMUtime = 0;
// I think some part can implement namespace, but let's leave it for now

// MPU6050 data variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
//GPS data variables

//Declare function
int* tachoMeter(volatile int, volatile int); void RemoteCommand();
void resetM(); void hardbreak(); void fwd(); void bwd(); void tLeft(); void tRight();
void accelerate(short, float, unsigned int); void decelerate(short ,float, unsigned int);

/* ISR for hardware interrupt */
  void countPulseL() { pulseCountL++;} 
  void countPulseR() { pulseCountR++;}
  

/* ---ISR of Timer1 => can't use Servo Libray , and PWM pin 9 , 10--- */
ISR(TIMER1_COMPA_vect){  
  // OCR1A += 60000; // Advance The COMPA Register
  TCNT1 = 0;
  // Due to the fact that we are not enalbing it in setup , we will preload & Enable it in the loop right away
  decelerate(speedControlA,0.02, pwmcounter);
  decelerate(speedControlB,0.02 , pwmcounter);
}

// Sensors instance
MPU6050 mpu(Wire);

void setup() {
  /*Timer interrupt for NEC protocol IR remote (interrupt every 28-30 ms , which should be interval between each IR frame)
  but it will soon be changed to other control device*/
  
  TCCR1A = 0;           // Init Timer1A
  TCCR1B = 0;           // Init Timer1B
  TCCR1B |= B00000010;  // Prescaler = 8
  OCR1A = 0; // Preload this thing as zero first
  // Counter enable will be after the the control device is released

  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN,ENABLE_LED_FEEDBACK); 
  // in IRremote 4.0 we don't need to instantiate Receiver or sender object, it is already instantiate in the library

  Wire.begin(); // join I2C bus
  mpu.begin(); // initialize mpu instance

  Serial.println(F("Calculating gyro offset, so Keep MPU6050 normal to X Y Z axis"));
  delay(1000);
  mpu.calcOffsets(); // Calibrate both gyro and acc offset 
  Serial.println("Done: \n");

  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(RPMpinRT, INPUT); // photointerrupter Module seems to have on board pull up resister
  pinMode(RPMpinLT, INPUT);
  resetM();

  attachInterrupt(digitalPinToInterrupt(RPMpinLT), countPulseL, RISING); // for Left Wheel
  attachInterrupt(digitalPinToInterrupt(RPMpinRT), countPulseR, RISING); // for Right wheel
}


void loop() {
  //1. Decode IR Remote button
  //2. Execute Command Accordingly

  RemoteCommand(); // if absense of IR pulses, arduino Timer1 to count for 30 ms, decelerate , if has IR pulses don't decelerate
 
  digitalWrite(in2,1);
  digitalWrite(in3,1);
  // digitalWrite(ena,HIGH);
  // digitalWrite(enb,HIGH);

  // 2. Write the speedControl value to PWM pin , to control motor speed

  // Change ena & enb to other PWM pin because Timer1 is used 
  analogWrite(ena,speedControlA);
  analogWrite(enb, speedControlB);


  /***** Data Acquisition of Sensors *****/

  //1. Tachometer

    // Serial.print("Counter L: ");  
    // Serial.println(pulseCountL); 
    // Serial.print("Counter R Right : ");  
    // Serial.println(pulseCountR); 
    // //RPM from optical counter (Poor accuracy but anyway)
    // Serial.println(tachoMeter(pulseCountL , pulseCountR)[0]);  
    // Serial.println(tachoMeter(pulseCountL,  pulseCountR)[1]);  

  // 2. IMU & Temperature sensor

  mpu.update(); // update the memory register with new value

  if(millis() - lastIMUtime > 100){ // more than 10 milisec. interval to prevent monitor flooding
  mpu.getAccX();
  mpu.getAccY();
  mpu.getAccZ();
  
  mpu.getAngleX();
  mpu.getAngleY();
  mpu.getAngleZ();
  Serial.println();

  lastPulseTime = millis(); // reset timer variable

  }

  // 3. GPS module

  // 4. UltraSonic sensor (For obstacle avoidance)

  // 5. IR sensor Tracker (For Line Following & PID control purpose)

  // This one might only use Arduino , and has no Wireless comm involves

}


void resetM(){
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}


int* tachoMeter(volatile int revL , volatile int revR){
  
  /*Need to employ a more timer based method to Calculate RPM*/

  unsigned long currentTime = millis();
  // Begin the measurement about 1 sec after both wheels has spinned from the last measurement.
  if(currentTime- lastPulseTime >= 1000){

    /* Once the function return the pointer to 1st element of "array" , all local var mem of this funct. 
    will be delocated to save space , thus our returned pointer can't dereference any of that array element mem. address
    declare static array to prevent deallocation of this variable mem address */
    detachInterrupt(digitalPinToInterrupt(RPMpinLT));
    detachInterrupt(digitalPinToInterrupt(RPMpinRT));
    static int array[2];
    array[0] = (revL / 20.0)/2 * (60000.0 / (currentTime - lastPulseTime)); // first address (for Left wheel)
    array[1] = (revR / 20.0)/2 * (60000.0 / (currentTime - lastPulseTime)); // second address (for Right wheel)
    
    attachInterrupt(digitalPinToInterrupt(RPMpinLT), countPulseL, RISING);
    attachInterrupt(digitalPinToInterrupt(RPMpinRT), countPulseR, RISING);
    lastPulseTime = currentTime;
    return array;
  }
  return 0 ;
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
        pwmcounter += 3;
        accelerate(255, 0.02, pwmcounter);
        // Limit speed to exact 255 (100% Duty cycle) regardless of how much we accelerate , 
        // to simulate a car terminal speed & Acceleration being primary dictated by engine power & gear box
        break;
    }

    // Enable receiving of the next value or from next IR frame (Next Pulse set)
    IrReceiver.resume(); 
    OCR1A += 60000;
    TIMSK1 = 0;
    TCNT1 = 0;
  } 

  // IF WE LET GO OF THE BUTTON
  if(!IrReceiver.decode()) {
    OCR1A += 60000;
    TIMSK1 |= B00000010;
    TIMSK1 = 0;
    
    // decelerate(255,0.02,pwmcounter);
    // Serial.println("***********EXIT"); 
    
    return;
  }
  
}

/*** Movement function ***/

  void hardbreak(){
        speedControlA = 0;
        speedControlB = 0;
        pwmcounter = 0;
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


/*Problem Acceleration : passing currentspeed value as initial speed argument will just make acceleration spikes up chaotically 
  Problem Deceleration : using new Current Speed everytime will generate different brake Curve

  We need to find the some get around of this , because if the car already has initial velecity ,accelerate & decelerate
  without considering this bias isn't accurate
  
----------------------------------
  Oh , I don't need that anyway : Because of my control system revolve around incrementing pwmcounter, and use that to
  exponentiate my accel , and decel function , so my initial speed is already captured in the current PWM count
*/

  //Acceleration (RC charge)
  void accelerate(short max_accel,float konst, unsigned int count) {
    // initial speed is there to accelerate up from that point
    byte pwmAccel = max_accel *(1-exp((-konst)*(count)));
    if (pwmAccel >= max_accel) {
        pwmAccel = max_accel;
      }
    speedControlA = speedControlB = pwmAccel;  
  }

  // there needs to be a way to save the current speed (PWM duty we are in right now)

  //Deceleration (RC discharge)
  void decelerate(short max_decel ,float konst, unsigned int count){ 
    // currentSpeed is there to decelerate down from that point
    byte pwmDecel = max_decel *(exp((-konst)*(count)));
    if (pwmDecel <= 0) {
        pwmDecel = 0;
      }
    speedControlA = speedControlB = pwmDecel;    
  }

