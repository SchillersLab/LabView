#include <MultiStepper.h>
#include <AccelStepper.h>
#include <Stepper.h>
#include <Servo.h> 


#define SERVO0 3
#define SERVO1 4
#define SERVO2 5
#define SERVO3 6

#define STEPPER0_DIR  7 
#define STEPPER0_STEP 8
#define STEPPER0_NEN  9
#define STEPPER1_DIR  10
#define STEPPER1_STEP 11
#define STEPPER1_NEN  12
#define STEPPER2_DIR  24
#define STEPPER2_STEP 25
#define STEPPER2_NEN  26

#define GPIO0 29
#define GPIO1 30
#define GPIO2 31
#define GPIO3 32
#define GPIO4 35
#define GPIO5 36
#define DAC0 A21
#define DAC1 A22

#define DCMOTOR_PWMA 14
#define DCMOTOR_AI1  16
#define DCMOTOR_AI2  15
#define DCMOTOR_BI1  18
#define DCMOTOR_BI2  19
#define DCMOTOR_PWMB 20
#define DCMOTOR_STBY 17

#define RELAY0 39
#define RELAY1 38
#define RELAY2 37

#define LED0 21
#define LED1 22

AccelStepper stepper2(1, STEPPER2_STEP, STEPPER2_DIR); 


volatile unsigned long frameNum; 
long lastSentFrameNum;
const int resetTime = 2000;    // ms
elapsedMicros t;
elapsedMillis tLastNewFrame;

IntervalTimer IT;

char comBuf[100] = {0};
int nChar;

int stepper2Pos;
bool stepper2AtPos;

bool valveState, MotorState;
long timeToTurnOffValve, timeToTurnOffMotor;

int tLed, tGate, optoWindowStart, optoWindowEnd; 

bool Reset;
double angle, counter;
int aState, bState, aLastState, a, b;
int loopIter;




void newFrame(){
  frameNum++;
  tLastNewFrame = 0;

  if (frameNum >= optoWindowStart && frameNum <= optoWindowEnd){
      pulseOpto();
  }
}

void pulseOpto(){
   if (tLed != 0){
    digitalWrite(GPIO0, HIGH); // start gating
    digitalWrite(GPIO3, HIGH); // turn on LED
    
    long startTime = micros();
    while (micros() - startTime < tLed)
      stepper2.run();
    digitalWrite(GPIO3, LOW); // turn off LED
    while (micros() - startTime < tLed + tGate)
      stepper2.run();
    digitalWrite(GPIO0, LOW); // turn off gating
  }
}

void setup() {
  
  pinMode(STEPPER0_DIR, OUTPUT);
  pinMode(STEPPER0_STEP, OUTPUT);
  pinMode(STEPPER0_NEN, OUTPUT);
  pinMode(STEPPER1_DIR, OUTPUT);
  pinMode(STEPPER1_STEP, OUTPUT);
  pinMode(STEPPER1_NEN, OUTPUT);
  pinMode(STEPPER2_DIR, OUTPUT);
  pinMode(STEPPER2_STEP, OUTPUT);
  pinMode(STEPPER2_NEN, OUTPUT);  
  pinMode(RELAY0, OUTPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);  
  pinMode(LED0, OUTPUT);  
  pinMode(LED1, OUTPUT);  
 
  digitalWrite(STEPPER0_NEN, HIGH);  
  digitalWrite(STEPPER1_NEN, HIGH);  
  digitalWrite(STEPPER2_NEN, HIGH);


  pinMode(GPIO0, OUTPUT);
  pinMode(GPIO1, INPUT);
  pinMode(GPIO2, OUTPUT);
  pinMode(GPIO3, OUTPUT);
  pinMode(GPIO4, INPUT);
  pinMode(GPIO5, INPUT);
  pinMode(DAC0, OUTPUT);
  pinMode(DAC1, OUTPUT);
  

  stepper2.setMaxSpeed(3500);
  stepper2.setAcceleration(7000);
  stepper2Pos = 0;
  stepper2AtPos = true;

  valveState = false;
  timeToTurnOffValve = millis();

  MotorState = false;
  timeToTurnOffMotor = millis();

  Serial.begin(9600);
  attachInterrupt(GPIO1, newFrame, RISING);

  
  frameNum=0;
  lastSentFrameNum=0;
  t=0;
  tLed = 0;
  tGate = 0;
  optoWindowStart = 0;
  optoWindowEnd = 0;
  angle = 0; 
  counter = 0;
  aState = 0; 
  bState = 0;
  a = 0; 
  b = 0;
  aLastState = 0;
  loopIter = 0;


 analogWrite(DAC0, 0);
  
  delay(2000);
  Serial.println("Booting up!");
  Serial.flush();
}


void loop() {
  if (Serial.available()){
    comBuf[nChar] = Serial.read();
    if (comBuf[nChar] == '\n'){   // we've received a complete command
      comBuf[nChar] = '\0';       // terminate string
      if (comBuf[0] == 'S'){      // it's a "set-speed" command
        int s, sp;
        sscanf(comBuf, "S%d,%d", &s, &sp);
        analogWrite(DCMOTOR_PWMA, abs(sp));
        digitalWrite(DCMOTOR_AI1, (sp>0)?HIGH:LOW);
        digitalWrite(DCMOTOR_AI2, (sp>0)?LOW:HIGH);
        digitalWrite(DCMOTOR_STBY, (sp==0)?LOW:HIGH);
        MotorState = true;
        timeToTurnOffMotor = millis() + s;
      }
      if (comBuf[0] == 'T'){ // run 2p Trigger!
        digitalWrite(GPIO2, HIGH);
        digitalWrite(GPIO2, LOW);   
      }
      if (comBuf[0] == 'R'){ // rotate!
        int sp = atoi(comBuf+1);
        stepper2Pos += sp;
        digitalWrite(STEPPER2_NEN, LOW);
        stepper2.moveTo(stepper2Pos);
        stepper2AtPos = false;
      }

      if (comBuf[0] == 'Z'){ // Moving stage- In Trial
        Serial.println("hi");
        Serial.println(comBuf[0]);
        
        
        digitalWrite(STEPPER1_NEN, LOW);
        digitalWrite(STEPPER1_DIR,HIGH); // Enables the motor to move in a particular direction

        // Makes 200 pulses for making one full cycle rotation
        int tL, TMOVEf, TMOVEb;
        sscanf(comBuf, "Z%d,%d,%d", &tL, &TMOVEf, &TMOVEb);
        // Makes 400 pulses for making two full cycle rotation

        Serial.println(tL);
        Serial.println(TMOVEf);
        Serial.println(TMOVEb);

        for(int x = 0; x < TMOVEf; x++) {
          digitalWrite(STEPPER1_STEP,HIGH);
          delayMicroseconds(100);
          digitalWrite(STEPPER1_STEP,LOW);
          delayMicroseconds(100);
        }

        

        long startTime = micros();
        while (micros() - startTime < tL)
          stepper2.run();   

        
        digitalWrite(STEPPER1_DIR,LOW); //Changes the rotations direction

        // Makes 400 pulses for making two full cycle rotation

        for(int x = 0; x < TMOVEb; x++) {
          digitalWrite(STEPPER1_STEP,HIGH);
          delayMicroseconds(100);
          digitalWrite(STEPPER1_STEP,LOW);
          delayMicroseconds(100);

        }

        delay(1000);


      }
      if (comBuf[0] == '+'){ // Moving stage forward- out of Trial
        Serial.println("hi");
        Serial.println(comBuf[0]);
        digitalWrite(STEPPER1_NEN, LOW);
        digitalWrite(STEPPER1_DIR,HIGH); // Enables the motor to move in a particular direction
        int TMOVEf;
        sscanf(comBuf+1, "%d", &TMOVEf);
        Serial.println(TMOVEf);
        for(int x = 0; x < TMOVEf; x++) {
          digitalWrite(STEPPER1_STEP,HIGH);
          delayMicroseconds(100);
          digitalWrite(STEPPER1_STEP,LOW);
          delayMicroseconds(100);
        }
      }
      
      if (comBuf[0] == 'e'){ // disable motors!
        digitalWrite(STEPPER2_NEN, HIGH);
        digitalWrite(STEPPER1_NEN, HIGH);
      }
      if (comBuf[0] == '-'){ // Moving stage backward- out of Trial
        Serial.println("hi");
        Serial.println(comBuf[0]);
        digitalWrite(STEPPER1_NEN, LOW);
        digitalWrite(STEPPER1_DIR,LOW); // Enables the motor to move in a particular direction
        int TMOVEb;
        sscanf(comBuf+1, "%d", &TMOVEb);
        Serial.println(TMOVEb);
        for(int x = 0; x < TMOVEb; x++) {
          digitalWrite(STEPPER1_STEP,HIGH);
          delayMicroseconds(100);
          digitalWrite(STEPPER1_STEP,LOW);
          delayMicroseconds(100);
        }

      }
      if (comBuf[0] == 'e'){ // disable motors!
        digitalWrite(STEPPER2_NEN, HIGH);
        digitalWrite(STEPPER1_NEN, HIGH);
      }
      if (comBuf[0] == 'C'){ // set speed and acceleration
        int sp, ac;
        sscanf(comBuf, "C%d,%d", &sp, &ac);
        stepper2.setMaxSpeed(sp);
        stepper2.setAcceleration(ac);

      }
      if (comBuf[0] == 'O'){ // set optogenetic parameters
        sscanf(comBuf, "O%d,%d,%d,%d", &tLed, &tGate, &optoWindowStart, &optoWindowEnd);
      }
      if (comBuf[0] == 'V'){ // open valve 
        int v;
        sscanf(comBuf, "V%d", &v);
        digitalWrite(RELAY0, HIGH);
        valveState = true;
        timeToTurnOffValve = millis() + v;
      }
      if (comBuf[0] == 'W'){ // Bazzer Left
        int w;
        sscanf(comBuf, "W%d", &w);
        digitalWrite(RELAY1, HIGH);
        delay(15);
        digitalWrite(RELAY1, LOW);
      }
      // if (comBuf[0] == 'M'){ // Wheel stopper- hold
      //   // int M;
      //   // sscanf(comBuf, "M%d", &M);
      //   digitalWrite(RELAY2, HIGH);
      // }
      // if (comBuf[0] == 'M'){ // Wheel stopper- release
      //   int tm;
      //   sscanf(comBuf+1, "%d", &tm);
      //   digitalWrite(RELAY0, HIGH);
      //   long startTime = micros();
      //   while (micros() - startTime < tm)
      //     stepper2.run(); 
      //   digitalWrite(RELAY0, LOW);
        
      // }
      if (comBuf[0] == 'M'){ // Wheel stopper- release
        digitalWrite(LED1, HIGH);
      }
      if (comBuf[0] == 'm'){ // Wheel stopper- release
        digitalWrite(LED1, LOW);
      }

      if (comBuf[0] == 'B'){ // start continuous opto
        int f;
        sscanf(comBuf+1, "%d", &f);
        float period = 1000000.0/f;
        IT.begin(pulseOpto, period);  
      }
      if (comBuf[0] == 'b'){ // end continuous opto
        IT.end();
      }
      if (comBuf[0] == 'Q'){ // start continuous opto
        int tL;
        int tG = 500000; // 500 msec
        sscanf(comBuf+1, "%d%d", &tL, &tG);
        // sscanf(comBuf+2, "%d", &tG);
        long startTime = micros();
        digitalWrite(GPIO4, HIGH); // start gating
        while (micros() - startTime < tG)
          stepper2.run();        
        digitalWrite(GPIO3, HIGH); // turn on LED
        while (micros() - startTime < tL + tG)
          stepper2.run();
        digitalWrite(GPIO3, LOW); // end continuous opto
        while (micros() - startTime < tL + 2* tG)
          stepper2.run();
        digitalWrite(GPIO4, LOW); // end continuous gating
        // analogWrite(DAC0, 4095);

      }
      // if (comBuf[0] == 'q'){ 
      //    // turn off LED

      //   digitalWrite(GPIO3, LOW);
      //   // analogWrite(DAC0, 0);

      
      if (comBuf[0] == 'A' || comBuf[0] == 'a'){ // Reset position to zero
        Reset = (comBuf[0] == 'A'); 
      }
      if (comBuf[0] == 'L'){ // Lock lever
          analogWrite(DAC0, 4095);
      }
      if (comBuf[0] == 'l'){ // Unlock lever
          analogWrite(DAC0, 0);
      }
      nChar=0;
    } else {
      nChar++;
    }
  }
  if (frameNum != lastSentFrameNum){
    Serial.print("F"); // frame number
    Serial.println(frameNum);
    Serial.flush();
    lastSentFrameNum = frameNum;
  }
  
  noInterrupts();
  unsigned long tlnf = tLastNewFrame;
  interrupts();
  if (tlnf > resetTime){
    frameNum=0;
  }
  

  
  loopIter++;
  if (loopIter == 5){
     a = analogRead(GPIO4);
     b = analogRead(GPIO5); 
    
     if(a>900){aState = 1;}else{aState = 0;}
     if(b>900){bState = 1;}else{bState = 0;}
     
     if (aState != aLastState)
     {     
     if (bState != aState){ 
        counter++;
     }
     else {
       counter--;
      }
     }
     
   aLastState = aState;

     if (Reset){counter = 0;}
     
   angle = counter*(0.18); 
   
    Serial.print("G"); // rotation angle
    Serial.println(angle); 
    
    loopIter = 0;

  }

    
  if (valveState && millis() >= timeToTurnOffValve){
    digitalWrite(RELAY0, LOW);
    valveState = false;
  }
//  if (valveState && millis() >= timeToTurnOffValve){
//    digitalWrite(RELAY1, LOW);
//    valveState = false;
//  }
  if (valveState && millis() >= timeToTurnOffValve){
    digitalWrite(RELAY2, LOW);
    valveState = false;
  }
  
  if (MotorState && millis() >= timeToTurnOffMotor){
    analogWrite(DCMOTOR_PWMA, 0);
    MotorState = false;
  }
  stepper2.run();
}

