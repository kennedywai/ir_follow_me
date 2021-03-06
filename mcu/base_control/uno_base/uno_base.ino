/*
20160319 zach
1. add WHEEL_TYPE Define LEFT or RIGHT 
*/


//#include <Metro.h>
//#include "SoftwareSerial.h";

#define RIGHT_WHEEL 1
#define LEFT_WHEEL 2

#define WHEEL_TYPE RIGHT_WHEEL

#define encoder0PinA  2
#define encoder0PinB  3

#define motorIn1 6
#define InA 8
#define InB 7

#define LOOPTIME 40

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;
/*
const int Tx = 11;
const int Rx = 10; 
*/
char commandArray[3];
byte sT = 0;  //send start byte
byte sH = 0;  //send high byte
byte sL = 0;  //send low byte
byte sP = 0;  //send stop byte

byte rT = 0;  //receive start byte
byte rH = 0;  //receive high byte
byte rL = 0;  //receive low byte
byte rP = 0;  //receive stop byte

//SoftwareSerial mySerial(Rx, Tx);

volatile long Encoderpos = 0;
volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;
unsigned long cc = 0;

double omega_target = 0.0;
//double omega_target = 2.0;
double omega_actual = 0;

int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int CPR = 13*4;                                   // encoder count per revolution old 13*4  new 7*4
int gear_ratio = 65.5; 
int actual_send = 0;
int target_receive = 0;

//float Kp = 1.0;
//float Ki = 0.03;
//float Ki = 0.005;

float Kp = 0.9;
//float Ki = 0.03;
float Ki = 0.005;

float Kd = 0;
double error;
double pidTerm = 0;                                                            // PID correction
double sum_error, d_error=0;

double calculated_pidTerm;
double constrained_pidterm;

void setup() { 
 //TCCR0B = TCCR0B & B11111000 | B00000010; 
 pinMode(encoder0PinA, INPUT); 
 digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
 pinMode(encoder0PinB, INPUT); 
 digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

 attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
 attachInterrupt(1, doEncoder, CHANGE);
// pinMode(Rx, INPUT); pinMode(Tx, OUTPUT);
 pinMode(InA, OUTPUT); 
 pinMode(InB, OUTPUT); 
 //mySerial.begin (19200);
 Serial.begin (57600);
 //Serial.begin (57600);
} 

void loop() 
{       
  readCmd_wheel_angularVel();
//  omega_target = 0.1;
  /*
  if (millis()<2000)
    {
        omega_target = 8;
    }
  if (millis()>=5000)
    {
        omega_target = 0;
    }
  */  
  if((millis()-lastMilli) >= LOOPTIME)   
     {                                    // enter tmed loop
        dT = millis()-lastMilli;
        lastMilli = millis();
        
        getMotorData();                                                           // calculate speed
         
        //PWM_val = 255; //10.3 rad/s PWM_val = 192; //8.33 rad/s PWM_val = 128; //7.58 rad/s PWM_val = 64; //3.61 rad/s
        //PWM_val = 120; //6.65 rad/s

        sendFeedback_wheel_angularVel(); //send actually speed to mega
        
        PWM_val = (updatePid(omega_target, omega_actual));                       // compute PWM value from rad/s 
//	if (omega_target == 0 && omega_actual == 0) PWM_val = 0; 
  if (omega_target == 0) PWM_val = 0; 
            if (PWM_val <= 0)   { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, LOW);  digitalWrite(InB, HIGH); }
            if (PWM_val > 0)    { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, HIGH);   digitalWrite(InB, LOW);}
            
        //printMotorInfo();
        //cc = cc +1;
     }
}

void readCmd_wheel_angularVel()
{
  //if (mySerial.available() > 4) 
  if (Serial.available() > 4) 
  {
    //char rT = (char)mySerial.read(); //read target speed from mega
    char rT = (char)Serial.read(); //read target speed from mega
          if(rT == '{')
            {
              char commandArray[3];
              //mySerial.readBytes(commandArray,3);
              Serial.readBytes(commandArray,3);
              byte rH=commandArray[0];
              byte rL=commandArray[1];
              char rP=commandArray[2];
              if(rP=='}')         
                {
                  target_receive = (rH<<8)+rL; 
                  omega_target = double (target_receive*0.00076293945);  //convert received 16 bit integer to actual speed
                }
            }
  }         
}

void sendFeedback_wheel_angularVel()
{
  actual_send = int(omega_actual/0.00076293945); //convert rad/s to 16 bit integer to send
  char sT='{'; //send start byte
  byte sH = highByte(actual_send); //send high byte
  byte sL = lowByte(actual_send);  //send low byte
  char sP='}'; //send stop byte
  //mySerial.write(sT); mySerial.write(sH); mySerial.write(sL); mySerial.write(sP);
  Serial.write(sT); Serial.write(sH); Serial.write(sL); Serial.write(sP);
}

void getMotorData()  
{                               
  static long EncoderposPre = 0;       
  //converting ticks/s to rad/s
  //omega_actual = 4.5;
  if (WHEEL_TYPE==LEFT_WHEEL)
    omega_actual = -((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio) ;  //ticks/s to rad/s
  else
    omega_actual = ((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio) ;  //ticks/s to rad/s
    
  EncoderposPre = Encoderpos;                 
}

double updatePid(double targetValue,double currentValue)   
{            
  
  static double last_error=0;                            
  error = targetValue - currentValue; 
  // Added by KKuei to remove the noises
  //if (error <= 0.1 && error >= -0.1) error = 0.0;
  
  sum_error = sum_error + error * dT;
  // Added by KKuei to bound sum_error range
  sum_error = constrain(sum_error, -880, 880);
  
  d_error = (error - last_error) / dT;
  pidTerm = Kp * error + Ki * sum_error + Kd * d_error;   
  //pidTerm = Kp * error + Kd * d_error;                         
  last_error = error;  
  if (WHEEL_TYPE==LEFT_WHEEL)
    calculated_pidTerm = -pidTerm/0.04039215686;
  else
    calculated_pidTerm = pidTerm/0.04039215686;  
      
  constrained_pidterm = constrain(calculated_pidTerm, -255, 255);
  
  return constrained_pidterm;
  
  //return constrain(int(pidTerm/0.04039215686), -255, 255);
}


void doEncoder() {
//   Encoderpos++;
  pinAState = digitalRead(2);
  pinBState = digitalRead(3);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      Encoderpos --;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      Encoderpos --;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      Encoderpos --;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
}

void printMotorInfo()  
{                                                                      
   //Serial.print("  actual:");                  Serial.print(omega_actual);
   //Serial.println();
  
   //Serial.print("  target:");                  Serial.print(omega_target);
   
   //Serial.println();
  
   Serial.print(" target:");                  Serial.print(omega_target);
   Serial.print(" actual:");                  Serial.print(omega_actual);
   Serial.print(" sum_error:");              Serial.print(sum_error);
   Serial.print("  error:");                  Serial.print(error);
   Serial.print("  samples:");                  Serial.print(cc);
   //Serial.print("  dT:");                  Serial.print(dT);
   //Serial.println();

   Serial.println();
}

/*
void printMotorInfo()  
{                                                                      
   static int samples = 0;
   
   //Serial.print("  actual:");                  Serial.print(omega_actual);
   //Serial.println();
  
   //Serial.print("  target:");                  Serial.print(omega_target);
   
   //Serial.println();

   if (samples >= 1) {
      Serial.print("  target:");                  Serial.print(omega_target, 4);
      Serial.print("  actual:");                  Serial.print(omega_actual, 4);
      Serial.print("  sum_err:");                  Serial.print(sum_error, 4);
      Serial.print("  error:");                  Serial.print(error, 4);
      Serial.print("  dT:");                  Serial.print(dT, 4);
      Serial.print("  pidTerm:");             Serial.print(pidTerm, 4);
      Serial.print("  calculated_pidTerm:");  Serial.print(calculated_pidTerm, 4);
      Serial.print("  constrained_pidterm:"); Serial.print(constrained_pidterm, 4); 
      //Serial.println();

      Serial.println();

      samples = 0;
   } else 
   {
     samples ++;
   }  
}
*/
