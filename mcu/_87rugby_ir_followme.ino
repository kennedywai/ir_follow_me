/**
 * 5 IR sensors follow me function for 87rugby 
 */

//#include <SharpIR.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <Arduino.h>
#include <math.h>

#define  LEFT_FOLLOW_IR_SIDE        A0  // Read side left following IR sensor pin
#define  LEFT_FOLLOW_IR_VALUE_SIDE  analogRead(LEFT_FOLLOW_IR_SIDE)

#define  LEFT_FOLLOW_IR             A1  // Read left following IR sensor pin
#define  LEFT_FOLLOW_IR_VALUE       analogRead(LEFT_FOLLOW_IR)

#define  FOLLOWIR                   A2  // Read Middle following IR sensor pin
#define  FOLLOW_IR_VALUE            analogRead(FOLLOWIR)

#define  RIGHT_FOLLOW_IR            A3  // Read right following IR sensor pin
#define  RIGHT_FOLLOW_IR_VALUE      analogRead(RIGHT_FOLLOW_IR)

#define  RIGHT_FOLLOW_IR_SIDE       A4  // Read side right following IR sensor pin
#define  RIGHT_FOLLOW_IR_VALUE_SIDE analogRead(RIGHT_FOLLOW_IR_SIDE)

float IR_Distance = 0, Left_IR_Distance = 0, Right_IR_Distance = 0, Left_IR_Side_Distance = 0, Right_IR_Side_Distance = 0;
float Follow_IR_f = 0, Left_Follow_IR_f = 0, Right_Follow_IR_f = 0, Left_Follow_IR_Side_f = 0, Right_Follow_IR_Side_f = 0;
unsigned long PastTime = 0, PastPrintTime = 0, PastFilterTime = 0;
unsigned long PresentTime = millis();

ros::NodeHandle nh;
geometry_msgs::Vector3 ir_value, ir_value_side;
ros::Publisher pub_ir_value("sensor/ir_value", &ir_value);
ros::Publisher pub_ir_value_side("sensor/ir_value_side", &ir_value_side);

void setup() {
  pinMode(FOLLOWIR, INPUT);  
  pinMode(LEFT_FOLLOW_IR, INPUT);
  pinMode(RIGHT_FOLLOW_IR, INPUT); 
  pinMode(LEFT_FOLLOW_IR_SIDE, INPUT); 
  pinMode(RIGHT_FOLLOW_IR_SIDE, INPUT); 
  
  nh.initNode();
  nh.advertise(pub_ir_value);
  nh.advertise(pub_ir_value_side);
  
  Serial.begin(1000000);
}

void loop() {

  PresentTime = millis();              
  if((int)(PresentTime-PastFilterTime) > 20){// 20ms
    PastFilterTime = PresentTime;
    
    // Filtering the IR sensor value
    Follow_IR_f = 0.9*Follow_IR_f + 0.1*FOLLOW_IR_VALUE;
    Left_Follow_IR_f = 0.9*Left_Follow_IR_f + 0.1*LEFT_FOLLOW_IR_VALUE;
    Right_Follow_IR_f = 0.9*Right_Follow_IR_f + 0.1*RIGHT_FOLLOW_IR_VALUE;
    Left_Follow_IR_Side_f = 0.9*Left_Follow_IR_Side_f + 0.1*LEFT_FOLLOW_IR_VALUE_SIDE;
    Right_Follow_IR_Side_f = 0.9*Right_Follow_IR_Side_f + 0.1*RIGHT_FOLLOW_IR_VALUE_SIDE;
    
    // Transform the filtered IR sensor value to distance
    IR_Distance = 10650.08 * pow(Follow_IR_f, -0.935) - 10;                
    Left_IR_Distance = 10650.08 * pow(Left_Follow_IR_f, -0.935) - 10; 
    Right_IR_Distance = 10650.08 * pow(Right_Follow_IR_f, -0.935) - 10;
    Left_IR_Side_Distance = 10650.08 * pow(Left_Follow_IR_Side_f, -0.935) - 10;
    Right_IR_Side_Distance = 10650.08 * pow(Right_Follow_IR_Side_f, -0.935) - 10;
    
  }

  PresentTime = millis();              // 8~14 us
  if((int)(PresentTime-PastPrintTime) > 100){   // 20ms
    PastPrintTime = PresentTime;
    ir_value.x = Left_IR_Distance;
    ir_value.y = IR_Distance;
    ir_value.z = Right_IR_Distance;
    
    ir_value_side.x = Left_IR_Side_Distance;
    ir_value_side.y = Right_IR_Side_Distance;
    
    pub_ir_value.publish(&ir_value);
    pub_ir_value_side.publish(&ir_value_side);
    
    nh.spinOnce();
    //printIRInfo(); 
  }

}
/*
 * Low memory available, stability problems may occur.
 * 
void printIRInfo()
{
  //Serial.print(PresentTime);
  //Serial.println("  ");
  delay(500);
  Serial.print("Left_IR_SIDE_Distance: ");
  Serial.println(Left_IR_Side_Distance);
  Serial.print("Left_IR_Distance: ");
  Serial.println(Left_IR_Distance);
  Serial.print("IR_Distance: ");
  Serial.println(IR_Distance);
  Serial.print("Right_IR_Distance: ");
  Serial.println(Right_IR_Distance);
  Serial.print("Right_IR_SIDE_Distance: ");
  Serial.println(Right_IR_Side_Distance);
  Serial.println("  ");
  }
*/
