#include <ArduinoHardware.h>

#include <ros.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>

#include <std_msgs/Int16.h>





#define EN_L 12

#define IN1_L 10

#define IN2_L 11



#define EN_R 13

#define IN1_R 8

#define IN2_R 9



#define ENC_IN_LEFT_A 20

#define ENC_IN_RIGHT_A 18



#define ENC_IN_LEFT_B 21

#define ENC_IN_RIGHT_B 19





double w_r=0, w_l=0;



//wheel_rad is the wheel radius ,wheel_sep is

double wheel_rad = 0.0325, wheel_sep = 0.295;





ros::NodeHandle nh;

int lowSpeed = 50;

int highSpeed = 255;



double speed_ang=0, speed_lin=0;



boolean Direction_left = true;

boolean Direction_right = true;



const long encoder_minimum = -327682;

const long encoder_maximum = 327671;



const int interval = 100;

long previousMillis = 0;

long currentMillis = 0;



std_msgs::Int32 rwheel_ticks_32bit_msg;

ros::Publisher rightPub("rwheel_ticks_32bit", &rwheel_ticks_32bit_msg);

std_msgs::Int32 lwheel_ticks_32bit_msg;

ros::Publisher leftPub("lwheel_ticks_32bit", &lwheel_ticks_32bit_msg);



void right_wheel_tick() {

   

  // Read the value for the encoder for the right wheel

  int val = digitalRead(ENC_IN_RIGHT_B);

  if(val == LOW) {

    Direction_right = false; // Reverse

  }

  else {

    Direction_right = true; // Forward

  }

   

  if (Direction_right) {

     

    if (rwheel_ticks_32bit_msg.data == encoder_maximum) {

      rwheel_ticks_32bit_msg.data = encoder_minimum;

    }

    else {

      rwheel_ticks_32bit_msg.data++; 

    }   

  }

  else {

    if (rwheel_ticks_32bit_msg.data == encoder_minimum) {

      rwheel_ticks_32bit_msg.data = encoder_maximum;

    }

    else {

      rwheel_ticks_32bit_msg.data--; 

    }   

  }

}

// Increment the number of ticks

void left_wheel_tick() {

   

  // Read the value for the encoder for the left wheel

  int val = digitalRead(ENC_IN_LEFT_B);

  if(val == LOW) {

    Direction_left = true; // Reverse

  }

  else {

    Direction_left = false; // Forward

  }

   

  if (Direction_left) {

    if (lwheel_ticks_32bit_msg.data == encoder_maximum) {

      lwheel_ticks_32bit_msg.data = encoder_minimum;

    }

    else {

      lwheel_ticks_32bit_msg.data++; 

    } 

  }

  else {

    if (lwheel_ticks_32bit_msg.data == encoder_minimum) {

      lwheel_ticks_32bit_msg.data = encoder_maximum;

    }

    else {

      lwheel_ticks_32bit_msg.data--; 

    }   

  }

}

















void messageCb( const geometry_msgs::Twist& msg){

  speed_ang = msg.angular.z;

  speed_lin = msg.linear.x;

  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

}





ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void Motors_init();

void MotorL(int Pulse_Width1);

void MotorR(int Pulse_Width2);



void setup(){



  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);

  pinMode(ENC_IN_LEFT_B , INPUT);

  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);

  pinMode(ENC_IN_RIGHT_B , INPUT);

  // Every time the pin goes high, this is a tick

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);





 



  Motors_init();



  nh.initNode();

  nh.getHardware()->setBaud(57600);

  nh.subscribe(sub);

  nh.advertise(rightPub);

  nh.advertise(leftPub);

}





void loop(){

MotorL(w_l);



MotorR(w_r);



nh.spinOnce();



  currentMillis = millis();

  // If 100ms have passed, print the number of ticks

  if (currentMillis - previousMillis > interval) {

     

    previousMillis = currentMillis;

     

    rightPub.publish( &rwheel_ticks_32bit_msg );

    leftPub.publish( &lwheel_ticks_32bit_msg );

  }



}









void Motors_init(){



pinMode(EN_L, OUTPUT);



pinMode(EN_R, OUTPUT);



pinMode(IN1_L, OUTPUT);



pinMode(IN2_L, OUTPUT);



pinMode(IN1_R, OUTPUT);



pinMode(IN2_R, OUTPUT);



digitalWrite(EN_L, LOW);



digitalWrite(EN_R, LOW);



digitalWrite(IN1_L, LOW);



digitalWrite(IN2_L, LOW);



digitalWrite(IN1_R, LOW);



digitalWrite(IN2_R, LOW);



}



void MotorL(int Pulse_Width1){

if (Pulse_Width1 > 0){

  if (Pulse_Width1>255){

    Pulse_Width1 =255;

  }



     analogWrite(EN_L, Pulse_Width1);



     digitalWrite(IN1_L, HIGH);



     digitalWrite(IN2_L, LOW);



}







if (Pulse_Width1 < 0){



     Pulse_Width1=abs(Pulse_Width1);

    if (Pulse_Width1>255){

    Pulse_Width1 =255;

  }

     



     analogWrite(EN_L, Pulse_Width1);



     digitalWrite(IN1_L, LOW);



     digitalWrite(IN2_L, HIGH);



}





if (Pulse_Width1 == 0){



     analogWrite(EN_L, 0);



     digitalWrite(IN1_L, LOW);



     digitalWrite(IN2_L, LOW);



}



}





void MotorR(int Pulse_Width2){





  if (Pulse_Width2 > 0){

  if (Pulse_Width2>255){

    Pulse_Width2 =255;

  }



     analogWrite(EN_R, Pulse_Width2);



     digitalWrite(IN1_R, LOW);



     digitalWrite(IN2_R, HIGH);



}



if (Pulse_Width2 < 0){



  Pulse_Width2=abs(Pulse_Width2);

  if (Pulse_Width2>255){

    Pulse_Width2 =255;

  }



     analogWrite(EN_R, Pulse_Width2);



     digitalWrite(IN1_R, HIGH);



     digitalWrite(IN2_R, LOW);



}



if (Pulse_Width2 == 0){



     analogWrite(EN_R, 0);



     digitalWrite(IN1_R, LOW);



     digitalWrite(IN2_R, LOW);



}



}
