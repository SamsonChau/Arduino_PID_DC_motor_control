#include <Encoder.h>
#include "ArduPID.h"

#define EN 8
#define L_PWM  9
#define R_PWM 10

#define en_a 2
#define en_b 3

#define en_PPR 448.0
#define motor_gr 18.8
double encoder_bias = 0.0;
Encoder encoder(en_a, en_b);
  double feedback = 0;
  double output = 0;
  double setpoint = 0;
  double Kp = 0.5;
  double Ki = 0.1;
  double Kd = 0.05;

ArduPID motor_pid;

int set_position = 0;
int bytes = 0;
void motor_drive(int duty){
   if(duty < 0.0){
    analogWrite(R_PWM,abs(duty));
    analogWrite(L_PWM,0);
 }else if(duty > 0.0){
    analogWrite(L_PWM,abs(duty));
    analogWrite(R_PWM,0);
 }else{
    analogWrite(L_PWM,0);
    analogWrite(R_PWM,0);
 }
}
void setup() {
  Serial.begin(115200);
  pinMode(R_PWM,OUTPUT);
  pinMode(L_PWM,OUTPUT);
  pinMode(EN,OUTPUT);
  digitalWrite(EN,HIGH);
  encoder_bias = 360.0 / (motor_gr * en_PPR * 4);
  motor_pid.begin(&feedback, &output, &setpoint, Kp, Ki, Kd);
  
  // myController.reverse()               // Uncomment if controller output is "reversed"
  motor_pid.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  motor_pid.setOutputLimits(-255, 255);
  motor_pid.setBias(255.0 / 2.0);
  motor_pid.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  
  motor_pid.start();
  myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
}


void loop() {
  if (Serial.available()) {
    bytes = Serial.read();
    if (bytes != '\n'){
      set_position = set_position + 10;
      setpoint = (double)set_position;
    }
  }
  
 feedback = encoder.read()* encoder_bias;
 motor_pid.compute();
 motor_drive((int)output);
 delay(10);
}
