#include <HMotor.h>
#include <motor_define.h>

HMotor motor_left(MOTOR_LEFT);
HMotor motor_right(MOTOR_RIGHT);

void setup() {
  Serial.begin(115200);
  motor_left.initMotor();
  motor_left.setDirection(MOTOR_FWD);
  motor_left.setVelocity(20);
  /*motor_right.initMotor();
  motor_right.setDirection(MOTOR_FWD);
  motor_right.setVelocity(20);*/
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
}

void loop() {
 // printEverythingLeft();
  //delay(1000);
  motor_left.updateMotor();
  //motor_right.updateMotor();
  
  //Serial.println(motor_right._sum_error);
  //delay(1000);
}

void printEverythingLeft() {
  /*Serial.print(motor_left._dir);
  Serial.print("\t");
  Serial.print(motor_left._tgt_vel);
  Serial.print("\t");
  Serial.print(motor_left._loop_mode);
  Serial.print("\t");
  Serial.print(motor_left._dir);
  Serial.print("\t");
  Serial.print(motor_left.getVelocity());
  Serial.print("\t");
  Serial.println("");*/
}
