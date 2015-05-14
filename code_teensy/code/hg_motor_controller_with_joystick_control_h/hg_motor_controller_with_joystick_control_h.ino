#include <HMotor.h>
#include <motor_define.h>

#define HWSERIAL Serial3

#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

char buf[PACKET_SIZE];

HMotor motor_left(MOTOR_LEFT);
HMotor motor_right(MOTOR_RIGHT);

/* Basic read, checks for proper start byte */
int readInPacket(char *buf) {
  int success = 0;
  int bytes_read = 0;
  char c;

  /* Find beginning of packet */
  while( HWSERIAL.available() && (c = HWSERIAL.read()) != PACKET_START) {
    /*Serial.print(c);*/  }
  buf[0] = PACKET_START;
  bytes_read++;

  /* Read rest of packet */
  while( ((c = HWSERIAL.read()) != -1) && bytes_read < PACKET_SIZE) {
    buf[bytes_read] = c;
    bytes_read++;
  }

  /* Verification */
  if(buf[PACKET_SIZE - 1] == PACKET_END) {
    success = 1;
  }

  return success;
}

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  motor_left.initMotor();
  motor_left.setDirection(MOTOR_FWD);
  motor_left.setVelocity(3);

  motor_right.initMotor();
  motor_right.setDirection(MOTOR_FWD);
  motor_right.setVelocity(3);
  //motor_right.setBrake();


  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

}
float vel;
long time;
void loop() {
 // time = micros();
  motor_right.updateMotor();
  motor_left.updateMotor();
 
 acceptCommands();
  Serial.print(motor_left._tgt_vel);
  Serial.print("\t");
  Serial.print(motor_left._cur_vel);
  Serial.print("\t");
  Serial.print(motor_right._tgt_vel);
  Serial.print("\t");
  Serial.println(motor_right._cur_vel);
  //Serial.println(micros() - time);
  /*
  Serial.print(motor_left.getVelocity());
  Serial.print("\t");
  //Serial.print(motor_left._tgt_vel);
  Serial.print("\t");
  Serial.print(motor_right.getVelocity());
  Serial.print("\t");*/
  //Serial.println(motor_right._tgt_vel);

}

void testOfTime() {
  /*if(millis() > 20000) {
   motor_left.setDirection(MOTOR_FWD);
   motor_left.setVelocity(20);
   motor_right.setDirection(MOTOR_FWD);
   motor_right.setVelocity(20);
   }
   else if(millis() > 15000) {
   motor_left.setDirection(MOTOR_FWD);
   motor_left.setVelocity(15);
   motor_right.setDirection(MOTOR_FWD);
   motor_right.setVelocity(15);
   }
   else if(millis() > 12000) {
   motor_left.setDirection(MOTOR_FWD);
   motor_left.setVelocity(10);
   motor_right.setDirection(MOTOR_FWD);
   motor_right.setVelocity(10);
   }
   else {
   motor_left.setDirection(MOTOR_FWD);
   motor_left.setVelocity(5);
   motor_right.setDirection(MOTOR_FWD);
   motor_right.setVelocity(5);
   }*/
}

/*void thisWorked() {
  motor_left.updateMotor();
  motor_right.updateMotor();
  Serial.print(motor_right._dir);
  Serial.print("\t");
  Serial.print(motor_right._cur_vel);
  Serial.print("\t");
  Serial.print(motor_right._tgt_vel);
  Serial.print("\t");
  Serial.print(motor_right._p_pwm);
  Serial.print("\t");
  Serial.print(motor_right._p_in_a);
  Serial.print("\t");
  Serial.print(motor_right._p_in_b);
  Serial.print("\t");
  Serial.println(motor_right._cur_duty);*/
  //Serial.print(motor_left.getVelocity());
  //Serial.print("\t");
  // Serial.print(motor_left._sum_error);
  // Serial.print("\t");
  //Serial.print(motor_right.getVelocity());
  //Serial.print("\t");
  //Serial.println("");
  //Serial.println(motor_right._sum_error);
//}

void showMeTheData() {
  /* Serial.print("[");
   Serial.print((int)buf[0]);
   Serial.print("][");
   Serial.print((int)buf[1]);
   Serial.print("][");
   Serial.print((int)buf[2]);
   Serial.print("][");
   Serial.print((int)buf[3]);
   Serial.print("][");
   Serial.print((int)buf[4]);
   Serial.print("][");
   Serial.print((int)buf[5]);
   Serial.print("][");
   Serial.print((int)buf[6]);
   Serial.print("][");
   Serial.print((int)buf[7]);
   Serial.print("][");
   Serial.print((char)buf[8]);
   Serial.print("][");
   Serial.print((char)buf[9]);
   Serial.print("][");
   Serial.println("]");*/

}

void acceptCommands() {
  if(HWSERIAL.available() >= PACKET_SIZE) {
    if(readInPacket(buf)) {
      // Serial.println("R");

      // left motor settings 
      if(buf[1] == 0) {
        motor_left.setBrake();
        motor_right.setBrake();
      }
      else {
        if(buf[1] == 1) {
          motor_left.setDirection(MOTOR_FWD);
          /*Serial.println("ey"); */        }
        else if(buf[1] == 2) {
          motor_left.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[2] % 23);
        vel += (float)((int)buf[3] % 100)/100;
        /*Serial.print(vel);
         Serial.print("\t");*/
        motor_left.setVelocity(vel);

        if(buf[4] == 1) {
          motor_right.setDirection(MOTOR_FWD); 
        }
        else if(buf[4] == 2) {
          motor_right.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[5] % 23);
        vel += (float)((int)buf[6] % 100)/100; 
        motor_right.setVelocity(vel);
        /*Serial.println(vel);*/
       /* motor_left.updateMotor();
        motor_right.updateMotor();*/
      }
    }
    else {
      /*Serial.println("F");*/
    }
  }
}

