#include <motor_define.h>
#include <motor_control.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//todo: make classes for motors, all functions defined,
// just pass in pin assignments 

void setup_digital_pins() {
  pinMode(M1_IN_A, OUTPUT);
  pinMode(M1_IN_B, OUTPUT);
  pinMode(M2_IN_A, OUTPUT);
  pinMode(M2_IN_B, OUTPUT);

  pinMode(M1_ENC_A, INPUT);
  pinMode(M1_ENC_B, INPUT);
  pinMode(M2_ENC_A, INPUT);
  pinMode(M2_ENC_B, INPUT);
}



#define MSR_INTVL_MS 5
#define COUNTS_PER_RAD 509.296 // (3200 counts/rev) / (2pi rads / rev)
#define BUF_SIZE 5

float vel_buf[BUF_SIZE];
float vel_sum = 0;

void zero_vel_buf() {
  int i;
  for(i = 0; i < BUF_SIZE; i++) {
    vel_buf[i] = 0;
  }
}

/* Returns velocity in rads/sec */
float estimateVel() {
  uint8_t oldstate = readEncoder(MOTOR_LEFT);
  uint8_t curstate;
  int count;
  long start_time = millis();
  
  while(millis() - start_time < MSR_INTVL_MS) {
    oldstate = curstate;
    curstate = readEncoder(MOTOR_LEFT);
    if((oldstate << 1) ^ (curstate << 1)) {
      count++;
    }
  }
  
  //return ((float)count * 1000/MSR_INTVL_MS) * 60 / 3200;
  return ((float)count * (1000 / MSR_INTVL_MS)) / COUNTS_PER_RAD;
}

void shift_into_buf(float *buf, float val) {
  int i;
  for(i = 0; i < BUF_SIZE - 1; i++) {
    buf[i] = buf[i+1];
  }
  buf[BUF_SIZE - 1] = vel;
}

/* Updates internal rad/s velocity buffer and returns the 
 * new velocity estimate */
float get_updated_vel() {
  float newVal = estimateVel();
  vel_sum += newVal - vel_buf[0];
  shift_into_buf(vel_buf, newVal); 
  return vel_sum / BUF_SIZE;
}


/* PID variables */
float rads_target = 2, rads_current = 0;
float old_error, cur_error, sum_error = 0;
int freq = 100; // run at [freq] Hz
long time_start;
float pwm_duty = 1;

/* PWM gains */
float k_p = .017, pwm_p;
float k_i = .00002, pwm_i;
float k_d = .3, pwm_d;

// good no load gains: .005, .00005, .01
// good 15 rad/s vice gains: .01, .00001, .12
// better 15rsvg: .01, .000001, .15
// nice 15rsvsg: .012, .00002, .2
// even nice 15rsvg: .015, .00002, .2
// really bueno: .015, .00002, .3

void setup() {
  
  /** Mandatory setup **/
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  /* Digital pin config */
  setup_digital_pins();
  initializeMotorPWM();
  
  zero_vel_buf();
 
  /* Feed a value into the encoder before we start moving */
  setMotorDirection(MOTOR_LEFT, MOTOR_FWD);
  setMotorSpeed(MOTOR_LEFT, 1);
  
  time_start = millis();
}

void loop() {
  //time_start = micros();
  rads_current = get_updated_vel();
  Serial.println(rads_current);
  //rads_current = get_updated_vel();
  old_error = cur_error;
  cur_error = rads_target - rads_current;
  //Serial.println(cur_error);
  
  /* Calculate contributions to new voltage level for each gian */
  pwm_p = k_p * cur_error;
  
  sum_error += cur_error;
  pwm_i = k_i * sum_error;
  
  pwm_d = k_d * (cur_error - old_error);
  //Serial.println(pwm_p);
  
  pwm_duty += pwm_p + pwm_i + pwm_d;
  trim_float(&pwm_duty, 0.0, 1.0);
  //Serial.println(pwm_duty);
  
  setMotorSpeed(MOTOR_LEFT, pwm_duty);
  //Serial.println(micros() - time_start);
  
 // pwm_duty += pwm_p;
  //trim_float(&pwm_duty, 0.0, 1.0);
  
 // Serial.print("PWM: ");
  //Serial.println(pwm_p);
  /* Send PWM to motor */
  /*setMotorSpeed(MOTOR_LEFT, pwm_duty);*/
  //Serial.println("");
  //delay(1000);
  
  if(millis() - time_start > 10000) {
    rads_target = 7;
  }
  /*else if(millis() - time_start > 15000) {
    rads_target = 0;
  }
  else if (millis() - time_start > 10000) { 
    rads_target = 6.28;
  }*/
  else if(millis() - time_start > 5000) {
    rads_target = 15;
  }
}



