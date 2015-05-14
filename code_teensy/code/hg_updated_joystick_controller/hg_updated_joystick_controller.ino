#include <motor_define.h>

#define HWSERIAL Serial1
#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

#define TRN_PIN 16
#define SPD_PIN 15
#define STT_PIN 10
#define DIR_PIN 9
#define FWD_LED 12
#define BWD_LED 11

#define DEBOUNCE_TIME_MS 200

uint8_t buf[PACKET_SIZE-2];

typedef struct controller_state {
  int is_moving;
  long is_moving_update_time;
  int dir;
  long dir_update_time;
  float spd;
  float turn;
  float spd_left;
  float spd_right;
  int fwd_led_state;
  int bwd_led_state;
  int brk_led_state;
  int is_point_turn;
} controller_state;

controller_state c_state;

/* Maybe want to separate sending data fucntion from wrapping it properly */
void sendPacket(uint8_t *buf) {
  int i;
  HWSERIAL.write(PACKET_START);
  for(i =  0; i < PACKET_SIZE-2; i++) {
    HWSERIAL.write(buf[i]);
  }
  HWSERIAL.write(PACKET_END);
}

void determine_move() {
  if(digitalRead(STT_PIN) && millis() - c_state.is_moving_update_time > DEBOUNCE_TIME_MS) {
    c_state.is_moving ^= 1;
    c_state.is_moving_update_time = millis();
  }
}

void determine_dir() {
  if(digitalRead(DIR_PIN) && millis() - c_state.dir_update_time > DEBOUNCE_TIME_MS) {
    if(c_state.dir == MOTOR_FWD) {
      c_state.dir = MOTOR_BWD;
    }
    else {
      c_state.dir = MOTOR_FWD;
    }
    c_state.dir_update_time = millis();
  } 
}

void determine_speed() {
  c_state.spd = (float)(1023 - analogRead(SPD_PIN)) / 1023.0 * 23.0;
}

#define POT_MID_VAL 512
#define POT_CTR_THRESH 100
void determine_turn() {
  int val = analogRead(TRN_PIN);
  float diff = val - POT_MID_VAL;
  
  if(c_state.is_moving) {
    c_state.is_point_turn = 0;
    if(abs(diff) > POT_CTR_THRESH) {
      c_state.is_point_turn = 1;
      if(diff > 0) {
        c_state.spd_left = c_state.spd - c_state.spd*(0.5 * diff/POT_MID_VAL) * c_state.dir;
        c_state.spd_right = c_state.spd * c_state.dir;
      }
      else {
        c_state.spd_left = c_state.spd * c_state.dir;
        c_state.spd_right = c_state.spd + c_state.spd*(0.5 * diff/POT_MID_VAL) * c_state.dir;
      }
    }
    else {
      c_state.spd_left = c_state.spd * c_state.dir;
      c_state.spd_right = c_state.spd * c_state.dir;
    }
    
  }
  else {
    if(abs(diff) > POT_CTR_THRESH) {
      c_state.is_point_turn = 1;
      c_state.spd_left = -(diff / 50);
      c_state.spd_right = -c_state.spd_left;
      /*Serial.print(c_state.spd_left);
      Serial.print("\t");
      Serial.println(c_state.spd_right);*/
    }
    else {
      c_state.spd_left = 0.0;
      c_state.spd_right = 0.0;
      c_state.is_point_turn = 0;
    }
  }
}

void determine_packet() {
  float left_vel = abs(c_state.spd_left);
  float right_vel = abs(c_state.spd_right);
  if((c_state.is_moving || c_state.is_point_turn) && left_vel > 0.20) {
    buf[0] = c_state.spd_left > 0 ? 1 : 2;
    buf[1] = (int) left_vel;
    buf[2] = (int)((left_vel - buf[1]) * 100);
    buf[3] = c_state.spd_right > 0 ? 1 : 2;
    buf[4] = (int) right_vel;
    buf[5] = (int)((right_vel - buf[4]) * 100);
  }
  else {
    buf[0] = buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = 0;
  }
}

void setLEDState() {
  digitalWrite(13, c_state.is_moving ? HIGH : LOW);
  digitalWrite(FWD_LED, c_state.dir == MOTOR_FWD ? HIGH : LOW);
  digitalWrite(BWD_LED, c_state.dir == MOTOR_FWD ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200); 
  buf[6] = 'f';
  buf[7] = 'g';
  pinMode(13, OUTPUT);
  pinMode(STT_PIN, INPUT);
  pinMode(DIR_PIN, INPUT);
  pinMode(FWD_LED, OUTPUT);
  pinMode(BWD_LED, OUTPUT);
  
  c_state.is_moving = 0;
  c_state.dir = MOTOR_FWD;
  c_state.spd = 0.0;
  c_state.turn = 0.0;
  c_state.is_moving_update_time = 0;
  c_state.dir_update_time = 0;
  c_state.is_point_turn = 0;
}

void loop() {
  
  determine_move();
  determine_dir();
  determine_speed();
  determine_turn();
  determine_packet();
  setLEDState();
  /*Serial.print(c_state.spd_left);
  Serial.print("\t");
  Serial.println(c_state.spd_right);
  int i;
  for(i = 0; i < PACKET_SIZE-2; i++){ 
    Serial.print((int)buf[i]);
    Serial.print("|");
  }
  Serial.println("");
  Serial.println("");*/
  sendPacket(buf);
  delay(10);
 }
