#include <motor_define.h>
#include <mrf24j.h>
#include <motor_controller_comms.h>

#define HWSERIAL Serial1


/* Joystick*/
#define V_PIN 16
#define H_PIN 15
#define EDI_PIN 8
#define R_PIN 7

#define JOY_THRESH 25

#define V_CTR 513
#define H_CTR 516

#define DEBOUNCE_TIME_MS 200

/* SPI */
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13
#define CS_PIN   10
#define INT_PIN  18
#define WAKE_PIN 17
#define RST_PIN  14

#define MY_ADDR 0x4001
#define THEIR_ADDR 0x5000
#define WIRELESS_CHANNEL 0x14

long last_time;
long tx_interval = 1000;

float vel_l = 0.0;
float vel_r = 0.0;
float spd_l;
float spd_r;


unsigned char buf[PACKET_SIZE];

Mrf24j mrf(RST_PIN, CS_PIN, INT_PIN);

void send_packet_wireless() {
  char temp[PACKET_SIZE + 1];
  int i;
  
  for(i = 0; i < PACKET_SIZE; i++) {
    temp[i] = (char)buf[i];
  }
  temp[PACKET_SIZE] = '/0';
  mrf.send16(THEIR_ADDR, temp);
  
 /* for(i = 0; i < PACKET_SIZE + 1; i++) {
    Serial.print((int)temp[i]);
    Serial.print('|');
  }
  Serial.println("");*/
}

void mrf_isr() {
  mrf.interrupt_handler();
}

void get_vel_and_spd() {
  float diff_v = analogRead(V_PIN) - V_CTR;
  float diff_h = analogRead(H_PIN) - H_CTR;
  
  vel_l = 0.0;
  vel_r = 0.0;
  
  if(abs(diff_h) > JOY_THRESH) {
    vel_l = diff_h/30;
    vel_r = -diff_h/30;
  }
  if(abs(diff_v) > JOY_THRESH) {
    vel_l += diff_v / 40;
    vel_r += diff_v / 40;
  }
  spd_l = abs(vel_l);
  spd_r = abs(vel_r);
}

void make_packet() {
  if(spd_l > .02) {
    buf[0] = vel_l > 0.0 ? 1 : 2;
    buf[1] = (int) spd_l;
    buf[2] = (int)((spd_l - buf[1]) * 100);
  }
  if(spd_r > .02) {
    buf[3] = vel_r > 0.0 ? 1 : 2;
    buf[4] = (int) spd_r;
    buf[5] = (int)((spd_r - buf[4]) * 100);
  }
  else {
    buf[0] = buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = 0.0;
  }
}


void handle_rx() {
  
}


void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}


void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200); 
  buf[8] = 'f';
  buf[9] = 'g';

  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe);
  mrf.set_channel(WIRELESS_CHANNEL);
  mrf.address16_write(MY_ADDR);
  
  attachInterrupt(INT_PIN, mrf_isr, CHANGE);
  interrupts();
  
  pinMode(EDI_PIN, INPUT);
  pinMode(R_PIN, INPUT);
}


/* 177 us to send packet over wirelss vs 35-ish on serial */
void loop() {
  get_vel_and_spd();
  make_packet_vels(buf, vel_l, vel_r);
  buf[7] = 0;
  if(digitalRead(EDI_PIN)) {
    buf[7] |= 0x03;
  }
  send_packet_wireless();
  
  mrf.check_flags(&handle_rx, &handle_tx);
  delay(10);
 }
