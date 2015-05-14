#include <motor_define.h>
#include <mrf24j.h>
#include <motor_controller_comms.h>

#define HWSERIAL Serial1
#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

/* Joystick*/
#define V_PIN 16
#define H_PIN 15
#define L_PIN 8
#define R_PIN 7

#define JOY_THRESH 50

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

/* LEDs */
#define ORG_LED 13
#define GRN_LED 12
#define RED_LED 11

long last_time;
long tx_interval = 1000;


unsigned char buf[PACKET_SIZE];

Mrf24j mrf(RST_PIN, CS_PIN, INT_PIN);

void mrf_isr() {
  mrf.interrupt_handler();
}


/* Maybe want to separate sending data fucntion from wrapping it properly */
void send_packet_wireless() {
  char temp[PACKET_SIZE + 1];
  int i;
  
  for(i = 0; i < PACKET_SIZE; i++) {
    temp[i] = buf[i];
  }
  temp[PACKET_SIZE] = '/0';
  mrf.send16(THEIR_ADDR, temp);
}


float vel_l = 0.0;
float vel_r = 0.0;


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
  else if(digitalRead(L_PIN)) {
    vel_l = 3.0;
    vel_r = -3.0;
  }
  else if(digitalRead(R_PIN)) {
    vel_l = -3.0;
    vel_r = 3.0;
  }
}


/*void set_leds() {
  if(spd_l > .02 || spd_r > .02) {
    digitalWrite(GRN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
  else {
    digitalWrite(GRN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
}*/

void handle_rx() {
   /* Serial.print("received a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    if(mrf.get_bufferPHY()){
      Serial.println("Packet data (PHY Payload):");
      for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
          Serial.print(mrf.get_rxbuf()[i]);
      }
    }
    
    Serial.println("\r\nASCII data (relevant data):");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    
    Serial.print("\r\nLQI/RSSI=");
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);*/
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
  buf[6] = 'f';
  buf[7] = 'g';

  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe);
  mrf.set_channel(WIRELESS_CHANNEL);
  mrf.address16_write(MY_ADDR);
  
  attachInterrupt(INT_PIN, mrf_isr, CHANGE);
  interrupts();
  //pinMode(GRN_LED, OUTPUT);
  //pinMode(RED_LED, OUTPUT);
  
  pinMode(L_PIN, INPUT);
  pinMode(R_PIN, INPUT);
}


/* 177 us to send packet over wirelss vs 35-ish on serial */
void loop() {
  get_vel_and_spd();
  make_packet_vels(buf, vel_l, vel_r);
  send_packet_wireless();
  
  /*Serial.print(vel_l);
  Serial.print("\t");
  Serial.println(vel_r);*/
  
  mrf.check_flags(&handle_rx, &handle_tx);
  delay(10);
 }
