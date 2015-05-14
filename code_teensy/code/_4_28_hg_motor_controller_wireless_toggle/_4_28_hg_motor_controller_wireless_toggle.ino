#include <HMotor.h>
#include <motor_define.h>
#include <motor_controller_comms.h>
#include <mrf24j.h>

#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13
#define CS_PIN   10
#define INT_PIN  17
#define WAKE_PIN 16
#define RST_PIN  9

#define MY_ADDR 0x5000
#define THEIR_ADDR 0x4001
#define WIRELESS_CHANNEL 0x14

#define EDI_PIN 8

Mrf24j mrf(RST_PIN, CS_PIN, INT_PIN);

unsigned char buf[PACKET_SIZE];

HMotor motor_left(MOTOR_LEFT);
HMotor motor_right(MOTOR_RIGHT);

int received = 0;

long last_time;
long tx_interval = 1000;

long time_since_received_ms;
long current_time = 0;

#define RCV_WRLSS 1
#define RCV_CEREAL 2

int receive_mode = RCV_WRLSS;

void mrf_isr() {
  mrf.interrupt_handler();
}


void handle_rx() {
    time_since_received_ms = millis();
    if(mrf.get_rxinfo()->frame_length == 18) {
      int i;
      unsigned char *rx_buf = mrf.get_rxinfo()->rx_data;
      for(i = 0; i < PACKET_SIZE; i++) {
        buf[i] = rx_buf[i];
       Serial.print((int)buf[i]);
        Serial.print('|');
      }
      Serial.println();
      //Serial.println(mrf.get_rxinfo()->frame_length);
      if(buf[7] & 0x03) {
        receive_mode = RCV_CEREAL;
      }
      else {
        acceptCommands();
        receive_mode = RCV_WRLSS;
      }
    }
    else {
      //Serial.println("Brake");
      motor_left.setBrake();
      motor_right.setBrake();
    }
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
  Serial.begin(115200);
  
  pinMode(CS_PIN, OUTPUT);
  pinMode(WAKE_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT);
  
  pinMode(EDI_PIN, INPUT);
  
  mrf.reset();
  mrf.init();
  
  mrf.set_channel(WIRELESS_CHANNEL);
  mrf.set_pan(0xcafe);
  mrf.address16_write(MY_ADDR);
  
  attachInterrupt(INT_PIN, mrf_isr, CHANGE);
  interrupts();
  
  motor_left.initMotor();
  motor_left.setBrake();

  motor_right.initMotor();
  motor_right.setBrake();
}

float vel;
long time;



void loop() {
  Serial.println(digitalRead(EDI_PIN));
  motor_left.updateMotor();
  motor_right.updateMotor();
  mrf.check_flags(&handle_rx, &handle_tx);
  if ((current_time = millis()) - time_since_received_ms > 200) {
    mrf.send16(THEIR_ADDR, "abcd");    
    motor_left.setBrake();
    motor_right.setBrake();
    Serial.print('s');
  }
  
  if((current_time = millis()) - time_since_received_ms > 2000) {
    receive_mode = RCV_CEREAL;
  }
  
  switch(receive_mode)
  {
    case RCV_WRLSS:
      digitalWrite(EDI_PIN, LOW);
      Serial.println("WR");
      while(SERIAL_PORT.available()) {
        SERIAL_PORT.read();
      }
      break;
    case RCV_CEREAL:
      Serial.println("SR");
      digitalWrite(EDI_PIN, HIGH);
      if(SERIAL_PORT.available() >= PACKET_SIZE) {
        if(read_in_packet(buf)) {
          acceptCommands();
        }
      }
      break;
    default:
      break;
  }
}



void acceptCommands() {
   if(buf[1] == 0) {
     motor_left.setBrake();
     motor_right.setBrake();
   }
   else {
     if(buf[1] == MOTOR_FWD) {
       motor_left.setDirection(MOTOR_FWD);
       /*Serial.println("ey"); */        }
        else if(buf[1] == MOTOR_BWD) {
          motor_left.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[2] % 23);
        vel += (float)((int)buf[3] % 100)/100;
        //Serial.println(vel);
       //  Serial.print("\t");*/
        motor_left.setVelocity(vel);

        if(buf[4] == MOTOR_FWD) {
          motor_right.setDirection(MOTOR_FWD); 
        }
        else if(buf[4] == MOTOR_BWD) {
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
