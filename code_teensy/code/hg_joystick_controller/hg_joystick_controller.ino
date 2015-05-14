#define HWSERIAL Serial1

#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

char buf[PACKET_SIZE-2];

/* Maybe want to separate sending data fucntion from wrapping it properly */
void sendPacket(char *buf) {
  int i;
  HWSERIAL.write(PACKET_START);
  for(i =  0; i < PACKET_SIZE-2; i++) {
    HWSERIAL.write(buf[i]);
  }
  HWSERIAL.write(PACKET_END);
}

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200); 
  buf[6] = 'f';
  buf[7] = 'g';
  pinMode(13, OUTPUT);
}

void loop() {
  
  if(analogRead(17) > 500) {
    buf[0] = (uint8_t) 1;
    buf[1] = (uint8_t)2;
    buf[2] = (uint8_t)0;
    buf[3] = (uint8_t) 1;
    buf[4] = (uint8_t) 2;
    buf[5] = (uint8_t) 0;
    Serial.println("F");
  }
  else if(analogRead(15) > 500) {
    buf[0] = (uint8_t) 2;
    buf[1] = (uint8_t)5;
    buf[2] = (uint8_t)0;
    buf[3] = (uint8_t) 1;
    buf[4] = (uint8_t) 5;
    buf[5] = (uint8_t) 0;
    Serial.println("L");
  }
  else if(analogRead(16) > 500) {
    buf[0] = (uint8_t) 1;
    buf[1] = (uint8_t)5;
    buf[2] = (uint8_t)0;
    buf[3] = (uint8_t) 2;
    buf[4] = (uint8_t) 5;
    buf[5] = (uint8_t) 0;
    Serial.println("R");
  }
  else {
    buf[0] = (uint8_t) 0;
    buf[1] = (uint8_t)0;
    buf[2] = (uint8_t)0;
    buf[3] = (uint8_t) 0;
    buf[4] = (uint8_t) 0;
    buf[5] = (uint8_t) 0;
    Serial.println("S");
  }
  
  digitalWrite(13,HIGH);
  sendPacket(buf);
  delay(100);
  digitalWrite(13,LOW);
  delay(50);
 }
