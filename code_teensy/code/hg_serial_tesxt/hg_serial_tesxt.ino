#define HWSERIAL Serial1

#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

char buf[PACKET_SIZE];

/* Basic read, checks for proper start byte */
int readInPacket(char *buf) {
  int success = 0;
  int bytes_read = 0;
  char c;
  
  /* Find beginning of packet */ 
  while( HWSERIAL.available() && (c = HWSERIAL.read()) != PACKET_START) {
  Serial.print(c);}
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
  Serial.begin(9600);
  HWSERIAL.begin(9600); 
}

void loop() {
  if(HWSERIAL.available() >= PACKET_SIZE) {
    if(readInPacket(buf)) {
      int i;      
      for(i = 1; i < PACKET_SIZE - 1; i++) {
        Serial.print((int)buf[i]);
        Serial.print("\t");
      }
      Serial.println("");
    }
    else {
      Serial.println("Something sent: failed to read packet or garbage was sent. ");
    }
  }
}

