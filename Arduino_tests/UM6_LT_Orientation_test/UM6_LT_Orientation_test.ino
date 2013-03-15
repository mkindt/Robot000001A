
// Code uploaded to the Robotics class wiki by Dr Brock --
// changes since then: modifications of mag readings to find degrees
//-----------------------------------
// This sets the communication configuration so that the UM6
// is in listen mode and will send acceleration and magnetic
// readings in response to a UM6_GET_DATA command.

// Unfortunately, there is still a problem with reading packets.
// Sometimes the program gets out of sync with the input.
#include <math.h> 
// This is the UART where the UN6 is connected 
#define ORIENTSERIAL Serial1
// This is the UART where information is sent
#define INFOSERIAL   Serial

#define UM6_COMMUNICATION           0x00
#define UM6_GYRO_PROC_XY            0x5C
#define UM6_GYRO_PROC_Z             0x5D
#define UM6_ACCEL_PROC_XY           0x5E
#define UM6_ACCEL_PROC_Z            0x5F
#define UM6_MAG_PROC_XY             0x60
#define UM6_MAG_PROC_Z              0x61
#define UM6_EULER_PHI_THETA         0x62
#define UM6_EULER_PSI               0x63
#define UM6_GET_FW_VERSION          0xAA
#define UM6_GET_DATA                0xAE
#define UM6_BAD_CHECKSUM            0xFD
#define UM6_UNKNOWN_ADDRESS         0xFE
#define UM6_INVALID_BATCH_SIZE      0xFF

unsigned char packetUM[67] ; // max size of a packet
int lengthUM = 0;            // number of bytes used within packetUM
int expectedUM ;             // number of bytes expected for packet
                             // valid only if lengthUM >= 4
int angle;
float heading;
// Returns
//   1 if packet read is successful
//   0 if packet is incomplete

const char SNP[] = "snp" ;

int readSNPandPT(void) {
  int lengthToRead = 4 - lengthUM ;
  if (lengthToRead < 0) {
    // Already have enough
    return 1 ;
  }
  ORIENTSERIAL.setTimeout(10) ;
  int numRead = ORIENTSERIAL.readBytes((char *)&packetUM[lengthUM], lengthToRead) ;
  if (numRead != lengthToRead) {
    // Will need to read more later
    lengthUM += numRead ;
    return 0 ;
  }
  // Something odd happens
  // Sometimes First char is 0xFF, but ORIENTSERIAL.read() returns -1 if no data
  // Could it be a bug?
  if (packetUM[0] != 's' && !strncmp((char *)&packetUM[1], SNP, 3)) {
    lengthUM = 3 ;
    strncpy((char *)packetUM, SNP, 3) ;
    return 0 ;
  } 
  if (strncmp((char *)packetUM, SNP, 3) == 0) {
    // Have the snp and PT
    unsigned char pt = packetUM[3] ;
    int numRegs ;
    if (pt & 0x80) {      // Has Data
      if (pt & 0x40) {    // Is Batch
        numRegs = (pt & 0x3C) >> 2 ;
      } else {            // not batch
        numRegs = 1 ;
      }
    } else {              // no data
      numRegs = 0 ;
    }
    expectedUM = 7 + 4*numRegs ;
    lengthUM = 4 ;
    return 1 ;
  }
  // The hack -- Don't think this has ever been reached
  Serial.println("*** Unexpected characters") ;
  Serial.print("***") ;
  int snpPos ;
  for (snpPos=1;
       snpPos<=3 && strncmp((char *)&packetUM[snpPos], SNP, 4-snpPos);
       ++snpPos) ;
  for (int i=0; i<snpPos; ++i) {
    Serial.write(' ') ;
    Serial.print(packetUM[i], HEX) ;
  }
  Serial.println() ;
  lengthUM = 4-snpPos ;
  strncpy((char *)packetUM, SNP, lengthUM) ;
  return 0 ;
}

void dumpPacket(void) {
  for (int i=0; i<lengthUM; ++i) {
    Serial.print(packetUM[i], HEX) ;
    Serial.print(" [") ;
    Serial.write(packetUM[i]) ;
    Serial.print("] ") ;
  }
  Serial.println() ;
}

int readPacket(void) {
  if (!readSNPandPT()) {
    return 0 ;
  }
  int lengthToRead = expectedUM - lengthUM ;
  if (lengthToRead == 0) {
    return 1 ;
  }
  ORIENTSERIAL.setTimeout(10) ;
  int numRead = ORIENTSERIAL.readBytes((char *)&packetUM[lengthUM], lengthToRead) ;
  lengthUM += numRead ;
  if (numRead < lengthToRead) {
    return 0 ;
  } 
  unsigned int packetsum = 0 ;
  for (int i=0; i<lengthUM-2; ++i) {
    packetsum += packetUM[i] ;
  }
  unsigned int checksum = (packetUM[lengthUM-2] << 8) + packetUM[lengthUM-1] ;
  if (checksum != packetsum) {
    Serial.print("*** bad checksum: ") ;
    Serial.print(checksum) ;
    Serial.print(" != ") ;
    Serial.println(packetsum) ;
    Serial.print("*** ") ;
    dumpPacket() ;
    Serial.println() ;
    lengthUM = 0 ;
    return 0 ;
  }
  return 1 ;
}

struct packetInfo {
  unsigned char add ;
  unsigned char pt ;
  int numRegs ;
  unsigned char reg[12][4] ;
} ;

void parsePacket(struct packetInfo *p) {
  p->add = packetUM[4] ;
  p->pt  = packetUM[3] ;
  int rPos = 5 ;
  int regNum = 0 ;
  for (int rPos = 5; rPos < lengthUM-2 ; rPos += 4) {
    for (int j=0; j<4; ++j) {
      p->reg[regNum][j] = packetUM[rPos+j] ;
    }
    ++regNum ;
  }
  p->numRegs = regNum ;
}

struct pos3dInfo {
  int x ;
  int y ;
  int z ;
} ;

struct orientationInfo {
  unsigned char responders ;
  struct pos3dInfo accel ;
  // pos3dInfo gyro ;
  struct pos3dInfo mag ;
} ;

void initOrientation(struct orientationInfo *o) {
  o->responders = 0 ;
}

int orientationDataReady(struct orientationInfo *o) {
  return o->responders == 0x0F ;    
}

void printOrientation(struct orientationInfo *o) {
  Serial.print("{\"accel\":{\"x\":") ;
  Serial.print(o->accel.x) ;
  Serial.print(",\"y\":") ;
  Serial.print(o->accel.y) ; 
  Serial.print(",\"z\":") ;
  Serial.print(o->accel.z) ; 
  Serial.print("},\"mag\":{\"x\":") ;
  Serial.print(o->mag.x) ;
  Serial.print(",\"y\":") ;
  Serial.print(o->mag.y) ; 
  Serial.print(",\"z\":") ;
  Serial.print(o->mag.z) ;
  Serial.println("}}") ;
  heading = atan2(o->mag.y , o->mag.x);
   //if(heading < 0){
   // heading += 2*PI;
   //}
   angle = heading * 180/M_PI;
   if (o->mag.y > 0) {
     angle = 90 - angle;
   }
   else if (o->mag.y < 0) {
        angle = 270 - angle;
   }
   Serial.print(angle) ;
  Serial.println("}}") ;
  o->responders = 0 ;
}

int twosComp2int(const unsigned char *pair) {
  return  (int)(((unsigned int)pair[0]<<8) + pair[1]) ;
}

void processPacket(struct packetInfo *p, struct orientationInfo *o) {
  if (p->pt & 0x80) {               /* Has data */
    for (int i=0; i<p->numRegs; ++i) {
      unsigned char  regNum = p->add + (unsigned char)i ;
      unsigned char *regVal = p->reg[i] ;
      int A = twosComp2int(&regVal[0]) ;
      int B = twosComp2int(&regVal[2]) ;
      switch (regNum) {
      case UM6_ACCEL_PROC_XY:
        o->accel.x = A ;
        o->accel.y = B ;
        o->responders |= 0x1 ;
        break ;
      case UM6_ACCEL_PROC_Z:
        o->accel.z = A ;
        o->responders |= 0x2 ;
        break ;
      case UM6_MAG_PROC_XY:
        o->mag.x = A ;
        o->mag.y = B ;
        o->responders |= 0x4 ;
        break ;
      case UM6_MAG_PROC_Z:
        o->mag.z = A ;
        o->responders |= 0x8 ;
        break ;
      }
    }
  }
}

void sendCommand(unsigned char *commBuff, int commSize) {
  unsigned int chksum = 0 ;
  for (int i=0; i<commSize-2; ++i) {
    chksum += commBuff[i] ;
  }
  commBuff[commSize-2] = (chksum>>8) & 0xFF ;
  commBuff[commSize-1] = chksum & 0xFF ;
  unsigned char packAdd = commBuff[4] ;

  int numberWritten = 0 ;
  while (numberWritten<commSize) {
    int writOut = ORIENTSERIAL.write(&commBuff[numberWritten],
                                     commSize-numberWritten) ;
    ORIENTSERIAL.flush() ;
    numberWritten += writOut ;
  }

}

void readReg(int regAddr) {
  unsigned char commBuff[7] ;
  strncpy((char *)commBuff, SNP, 3) ;
  commBuff[3] = 0 ;
  commBuff[4] = regAddr ;
  sendCommand(commBuff, 7) ;
}

void turnOffBroadcast() {         // pp. 29-30
  unsigned char commBuff[11] ;
  strncpy((char *)commBuff, SNP, 3) ;
  commBuff[3] = 0x80 ;            // Has Data & !Is Batch
  commBuff[4] = UM6_COMMUNICATION ;
  commBuff[5] = 0x03 ;            // Disabling broadcast, enabling acc, mag 
  // commBuff[5] = 0x44 ;            // Disabling ....
  commBuff[6] = 0x00 ;
  commBuff[7] = 0x05 ;            // Baudrate = 115200
  commBuff[8] = 0x00 ;            // Broadcast 20Hz (disabled)
  sendCommand(commBuff, 11) ;
}  
  
unsigned long nextPolltime ;
struct orientationInfo orientation ;

#define POLLPERIOD 1000

void setup() {
  ORIENTSERIAL.begin(115200) ;
  Serial.begin(57600) ;
  turnOffBroadcast() ;
  nextPolltime = POLLPERIOD ;
  initOrientation(&orientation) ;
}



void loop() {
  struct packetInfo r ;
  int result = readPacket() ;
  if (result) {
    parsePacket(&r) ;
    processPacket(&r, &orientation) ;
    lengthUM = 0 ;
  }
  if (orientationDataReady(&orientation)) {
    printOrientation(&orientation) ;
  }
  unsigned long now = millis() ;
  if (now > nextPolltime) {
    readReg(UM6_GET_DATA) ;
    nextPolltime += POLLPERIOD ;
  }
}


