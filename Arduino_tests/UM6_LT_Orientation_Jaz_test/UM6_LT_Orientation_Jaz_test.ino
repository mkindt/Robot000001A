//AUTHOR: Jaz S
//Code for Arduino Uno to retrieve data from CH Robotics IMU
//Bytes sent from IMU is recieved in the following order (1st to last): B3, B2, B1, B0.

#include <SoftwareSerial.h>

#define Tx    19//3 //18
#define Rx    18//2  //19
#define BAUD  57600

#define REG_GYRO_PROC_XY   0x5C
#define REG_GYRO_PROC_Z    0x5D

#define GYRO_SCALE_FACTOR  0.0109863    // Convert register data to Degrees per second

/*PT byte we're sending out... */
#define PT_HAS_DATA        0x80  //10000000
#define PT_IS_BATCH        0x40  //01000000
#define PT_BATCH_LEN       0x08  //Batch length = 1 

SoftwareSerial UM6Data(Rx, Tx); //pin2 is Rx; pin3 is Tx

struct IMU{ 
    int in;
}phi, theta, psi; //Roll, pitch and yaw, respectively.

void setup(){
  Serial.begin(BAUD);   //Sets baud rate for communication with PC.
  UM6Data.begin(BAUD);
}

void loop(){
  
  POLL_AGAIN:
  poll_UM6_gyro();
  read_UM6_gyro();  //Needs to be called before being used in the 'if' statement. 
   
  if (read_UM6_gyro() == true)  {
    phi.in = phi.in*GYRO_SCALE_FACTOR;
    theta.in = theta.in*GYRO_SCALE_FACTOR;
    psi.in = psi.in*GYRO_SCALE_FACTOR;
  }
  else  {
    goto POLL_AGAIN; 
  }
  
}//end VOID

void poll_UM6_gyro(){  //This void function makes a request for a packet from the IMU
  Serial.print("\nPolling the UM6...\n");
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN) + REG_GYRO_PROC_XY; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  
  UM6Data.write('s');
  UM6Data.write('n');
  UM6Data.write('p');
  UM6Data.write(PT_IS_BATCH | PT_BATCH_LEN);
  UM6Data.write(REG_GYRO_PROC_XY);
  UM6Data.write(chksum1);
  UM6Data.write(chksum0);
}  //end poll_UM6_gyro

boolean read_UM6_gyro(){
  Serial.print("Attempting to read IMU...\n");
  phi.in = 0;
  theta.in = 0;
  psi.in = 0;
  
  unsigned int c = 0;
  unsigned int data[5] = {0};
  unsigned long data_sum = 0; 
  byte blank = 0, temp = 0;
  byte chksum1 = 0, chksum0 = 0;
  unsigned int chksum = 0;
  
  //If there's data in the serial temp register and we haven't finished retrieving data...
  if ((UM6Data.available() > 0)){
    c = UM6Data.read();
    if ((c == 's')){
      c = UM6Data.read();
      if ((c == 'n')){
        c = UM6Data.read();
        if ((c == 'p')) {
          c = UM6Data.read();
          if (c == (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN)){
            c = UM6Data.read();
            if (c == REG_GYRO_PROC_XY)  {
              for (byte i = 0; i < 6; i++)  {
                data[i] = UM6Data.read();
                data_sum += data[i];
              }
              blank = UM6Data.read();
              blank = UM6Data.read();
              chksum1 = UM6Data.read();
              chksum0 = UM6Data.read();            
              chksum = (chksum1 << 8) | chksum0;
              if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN) + REG_GYRO_PROC_XY + phi.in + theta.in + psi.in)){
                Serial.println("Checksum is good. Returning true!");
                phi.in = (data[1] | (data[0] << 8));
                theta.in = (data[3] | (data[2] << 8));
                psi.in = (data[5] | (data[4] << 8));
                return true;
              }
              else {
                Serial.println("Flushing buffer...!");
                FLUSH:
                while ((UM6Data.available() > 0))  {
                blank = UM6Data.read();
                }
                return false;
              }
            }
            else  {
             goto FLUSH;
            }
          }
          else  {
           goto FLUSH; 
          }
        }
        else  {
         goto FLUSH; 
        }
      }
      else  {
       goto FLUSH; 
      }
    }
    else {
     goto FLUSH; 
    }
  }//end if
  else if (UM6Data.available() == 0)  {  //Requested data never arrived...
    Serial.println("Data never arrived....");
    return false;
  }  //end else
  else  {
    Serial.println("Unexpected error.");
    return false;
  }
}  //end read_UM6_gyro
