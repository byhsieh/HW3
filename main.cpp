#include "mbed.h"

#include "mbed_events.h"

#include "fsl_port.h"

#include "fsl_gpio.h"

#include "math.h"

#include "iostream"

using namespace std;

#define UINT14_MAX        16383

// FXOS8700CQ I2C address

#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0

#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0

#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1

#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1

// FXOS8700CQ internal register addresses

#define FXOS8700Q_STATUS 0x00

#define FXOS8700Q_OUT_X_MSB 0x01

#define FXOS8700Q_OUT_Y_MSB 0x03

#define FXOS8700Q_OUT_Z_MSB 0x05

#define FXOS8700Q_M_OUT_X_MSB 0x33

#define FXOS8700Q_M_OUT_Y_MSB 0x35

#define FXOS8700Q_M_OUT_Z_MSB 0x37

#define FXOS8700Q_WHOAMI 0x0D

#define FXOS8700Q_XYZ_DATA_CFG 0x0E

#define FXOS8700Q_CTRL_REG1 0x2A

#define FXOS8700Q_M_CTRL_REG1 0x5B

#define FXOS8700Q_M_CTRL_REG2 0x5C

#define FXOS8700Q_WHOAMI_VAL 0xC7

#define PI 3.14159265


I2C i2c( PTD9,PTD8);

Serial pc(USBTX, USBRX);

DigitalOut led(LED1);

InterruptIn btn(SW2);


EventQueue eventQueue;

EventQueue printfQueue;

int m_addr = FXOS8700CQ_SLAVE_ADDR1;


void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);

void FXOS8700CQ_writeRegs(uint8_t * data, int len);


void tilt_event_logger() {


   pc.baud(115200);


   uint8_t who_am_i, data[2], res[6];

   int16_t acc16;

   float t[3];


   // Enable the FXOS8700Q


   FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);

   data[1] |= 0x01;

   data[0] = FXOS8700Q_CTRL_REG1;

   FXOS8700CQ_writeRegs(data, 2);


   // Get the slave address

   FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);


   // pc.printf("Here is %x\r\n", who_am_i);

   while (true) {


      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);


      acc16 = (res[0] << 6) | (res[1] >> 2);

      if (acc16 > UINT14_MAX/2)

         acc16 -= UINT14_MAX;

      t[0] = ((float)acc16) / 4096.0f;


      acc16 = (res[2] << 6) | (res[3] >> 2);

      if (acc16 > UINT14_MAX/2)

         acc16 -= UINT14_MAX;

      t[1] = ((float)acc16) / 4096.0f;


      acc16 = (res[4] << 6) | (res[5] >> 2);

      if (acc16 > UINT14_MAX/2)

         acc16 -= UINT14_MAX;

      t[2] = ((float)acc16) / 4096.0f;

      float xangle = atan(t[0] / (sqrt(pow(t[1],2) + pow(t[2],2))));
      float yangle = atan(t[1] / (sqrt(pow(t[0],2) + pow(t[2],2))));
      float zangle = atan((sqrt(pow(t[0],2) + pow(t[1],2))) / t[2]);

      float Pitch = xangle * 180 / PI;
      float Roll = yangle * 180 / PI;
      float Yaw = zangle * 180 / PI;
      
      int time_interval = int(10.0/0.1);
      int time_counter = 0;
      float tilt_event0 = 0.0;
      float tilt_event1 = 1.0;

      if ( time_counter <= time_interval ){
         if ( Pitch > 45.0 || Roll > 45.0 || Yaw > 45.0 ){

            pc.printf("%1.4f %1.4f %1.4f %1.1f\r\n",t[0],t[1],t[2],tilt_event1);

            // pc.printf("FXOS8700Q Angle: Pitch_X=%2.2f Roll_Y=%2.2f Yaw_Z=%2.2f \r\n",Pitch,Roll,Yaw);

            // pc.printf("%1.1f\r\n",tilt_event1);

            time_counter++;

            wait(0.1);

         }
         else {
            
            pc.printf("%1.4f %1.4f %1.4f %1.1f\r\n",t[0],t[1],t[2],tilt_event0);

            // pc.printf("%1.1f\r\n",tilt_event0);

            time_counter++;

            wait(0.1);

         }
         
      }
      
   }

}

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {

   char t = addr;

   i2c.write(m_addr, &t, 1, true);

   i2c.read(m_addr, (char *)data, len);

}


void FXOS8700CQ_writeRegs(uint8_t * data, int len) {

   i2c.write(m_addr, (char *)data, len);

}


void blink_led()  {
  
   led = !led;

}

void btn_fall_irq() {

   eventQueue.call_every(500,blink_led);
 
   printfQueue.call(&tilt_event_logger);

}

int main() {

  

  Thread printfThread;

  printfThread.start(callback(&printfQueue, &EventQueue::dispatch_forever));


  Thread eventThread;

  eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));


  btn.fall(&btn_fall_irq);

  while (1) {wait(1);}
}