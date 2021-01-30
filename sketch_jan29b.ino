

#include <SPI.h>
#include <mcp_can.h>



const int spiCSPin = 10;


#define DEBUG   0


// Motor controller CAN base address and packet offsets
#define MC_CAN_BASE   0x400   // High = Serial Number             Low = "TRIa" string
#define MC_LIMITS   0x01    // High = Active Motor/CAN counts   Low = Error & Limit flags
#define MC_BUS      0x02    // High = Bus Current               Low = Bus Voltage
#define MC_VELOCITY   0x03    // High = Velocity (m/s)            Low = Velocity (rpm)
#define MC_PHASE    0x04    // High = Phase A Current           Low = Phase B Current
#define MC_V_VECTOR   0x05    // High = Vd vector                 Low = Vq vector
#define MC_I_VECTOR   0x06    // High = Id vector                 Low = Iq vector
#define MC_BEMF_VECTOR  0x07    // High = BEMFd vector              Low = BEMFq vector
#define MC_RAIL1    0x08    // High = 15V                       Low = Unused
#define MC_RAIL2    0x09    // High = 3.3V                      Low = 1.9V
#define MC_FAN      0x0A    // High = Reserved                  Low = Reserved
#define MC_TEMP1    0x0B    // High = Heatsink Phase C Temp     Low = Motor Temp
#define MC_TEMP2    0x0C    // High = Heatsink Phase B Temp     Low = CPU Temp
#define MC_TEMP3    0x0D    // High = Heatsink Phase A Temp     Low = Unused
#define MC_CUMULATIVE 0x0E    // High = DC Bus AmpHours           Low = Odometer


#define DC_CAN_BASE     0x500
#define DC_DRIVE        1
#define DC_POWER    2

float rpm;
float current;

uint8_t sendBuf[8];

uint8_t      bytes[sizeof(float)];


MCP_CAN CAN(spiCSPin);

void setup() {
  Serial.begin(115200);

  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);

  
  

  while(CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");

 
  
}

void loop() {


  byte len = 0;
  byte readBuf[8];

  while(CAN_MSGAVAIL ==   CAN.checkReceive())
  {
    int id= CAN.readMsgBuf(&len,readBuf);

    if( CAN.isRemoteRequest() ){

      Serial.print("GELEN:");
      Serial.print(CAN.isExtendedFrame());
      Serial.println(CAN.getCanId(), HEX);

      
    }



    switch( CAN.getCanId()- MC_CAN_BASE){

       case MC_LIMITS:
       {
         
          for(int i=7;i>=0;i--)
          {
            Serial.print(readBuf[i], HEX);
            Serial.print("\t");
          }
        Serial.println();
        
          break;
       }


       case MC_BUS:
       {
        #if DEBUG
          for(int i=7;i>=0;i--)
          {
            Serial.print(readBuf[i], HEX);
            Serial.print("\t");
          }
        Serial.println();
        #endif

          float bus_voltage;
          float bus_current;


          memcpy(&bus_voltage, &readBuf[0], sizeof(float));
          memcpy(&bus_current, &readBuf[4], sizeof(float));

#if DEBUG
          Serial.print("BUS_VOLTAGE:");
          Serial.println(bus_voltage);
          Serial.print("BUS_CURRENT");
          Serial.println(bus_current);
#endif
          break;
       }


       case MC_VELOCITY:
       {
        #if DEBUG
          for(int i=7;i>=0;i--)
          {
            Serial.print(readBuf[i], HEX);
            Serial.print("\t");
          }
        Serial.println();
        #endif

          float vehicle_velocity;
          float motor_velocity;

          memcpy(&motor_velocity, &readBuf[0], sizeof(float));
          memcpy(&vehicle_velocity, &readBuf[4], sizeof(float));

#if DEBUG
          Serial.print("MOTOR VELOCITY:");
          Serial.println(motor_velocity);
          Serial.print("VEHICLE_VELOCITY:");
          Serial.println(vehicle_velocity);
#endif
          break;
       }


       case MC_PHASE:
       {
         #if DEBUG
          for(int i=7;i>=0;i--)
          {
            Serial.print(readBuf[i], HEX);
            Serial.print("\t");
          }
          
          Serial.println();
        #endif

          float phase_c_current;
          float phase_b_current;

          memcpy(&phase_b_current, &readBuf[0], sizeof(float));
          memcpy(&phase_c_current, &readBuf[4], sizeof(float));

#if DEBUG
          Serial.print("PHASE C CURRENT:");
          Serial.println(phase_c_current);
          Serial.print("PHASE B CURRENT:");
          Serial.println(phase_b_current);
#endif
          break;
       }

       case MC_BEMF_VECTOR:
       {
         #if DEBUG
          for(int i=7;i>=0;i--)
          {
            Serial.print(readBuf[i], HEX);
            Serial.print("\t");
          }
          
          Serial.println();
        #endif

          float BEMFd;
          float BEMFq;

          memcpy(&BEMFq, &readBuf[0], sizeof(float));
          memcpy(&BEMFd, &readBuf[4], sizeof(float));

 #if DEBUG
          Serial.print("BEMFq:");
          Serial.println(BEMFd);
          Serial.print("BEMFq:");
          Serial.println(BEMFq);

 #endif
          
          break;
       }

       default:
       {
        break;
       }
    
    }
  
 
  } 



  int direct;

  if( digitalRead(8) == HIGH){

    direct =1;
  }
  else{

    direct = -1;
  }

  float tyreMeter=0.5;
  float maxRpm = 20000;

  current =  analogRead(A0)*1.0/1023.0;

  
  rpm = maxRpm * (512-analogRead(A1))*1.0/512.0;

  if( rpm > 0){

    rpm = 4000;
  }

  
  if( rpm < 0){

    rpm = -1500;
  }


  Serial.print("Rpm:");
  Serial.println(rpm);
  Serial.print("Hiz:");
  Serial.println(rpm*tyreMeter/60);

  Serial.print("Akim:");
  Serial.println(current);


  memcpy(&sendBuf[4], &current, sizeof(float));
  memcpy(&sendBuf[0], &rpm, sizeof(float));


  CAN.sendMsgBuf(DC_CAN_BASE + DC_DRIVE, CAN_STDID, 8, &sendBuf[0]);
  
#if DEBUG
  Serial.println("------------------------------");
  Serial.println(sendBuf[0], HEX);  
  Serial.println(sendBuf[1], HEX);
  Serial.println(sendBuf[2], HEX);
  Serial.println(sendBuf[3], HEX);
  Serial.println(sendBuf[4], HEX);  
  Serial.println(sendBuf[5], HEX);
  Serial.println(sendBuf[6], HEX);
  Serial.println(sendBuf[7], HEX);

#endif


  float buscurrent = 1.0;

  sendBuf[0]= 0;
  sendBuf[1]= 0;
  sendBuf[2]= 0;
  sendBuf[3]= 0;

  memcpy(&sendBuf[4], &buscurrent, sizeof(float));


  CAN.sendMsgBuf(DC_CAN_BASE + DC_POWER, CAN_STDID, 8, &sendBuf[0]);
  
#if DEBUG
  Serial.println("------------------------------");
  Serial.println(sendBuf[0], HEX);  
  Serial.println(sendBuf[1], HEX);
  Serial.println(sendBuf[2], HEX);
  Serial.println(sendBuf[3], HEX);
  Serial.println(sendBuf[4], HEX);  
  Serial.println(sendBuf[5], HEX);
  Serial.println(sendBuf[6], HEX);
  Serial.println(sendBuf[7], HEX);

#endif


  int serialNumber = 8070;

  memcpy(&sendBuf[0], &serialNumber, sizeof(uint32_t));
  
  sendBuf[4]='6';
  sendBuf[5]='8';
  sendBuf[6]='0';
  sendBuf[7]='T';


#if DEBUG
  Serial.println("------------------------------");
  Serial.println(sendBuf[0], HEX);  
  Serial.println(sendBuf[1], HEX);
  Serial.println(sendBuf[2], HEX);
  Serial.println(sendBuf[3], HEX);
  Serial.println(sendBuf[4], HEX);  
  Serial.println(sendBuf[5], HEX);
  Serial.println(sendBuf[6], HEX);
  Serial.println(sendBuf[7], HEX);

#endif
  
  CAN.sendMsgBuf(DC_CAN_BASE  , CAN_STDID, 8, &sendBuf[0]);

  delay(250);

}
