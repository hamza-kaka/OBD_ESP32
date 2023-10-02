#include <HardwareSerial.h>
#include "EEPROM.h" 
#define EEPROM_SIZE 512

uint8_t canBeginCount=0;
uint8_t extended_odo_retires =0; 
uint8_t ext_baud=0;

char body[200];
String cantype;

char rxResult[50];

boolean rxFlag;
//////////
String txIdCan;
char txExtCan;
char txPayCan[8];

#define pidDelay 100  //100 ideal milliSecond
void pidDataTask(void *pvParameters);

TaskHandle_t TaskHandle_3;

String rxCom;

#include "PIDs.h"
#include"btDataPrint.h"////Bluetooth

enum {
  EXT_500,
  EXT_250,
  N_500,
  N_250
} EXT_BAUD;

void canAuto();
void setup()
  {
    Serial.begin(115200); 
    Serial.print("setup() running on core ");
    Serial.println(xPortGetCoreID());
    if(!EEPROM.begin(EEPROM_SIZE)){Serial.println("failed to initialise EEPROM"); delay(1000000);}
    pidEnableFlag=1; ///enable PID data
    CAN0.setCallback(0,callback);/////initlize
    Serial.println("init TASK");

    xTaskCreatePinnedToCore
    (
      pidDataTask, /* Function to implement the task */
      "PID Data TASK", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      24,  /* Priority of the task */
      &TaskHandle_3,  /* Task handle. */
      0
    ); /* Core where the task should run */
  }

void loop()
  {

  }

void canGet(void)
  {
    if(rxFlag==HIGH){requestCar_custom(reg0,reg1,reg2,reg3,reg4);}vTaskDelay(pidDelay);

    if (NORMAL_OP)
    {
      fuelStatus=0;requestCar(CANPID_FUEL);                  vTaskDelay(pidDelay);Serial.print(".");///0x03
      engineload=0;requestCar(CANPID_ENGINELOAD);            vTaskDelay(pidDelay);//Serial.print(".");
      coolantTemp=0;requestCar(CANPID_COOLANT_TEMP);         vTaskDelay(pidDelay);//Serial.print(".");
      sf1=0;requestCar(CANPID_st1);                          vTaskDelay(pidDelay);//Serial.print(".");//short term fuel trim b1
      lf1=0;requestCar(CANPID_lt1);                          vTaskDelay(pidDelay);//Serial.print(".");//long  term fuel trim b1
      sf2=0;requestCar(CANPID_st2);                          vTaskDelay(pidDelay);//Serial.print(".");//short term fuel trim b2
      lf2=0;requestCar(CANPID_lt2);                          vTaskDelay(pidDelay);//Serial.print(".");//long  term fuel trim b2
      gasPressure=0;    requestCar(CANPID_GAUGE_PRESSURE);   vTaskDelay(pidDelay);//Serial.print(".");   ///gauge pressure///0x0A
      manifold_press=0; requestCar(CANPID_MANIFOLD_PRESSURE);vTaskDelay(pidDelay);//Serial.print("."); ///manifold Pressure
      rpm=0;            requestCar(CANPID_RPM);              vTaskDelay(pidDelay);//Serial.print(".");
      vehicle_speed=0;  requestCar(CANPID_SPEED);            vTaskDelay(pidDelay);//Serial.print("."); ///vehicle speed
      timing_adv=0;     requestCar(CANPID_TIMING_ADV);       vTaskDelay(pidDelay);//Serial.print("."); ///timming advace///0x0E
      intakeTemp=0;     requestCar(CANPID_INTAKE_TEMP);      vTaskDelay(pidDelay);//Serial.print(".");///intake Temperature///0x0f
      maf=0;            requestCar(CANPID_MAF);              vTaskDelay(pidDelay);//Serial.print(".");//Mass Air Flow          //0x10
      throttle=0;       requestCar(CANPID_THROTTLE);         vTaskDelay(pidDelay);//Serial.print(".");///Throttle Position //0x11
      fuelT=0;          requestCar(CANPID_FUEL_TYPE);                 vTaskDelay(pidDelay);//Serial.print(".");///fuel type //0x51
      hybridBattRem=0;  requestCar(CANPID_HYBRID_BATT_REMAIN);        vTaskDelay(pidDelay);//Serial.print(".");//hybrid batt remaining 0x5B
      odometer=0;       //requestCar_odo(CANPID_ODOMETER);         vTaskDelay(pidDelay);//Serial.print(".");///odoMeter ///0xA6
      hybrid_Bvolt=0;   requestCar_four_byte(CANPID_HYBRID_BATT_VOLT);vTaskDelay(pidDelay);//Serial.print(".");
      fuel_percentage=0;requestCar_four_byte(CANPID_FUEL_PER);        vTaskDelay(pidDelay);//Serial.print(".");///fuel Percentage 0x9F
      veh_bat=0;requestCar_four_byte(CANPID_VEH_BAT);                 vTaskDelay(pidDelay);//Serial.print(".");///vehical Battery 0x42
      ////////
      //Serial.println("\nSend ODO METER Request");
      if(odoMeterFlag==0){odoMeterFlag=1;Serial.print("odometerflag       ");Serial.println(odoMeterFlag);requestCar_odometer(odoMeterFlag);vTaskDelay(pidDelay);}
      if(odoMeterFlag==2){requestCar_odometer(odoMeterFlag);vTaskDelay(pidDelay);}//Serial.print(".");
      
      //Serial.println("\nSend VIN  Request");
      memset(vin_arr,0,sizeof(vin_arr));///for removing garbage value
      if(vinFlag==0){vinFlag=1;requestCar_VIN(0);vTaskDelay(pidDelay);}
      if(vinFlag==2){requestCar_VIN(0);vTaskDelay(pidDelay);}//Serial.print(".");
      ////////////////////////////////////////
      ///////////////////////////////////////////////
    if(dtcFlag==1){requestCar_DTC(0, 0x03);requestCar_DTC(0, 0x07);requestCar_DTC(0, 0x0A);}
      //if(dtcClear==1){dtcClear = 0;requestCar_DTC_clear();}
    }
   
            
    for (int counter=0; counter<10;counter++)
    vTaskDelay(pidDelay);
    odo_can_id_counter++;
    if ( odo_can_id_counter == 7)
    {
      odo_can_id_counter=0;
    }
    
  }
  
void canAuto()
  {
    Serial.println("canauto");
    // Serial.print("rpm:");Serial.println(rpm);
    // Serial.print("coolant temp :");Serial.println(coolantTemp);
    rpm = 0;
    coolantTemp = 0;
    
    if(rpm==0 && coolantTemp==0)  
    {
      Serial.println("Auto Enable");
      autoFlag=1;
      canBeginCount=0;
      vinFlag=0;
      Serial.println("Searching CAN Protocol");
      cantype="CAN Fail";
    }
    String canSpeed="not allocated";
    vTaskDelay(pidDelay);

    while(autoFlag==1)
      {
        if(rpm==0 && coolantTemp==0)
          {
            canBeginCount++;
            Serial.print(canBeginCount);
            vTaskDelay(pidDelay);
            if(canBeginCount==1)
              {
                CAN0.begin(CAN_BPS_500K);
                CAN0.watchForRange(CAN_REPLY_ID_EXT,CAN_REPLY_ID_EXT_FINAL);
                EXTENDED_FLAG=1;
                requestCarFirst(CANPID_RPM);vTaskDelay(pidDelay);
                requestCarFirst(CANPID_COOLANT_TEMP);vTaskDelay(pidDelay);
                canSpeed = "29B/500K";
                ext_baud = EXT_500;
              }
            if(canBeginCount==2)
              {
                CAN0.begin(CAN_BPS_250K);CAN0.watchForRange(CAN_REPLY_ID_EXT, CAN_REPLY_ID_EXT_FINAL);
                EXTENDED_FLAG = 1;
                requestCarFirst(CANPID_RPM);vTaskDelay(pidDelay);
                requestCarFirst(CANPID_COOLANT_TEMP);vTaskDelay(pidDelay);
                canSpeed = "29B/250K";
                ext_baud = EXT_250;
              }
            if(canBeginCount==3)
              {
                CAN0.begin(CAN_BPS_250K);CAN0.watchForRange(0x611, CAN_REPLY_ID );//CAN0.watchFor(CAN_REPLY_ID); 
                EXTENDED_FLAG = 0;
                requestCarFirst(CANPID_RPM);vTaskDelay(pidDelay);
                requestCarFirst(CANPID_COOLANT_TEMP);vTaskDelay(pidDelay); 
                canSpeed = "11B/250K";
                ext_baud = N_250;
              }
            if(canBeginCount==4)
              {
                CAN0.begin(CAN_BPS_500K);CAN0.watchForRange(0x611, CAN_REPLY_ID );//CAN0.watchFor(CAN_REPLY_ID);
                EXTENDED_FLAG = 0;
                requestCarFirst(CANPID_RPM);vTaskDelay(pidDelay); 
                requestCarFirst(CANPID_COOLANT_TEMP);vTaskDelay(pidDelay);
                canSpeed = "11B/500K";
                ext_baud = N_500;
              }
            if(canBeginCount>4)
              {
                canSpeed = "CAN Fail";
                //autoFlag=0;
                canBeginCount=0;//Start searching Again
              }
          }
        else
          {
            Serial.println();
            Serial.print("CoolT : ");Serial.println(coolantTemp);
            Serial.print("RPM   : ");Serial.println(rpm);

            //Serial.println("SET CALL BACK");
            //CAN0.setCallback(0,callback);/////initlize

            Serial.println("Communication Successful");
            Serial.println(canSpeed);
            cantype=canSpeed;
            autoFlag=0;
          }
      }
      Serial.println(canSpeed);
        //Serial.print("  Can Connection Type: ");Serial.println(coun);
  }

void pidDataTask(void *pvParameters)
  {
    Serial.println("Start PID TASK");
    //int canCount = 0;
    while(1)
      {//if(canCount==50){canCount = 0;      
        Serial.println("CAN................");
        //pidDataFun();
        ///Serial.print("pid Task running on core ");Serial.println(xPortGetCoreID());
        canAuto();
        if(pidEnableFlag==HIGH)
          {///get can value every loop
            //pidEnableFlag=LOW;
            Serial.print("CAN Type :");Serial.println(cantype);
            Serial.print("rxFlag = ");Serial.println(rxFlag);
            if(rxFlag==HIGH)
              {
                customCommandDecode(rxCom);
              }
            //vTaskDelay(100);
            Serial.println("CANGet started");
            canGet();
            //coolantTemp=0;requestCar(CANPID_COOLANT_TEMP);         vTaskDelay(pidDelay);Serial.print(".");
            vTaskDelay(100);
            //customCommandEncode();
            Serial.println("pidSerialPrint");
            pidSerialPrint();//}canCount++; 
            odoMeterFlag=0;
          }
        vTaskDelay(5000);
      }
  }

// void inReadTask(void *pvParameters)
//   {
//     while(1){
//     int in1Val=digitalRead(in1);

//     if(in1Val==LOW){//Serial.println("IN");
//       digitalWrite(out1, HIGH); ///ON
//       digitalWrite(out2, HIGH);///ON
//     }else{
//       digitalWrite(out1,LOW); ///ON
//       digitalWrite(out2,LOW);///ON  
//     }
//     vTaskDelay(1000);
//     }
//   }

