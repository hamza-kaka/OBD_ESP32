int KLINE_FLAG = 0;//#include "Arduino.h"//#include "OBD9141.h"
#include <esp32_can.h>            // https://github.com/collin80/esp32_can AND https://github.com/collin80/can_common

//#define RX_PIN 16   ///
//#define TX_PIN 17   ///
//#define EN_PIN 4
//HardwareSerial kline(2);
////esp32 Builtin CAN 
////TJA1050_TxD->esp32_Rx(GPIO14)
////TJA1050_RxD->esp32_Tx(GPIO27)

//uint8_t kline_dtc_buf[5];uint8_t kline_pdtc_buf[5];

//OBD9141 obd;
float tem;
#define TESTING 0
#define NORMAL_OP 0
#define SEARCH_FOR_REQ 1
#define TEST_ODO 1
uint8_t resultarray[255][9];
uint64_t odometer_val;
uint8_t odometer_raw[4];
uint32_t odometer_can_id;
uint32_t possible_odo_ids[10];
uint8_t odo_can_id_counter = 0;

uint32_t can_id_odo_check[7] = {0xA6, 0x158, 0x516, 0x611, 0x330, 0x19D, 0x18FEC1FE}; // fist element in array is PID, rest are CAN IDs


boolean canCallBackFlag;
//////////////-Service 01-///////
#define CANPID_FUEL               0x03
#define CANPID_ENGINELOAD         0x04
#define CANPID_COOLANT_TEMP       0x05
#define CANPID_st1                0x06
#define CANPID_lt1                0x07
#define CANPID_st2                0x08
#define CANPID_lt2                0x09
#define CANPID_GAUGE_PRESSURE     0x0A
#define CANPID_MANIFOLD_PRESSURE  0x0B
#define CANPID_RPM                0x0C
#define CANPID_SPEED              0x0D
#define CANPID_TIMING_ADV         0x0E
#define CANPID_INTAKE_TEMP        0x0F
#define CANPID_MAF                0x10
#define CANPID_THROTTLE           0x11

#define CANPID_FUEL_TYPE          0x51
#define CANPID_HYBRID_BATT_REMAIN 0x5B

#define CANPID_ODOMETER           0x21 //odometer reading working on trainer with 0x21 pid 

#define CANPID_HYBRID_BATT_VOLT   0x9A
#define CANPID_FUEL_PER           0x2F
#define CANPID_VEH_BAT            0x42



//////////////-Service 02-////////////////
#define CANPID_VIN          0x02


/////////////////////////////////////////////////
#define CAN_REQST_ID        0x7DF
#define CAN_REPLY_ID        0x7E8

#define CAN_REQST_ID_EXT    0x18DB33F1
#define CAN_REPLY_ID_EXT    0x18DAF100
#define CAN_REPLY_ID_EXT_FINAL    0x18DAF1FF

#define LISTEN_ID           0x18DAF101
#define REPLY_ID            0x18DA01F1
#define FUNCTIONAL_ID       0x18DB33F1
///////////////////
int canSniff = 0;

boolean pidPrint;
boolean pidGetFlag=0;
boolean pidGetFlag1=0;

boolean pidEnableFlag = 0;

byte vinFlag = 0;
byte odoMeterFlag=0;

bool dtcFlag = 1;
byte dtc_arr[20];
byte dtc_arr1[20];
byte dtc_arr2[20];
byte dat_arr[20];
bool dtcClear = 0;
bool dtcClear_Flag = 0;

int autoFlag =1;//////start with auto searching
int EXTENDED_FLAG = 1;
uint16_t fuelStatus;
float engineload;
uint8_t coolantTemp;
float sf1;
float lf1;
float sf2;
float lf2;
uint16_t gasPressure,kline_gasPressure;
uint8_t manifold_press,kline_manifold_press;
uint16_t rpm,kline_rpm;
uint8_t vehicle_speed,kline_vehicle_speed;
float timing_adv,kline_timing_adv;
float intakeTemp,kline_intakeTemp;
uint16_t maf,kline_maf;
float throttle,kline_throttle;
float fuelT,kline_fuelT;
float hybridBattRem,kline_hybridBattRem;
double odometer;
float hybrid_Bvolt;
float fuel_percentage;
float veh_bat;

char vin_arr[18];
int vin;

uint16_t aq;uint16_t bq;uint16_t cq;uint16_t dq;uint16_t eq;uint16_t fq;uint16_t gq;
uint8_t totalByte;
uint8_t totalByte1;
uint8_t totalByte2;
int extended_offset;
//---
uint16_t kline_trouble_code;
uint16_t kline_pending_trouble_code;
///////////////////
uint32_t canIDReqEx=CAN_REQST_ID_EXT;
uint32_t canIDReq=CAN_REQST_ID;


int reg1,reg2,reg3,reg4;
char   rxExtCan;
uint32_t reg0;

////////////
/*
  void printFrame(CAN_FRAME *message){
  Serial.print(message->id, HEX);
  if (message->extended) Serial.print(" X ");
  else Serial.print(" S ");
  Serial.print(message->length, DEC);
  for (int i = 0; i < message->length; i++) {
    Serial.print(message->data.byte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  }
  */
///

void customCommandDecode(String com)
  {
    Serial.print("recive Command Request   :");Serial.println(com);
    //////////////////
    com.toLowerCase();
    String rxIdCan=rxCom.substring(0,rxCom.lastIndexOf(":")-1);
    rxExtCan=rxCom.charAt(rxCom.indexOf(":")-1);
    String rxPayCan=rxCom.substring(rxCom.indexOf(":")+1);
    //Serial.print("rx Can ID  :");Serial.println(rxIdCan);Serial.print("rx Extended:");Serial.println(rxExtCan);Serial.print("rx Payload :");Serial.println(rxPayCan);
    ///////////////-id string to convertion-//////////
    int idLen=rxIdCan.length()+1;char idBuf[idLen];rxIdCan.toCharArray(idBuf,idLen);  
    sscanf(idBuf,"%x",&reg0);//Serial.print("ID: ");Serial.println(reg0,HEX);
    ////////////////////////////////////
    ////////////////-payload String convertion-/////////////////
    int payLen=rxPayCan.length()+1;char payBuf[payLen];rxPayCan.toCharArray(payBuf,payLen);  
    char d1[]="00";char d2[]="00";char d3[]="00";char d4[]="00";
    sprintf(d1,"%s%s",String(payBuf[0]),String(payBuf[1]));
    sprintf(d2,"%s%s",String(payBuf[2]),String(payBuf[3]));
    sprintf(d3,"%s%s",String(payBuf[4]),String(payBuf[5]));
    sprintf(d4,"%s%s",String(payBuf[6]),String(payBuf[7]));
    //////////////-array to 16 base(hex) byte convertion-///////////////
    reg1=strtol(d1,NULL,16);
    reg2=strtol(d2,NULL,16);
    reg3=strtol(d3,NULL,16);
    reg4=strtol(d4,NULL,16);
    //Serial.print(" d0: ");Serial.print(reg0,HEX);Serial.print(" d1: ");Serial.print(reg1,HEX);Serial.print(" d2: ");Serial.print(reg2,HEX);Serial.print(" d3: ");Serial.print(reg3,HEX);Serial.print(" d4: ");Serial.println(reg4,HEX);
  }
///////////////////////////////////
void requestCar_custom(uint32_t d0,int d1,int d2,int d3,int d4)
  {
    /////////////-array prepare for send to CAN-////////////////////
    CAN_FRAME outgoing;
    if(rxExtCan=='x'){EXTENDED_FLAG=1;}else{EXTENDED_FLAG=0;}///check command extanded or not extanded
      outgoing.id            =d0;/// device ID
      outgoing.length        =8;////data length
      outgoing.extended      =EXTENDED_FLAG;///extended flag
      outgoing.rtr           = 0;
      outgoing.data.uint8[0] = d1;////////
      outgoing.data.uint8[1] = d2;
      outgoing.data.uint8[2] = d3;////data
      outgoing.data.uint8[3] = d4;
      outgoing.data.uint8[4] = 0xaa;
      outgoing.data.uint8[5] = 0xaa;
      outgoing.data.uint8[6] = 0xaa;
      outgoing.data.uint8[7] = 0xaa;
      CAN0.sendFrame(outgoing);////send to CAN
    //  Serial.println("Custom Frame Send");
  }
/////////////////////////////////////////////////
void customCommandEncode()
  {  
      //vTaskDelay(1);  
    //Serial.print(txIdCan);Serial.print(" ");Serial.print(txExtCan);Serial.print(" ");
    //for(int x = 0; x < 15; x++){Serial.print(txPayCan[x],HEX);} Serial.println("");
    /////receiving responce from CALLBACK  
    sprintf(rxResult,"%s%c%c%02x%02x%02x%02x%02x%02x%02x%02x",txIdCan,txExtCan,'|'
    ,txPayCan[0],txPayCan[1],txPayCan[2],txPayCan[3],txPayCan[4],txPayCan[5],txPayCan[6],txPayCan[7]);
    Serial.print("Custom Command Responce :");Serial.println(rxResult);
    //memset(txPayCan,0,sizeof(txPayCan)); 

    //delay(2000);
  }
void printFrame(CAN_FRAME *message)
  {
    txIdCan=String(message->id, HEX);
    //Serial.print("Form Simulator: "); Serial.print(message->id,HEX);
    if (message->extended) txExtCan='x';
    else txExtCan='s';
    //Serial.print(message->length, DEC);
    //   Serial.print(" ");
    memset(txPayCan,0,sizeof(txPayCan));
    for(int i = 0; i < message->length; i++)
    {
      txPayCan[i]=message->data.byte[i];
    //  Serial.print(txPayCan[i],HEX);
    }
    //  Serial.println();
  }
void printFrame1(CAN_FRAME *message)
  {
    uint32_t canID=message->id;
    uint32_t canID1;

    if(canID==0x7E8)
      {
        //////////-for 11 bit-/////////////  
        ///////////CANID=7E8
        //////////CANIDREQ=7E0
        canIDReq=canID-0x08;
        //////////////////////////////////////////////
        //Serial.println("11 Bit-/////////////");
        //Serial.print("ID     :"); Serial.println(canID,HEX);
        //Serial.print("IDREQ  :"); Serial.println(canIDReq,HEX);
      }
    else
      {
        //////////-for 29 bit-/////////////  
        //////////CANID     =18DAF111
        //////////ID1       =1100
        //////////CANIDREQEX=18DA11F1
          canID1=canID&0x000000ff;
          canID1=canID1<<8;
          canIDReqEx=0x18DA00F1|canID1;
        //////////////////////////////////////////////
        //Serial.println("29 Bit-//////////////");
        //Serial.print("ID     :"); Serial.println(canID,HEX);
        //Serial.print("ID1    :"); Serial.println(canID1,HEX);
        //Serial.print("IDREQEX:"); Serial.println(canIDReqEx,HEX);
      }
    //////////////////////////////////////////////

    ////////////////if (message->extended) txExtCan='x';
    ////////////////else txExtCan='s';
    ////////////////Serial.print(message->length, DEC);
    ////////////////Serial.print(" ");
    ////////////////memset(txPayCan,0,sizeof(txPayCan));
    //  for(int i = 0; i < message->length; i++){
    //      Serial.print(message->data.byte[i],HEX);Serial.print(" ");//   txPayCan[i]=message->data.byte[i];//    Serial.print(txPayCan[i],HEX);
    //  }
    //  Serial.println();
  }
   
   uint8_t can_rec_data_counter = 0;
void callback(CAN_FRAME *from_car)
  {
    int usamacall=1+usamacall;

    #if(TESTING)
    {
      Serial.println("storing all can rec data");
      if (can_rec_data_counter < 255)
      {
          resultarray[can_rec_data_counter][0] = from_car->id;
          resultarray[can_rec_data_counter][1] = from_car->data.uint8[0];
          resultarray[can_rec_data_counter][2] = from_car->data.uint8[1];
          resultarray[can_rec_data_counter][3] = from_car->data.uint8[2];
          resultarray[can_rec_data_counter][4] = from_car->data.uint8[3];
          resultarray[can_rec_data_counter][5] = from_car->data.uint8[4];
          resultarray[can_rec_data_counter][6] = from_car->data.uint8[5];
          resultarray[can_rec_data_counter][7] = from_car->data.uint8[6];
          resultarray[can_rec_data_counter][8] = from_car->data.uint8[7];
      }
      can_rec_data_counter++;
    }
	#endif
  if(from_car->data.uint8[2] == CANPID_COOLANT_TEMP){uint8_t coolantByte = from_car->data.uint8[3];coolantTemp = (coolantByte - 40);}
  if(from_car->data.uint8[2] == CANPID_RPM)         {uint8_t rpmOBDH = from_car->data.uint8[3];uint8_t rpmOBDL = from_car->data.uint8[4];rpm = (uint16_t) ((256 * rpmOBDH) + rpmOBDL) / (float)4;}
if (EXTENDED_FLAG)
{
    // if ((from_car->id > CAN_REPLY_ID_EXT-1 && from_car->id < CAN_REPLY_ID_EXT_FINAL+1) && from_car->data.uint8[2] == 0xA6)
    // {
    //     Serial.print("odometer is here in id: ");Serial.println(from_car->id);
    //     odometer_raw[0] = from_car->data.uint8[4];
    //     odometer_raw[1] = from_car->data.uint8[5];
    //     odometer_raw[2] = from_car->data.uint8[6];
    //     odometer_raw[3] = from_car->data.uint8[7];
    //     odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
    //     odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
    // }
    if ((from_car->id == 0x18DAF15D) && (from_car->data.uint8[0] == 0x21) && (from_car->data.uint8[1] == 0x31))
    {
        Serial.print("odometer is here in id: ");Serial.println(from_car->id);
          int byte_counter = 0;
          for (int iter = 3; iter>=0; iter--)
          {
            if(from_car->data.uint8[iter+2] == 0xAA) 
            {
              odometer_raw[3-iter] = 0;
              byte_counter++;
            }
            else
            {
              odometer_raw[3-iter] = from_car->data.uint8[2+(3-iter-byte_counter)];
            }
          }
          odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
        odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
    }
}
else 
{
  if (from_car->id == CAN_REPLY_ID && from_car->data.uint8[2] == 0xA6)
  {
      Serial.print("odometer is here in id: ");Serial.println(from_car->id);
      odometer_raw[0] = from_car->data.uint8[4];
      odometer_raw[1] = from_car->data.uint8[5];
      odometer_raw[2] = from_car->data.uint8[6];
      odometer_raw[3] = from_car->data.uint8[7];
      odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
      odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
  }
}

//   if (from_car->id == 0x158)
// {
//     Serial.print("odometer is here in id: ");Serial.println(from_car->id);
//     odometer_raw[0] = from_car->data.uint8[4];
//     odometer_raw[1] = from_car->data.uint8[5];
//     odometer_raw[2] = from_car->data.uint8[6];
//     odometer_raw[3] = from_car->data.uint8[7];
//     odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
//     odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
// }
//   if (from_car->id == 0x516)
// {
//     Serial.print("odometer is here in id: ");Serial.println(from_car->id);
//     odometer_raw[0] = from_car->data.uint8[4];
//     odometer_raw[1] = from_car->data.uint8[5];
//     odometer_raw[2] = from_car->data.uint8[6];
//     odometer_raw[3] = from_car->data.uint8[7];
//     odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
//     odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
// }
  if (from_car->id == 0x611 && !EXTENDED_FLAG)
{
    Serial.print("odometer is here in id: ");Serial.println(from_car->id);
    odometer_raw[0] = from_car->data.uint8[4];
    odometer_raw[1] = from_car->data.uint8[5];
    odometer_raw[2] = from_car->data.uint8[6];
    odometer_raw[3] = from_car->data.uint8[7];
    odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
    odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
}
//   if (from_car->id == 0x330)
// {
//     Serial.print("odometer is here in id: ");Serial.println(from_car->id);
//     odometer_raw[0] = from_car->data.uint8[4];
//     odometer_raw[1] = from_car->data.uint8[5];
//     odometer_raw[2] = from_car->data.uint8[6];
//     odometer_raw[3] = from_car->data.uint8[7];
//     odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
//     odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
// }
//   if (from_car->id == 0x19D)
// {
//     Serial.print("odometer is here in id: ");Serial.println(from_car->id);
//     odometer_raw[0] = from_car->data.uint8[4];
//     odometer_raw[1] = from_car->data.uint8[5];
//     odometer_raw[2] = from_car->data.uint8[6];
//     odometer_raw[3] = from_car->data.uint8[7];
//     odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
//     odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
// }

    

    

    #if (NORMAL_OP)

    canCallBackFlag=HIGH;  
    if(rxFlag==HIGH){//Serial.print("S: ");Serial.print(reg3,HEX);//Serial.print("   D: ");Serial.println(from_car->data.uint8[2],HEX);
    if(from_car->data.uint8[2] ==reg3){rxFlag=LOW;printFrame(from_car);}}
    /////////////////////////////////////////////////////////////////////////////
    printFrame1(from_car);
    ///////////////////////////////////
    //Serial.print(">>> ");Serial.println(from_car->data.uint8[2],HEX);

    if (from_car->id == 0x611)
    {
      Serial.println("odometer is here");
      odometer_raw[0] = from_car->data.uint8[4];
      odometer_raw[1] = from_car->data.uint8[5];
      odometer_raw[2] = from_car->data.uint8[6];
      odometer_raw[3] = from_car->data.uint8[7];
      odometer_val = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;
      odometer = (((odometer_raw[0] * 16777216) + (odometer_raw[1] * 65536) + (odometer_raw[2] * 256) + odometer_raw[3]) ) ;

    }
 
    /////////////
    //if(vinFlag==0){
    if(from_car->data.uint8[1] == 0x7F){Serial.println("unknown request");resultarray[from_car->data.uint8[2]][1] = from_car->data.uint8[1];}
    if(from_car->data.uint8[2] == CANPID_RPM)         {uint8_t rpmOBDH = from_car->data.uint8[3];uint8_t rpmOBDL = from_car->data.uint8[4];rpm = (uint16_t) ((256 * rpmOBDH) + rpmOBDL) / (float)4;}
    if(from_car->data.uint8[2] == CANPID_FUEL)        {uint8_t fuelOBDH = from_car->data.uint8[3];uint8_t fuelOBDL = from_car->data.uint8[4];fuelStatus = ((fuelOBDH << 8) + fuelOBDL);}
    if(from_car->data.uint8[2] == CANPID_ENGINELOAD)  {uint8_t engineload_byte = from_car->data.uint8[3];engineload = ((float)engineload_byte / 2.55);}
    if(from_car->data.uint8[2] == CANPID_COOLANT_TEMP){uint8_t coolantByte = from_car->data.uint8[3];coolantTemp = (coolantByte - 40);}
    ///////////////////////
    if(from_car->data.uint8[2] ==6){uint8_t b1 = from_car->data.uint8[3];sf1=(((float)b1 / 1.28) - 100);}
    //////////////////////
    if(from_car->data.uint8[2] ==7){uint8_t b2 = from_car->data.uint8[3];lf1 = (((float)b2 / 1.28) - 100);}
    /////////////////////
    if(from_car->data.uint8[2]==8){uint8_t b3 = from_car->data.uint8[3];sf2 = (((float)b3 / 1.28) - 100);}
    //////////////////////
    if(from_car->data.uint8[2]==9){uint8_t b4 = from_car->data.uint8[3];lf2 = (((float)b4 / 1.28) - 100);}
    //////////////////////
    if(from_car->data.uint8[2]==CANPID_GAUGE_PRESSURE) {
    uint8_t a1 = from_car->data.uint8[3];
    gasPressure = (3 * a1);
    }
    //////////////////
    if(from_car->data.uint8[2] == CANPID_MANIFOLD_PRESSURE) {
    uint8_t a2 = from_car->data.uint8[3];
    manifold_press = (a2);
    }
    /////////////////////
    if(from_car->data.uint8[2] == CANPID_SPEED){
    uint8_t a3 = from_car->data.uint8[3];
    vehicle_speed = (a3);
    }
    /////////////////////
    if(from_car->data.uint8[2] == CANPID_TIMING_ADV) {
    uint8_t a4 = from_car->data.uint8[3];
    timing_adv = (((float)a4 / 2) - 64);
    }
    /////////////////////
    if(from_car->data.uint8[2] == CANPID_INTAKE_TEMP) {
    uint8_t intakeByte = from_car->data.uint8[3];
    intakeTemp = (intakeByte - 40);
    }
    /////////////////////
    if(from_car->data.uint8[2] == CANPID_MAF){
    uint8_t aa = from_car->data.uint8[3];
    uint8_t bb = from_car->data.uint8[4];
    maf = (uint16_t) ((256 * aa) + bb) / (float)4;
    }
    //////////////////////
    if(from_car->data.uint8[2] == 0x11) {
    uint8_t a = from_car->data.uint8[3];
    throttle = ((float)a / 2.55);
    }
    //////////////////////
    if(from_car->data.uint8[2] == CANPID_FUEL_TYPE){
    uint8_t s = from_car->data.uint8[3];
    fuelT = s;
    }
    //////////////////////
    if(from_car->data.uint8[2] == CANPID_HYBRID_BATT_REMAIN) {
    uint8_t a = from_car->data.uint8[3];
    hybridBattRem = ((float)a / 2.55);
    }
    //////////////////////
    //////////////////////////////
    if(from_car->data.uint8[2] == CANPID_HYBRID_BATT_VOLT) {
    uint8_t af = from_car->data.uint8[3];
    hybrid_Bvolt = ((float)af / 2.55);
    }
    if(from_car->data.uint8[2] == CANPID_FUEL_PER){
    uint8_t fuel = from_car->data.uint8[3];
    fuel_percentage = ((float)fuel / 2.55);
    }
    if(from_car->data.uint8[2] ==CANPID_VEH_BAT){
    uint8_t batA = from_car->data.uint8[3];
    uint8_t batB = from_car->data.uint8[4];
    //Serial.print("A: ");Serial.println(batA);
    //Serial.print("B: ");Serial.println(batB);
    veh_bat=(float)((256 *batA)+batB)/1000;
    }

    //}
    //////////////////////
    if(from_car->data.uint8[2] == CANPID_ODOMETER /*0x62 && odoMeterFlag==1*/) {
    Serial.print(">> ");Serial.println(odoMeterFlag);

    uint8_t u1 = from_car->data.uint8[3];
    uint8_t u2 = from_car->data.uint8[4];
    uint8_t u3 = from_car->data.uint8[5];
    uint8_t u4 = from_car->data.uint8[6];
    odometer = (((u1 * 16777216) + (u2 * 65536) + (u3 * 256) + u4) / 10) ;
    Serial.print(",,,,,,");
    Serial.print(from_car->data.uint8[0]);
    Serial.print(",,,,,,");    
    Serial.print(from_car->data.uint8[1]);
    Serial.print(",,,,,,");
    Serial.print(u1);
    Serial.print(",,,,,,");
    Serial.print(u2);
    Serial.print(",,,,,,");
    Serial.print(u3);
    Serial.print(",,,,,,");
    Serial.print(u4);
    Serial.print(",,,,,,");
    //odometer = (u1+u2+u3+u4+1);   
    Serial.println(">>usama>>>>>>111");
    odoMeterFlag++;
    }
    // if(from_car->data.uint8[2] == CANPID_ODOMETER /*0x21 && odoMeterFlag==2*/){
    // Serial.print(">> ");Serial.println(odoMeterFlag);

    // uint8_t u1 = from_car->data.uint8[3];
    // uint8_t u2 = from_car->data.uint8[4];
    // uint8_t u3 = from_car->data.uint8[5];
    // uint8_t u4 = from_car->data.uint8[6];
    // //odometer = (((a * 16777216) + (b * 65536) + (c * 256) + d) / 10) ;
    // odometer = (u1+u2+u3+u4+2);
    // Serial.println(">>usama>>>>>>222");
    // odoMeterFlag++;
    //}

    // if(from_car->data.uint8[2] == CANPID_ODOMETER) {
    // uint8_t usma1 = from_car->data.uint8[2];
    // Serial.println(CANPID_ODOMETER);
    // Serial.print(":::::::");
    // Serial.println(from_car->data.uint8[2]);
    // uint8_t u1 = from_car->data.uint8[3];
    // uint8_t u2 = from_car->data.uint8[4];
    // uint8_t u3 = from_car->data.uint8[5];
    // uint8_t u4 = from_car->data.uint8[6];
    // //odometer = (((a * 16777216) + (b * 65536) + (c * 256) + d) / 10) ;
    // Serial.print(",,,,,,");
    // Serial.print(from_car->data.uint8[0]);
    // Serial.print(",,,,,,");    
    // Serial.print(from_car->data.uint8[1]);
    // Serial.print(",,,,,,");
    // Serial.print(u1);
    // Serial.print(",,,,,,");
    // Serial.print(u2);
    // Serial.print(",,,,,,");
    // Serial.print(u3);
    // Serial.print(",,,,,,");
    // Serial.print(u4);
    // Serial.print(",,,,,,");
    
    // odometer = (u1+u2+u3+u4+1);   
    // Serial.println(">>usama>>>>>>111");

    // }

    /////////////////////
    if(from_car->data.uint8[2] == 0x49 && vinFlag == 1){
    vin_arr[0]  = from_car->data.uint8[5];
    vin_arr[1]  = from_car->data.uint8[6];
    vin_arr[2]  = from_car->data.uint8[7];
    vinFlag++;
    //Serial.print(">> ");Serial.println(vinFlag);
    }
    if(from_car->data.uint8[0] == 0x21 && vinFlag == 2){
    vin_arr[3]  = from_car->data.uint8[1];
    vin_arr[4]  = from_car->data.uint8[2];
    vin_arr[5]  = from_car->data.uint8[3];
    vin_arr[6]  = from_car->data.uint8[4];
    vin_arr[7]  = from_car->data.uint8[5];
    vin_arr[8]  = from_car->data.uint8[6];
    vin_arr[9]  = from_car->data.uint8[7];
    vinFlag++;
    //Serial.print(">> ");Serial.println(vinFlag);
    }
    if(from_car->data.uint8[0] == 0x22 && vinFlag == 3){
    vin_arr[10] = from_car->data.uint8[1];
    vin_arr[11] = from_car->data.uint8[2];
    vin_arr[12] = from_car->data.uint8[3];
    vin_arr[13] = from_car->data.uint8[4];
    vin_arr[14] = from_car->data.uint8[5];
    vin_arr[15] = from_car->data.uint8[6];
    vin_arr[16] = from_car->data.uint8[7];
    vin_arr[17] = '\0';///null terminator , for removing extra garbuge
    //Serial.print(">>>> ");Serial.println(String(vin_arr));
    vinFlag=0;
    }
    if(vinFlag==0){
    if(dtcFlag == 1){
    if(from_car->data.uint8[1] == 67){
    totalByte = (char)from_car->data.uint8[2];
    if(totalByte > 2){totalByte = 2;}
    for(int i = 0; i < totalByte * 2; i++){
    dtc_arr[i] = (char)from_car->data.uint8[i + 3];}
    }
    if(from_car->data.uint8[1] == 71){
    totalByte1 = (char)from_car->data.uint8[2];
    if(totalByte1 > 2){totalByte1 = 2;}
    for (int i = 0; i < totalByte1 * 2; i++){dtc_arr1[i] = (char)from_car->data.uint8[i + 3];}
    }
    if(from_car->data.uint8[1] == 74){
    totalByte2 = (char)from_car->data.uint8[2];
    if(totalByte2 > 2){totalByte2 = 2;}
    for(int i = 0; i < totalByte2 * 2; i++){
    dtc_arr2[i] = (char)from_car->data.uint8[i + 3];}
    }}
    }
    #endif
  }
//////////////////////////////////////////////////
//////////////////////////////////////////////////
void requestCarFirst          (int dataa)          {CAN_FRAME outgoing;
if(EXTENDED_FLAG == 0){outgoing.id=CAN_REQST_ID;}else{outgoing.id = CAN_REQST_ID_EXT;}
//if(EXTENDED_FLAG == 0){outgoing.id=canIDReq;}else{outgoing.id =canIDReqEx;}

//Serial.print("ID: ");Serial.println(outgoing.id,HEX);
///////////////////////////////////////////////
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
///////////////////////////////////////////////
  outgoing.data.uint8[0] = 0x02;
  outgoing.data.uint8[1] = 0x01;
  outgoing.data.uint8[2] = dataa;
  outgoing.data.uint8[3] = 0xaa;
  outgoing.data.uint8[4] = 0xaa;
  outgoing.data.uint8[5] = 0xaa;
  outgoing.data.uint8[6] = 0xaa;
  outgoing.data.uint8[7] = 0xaa;
  CAN0.sendFrame(outgoing);
}
void requestCar          (int dataa)          {CAN_FRAME outgoing;
if(EXTENDED_FLAG==0){outgoing.id = canIDReq;}else{outgoing.id = canIDReqEx;}
//Serial.print("ID");Serial.print(CAN_REQST_ID_EXT);
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;
  outgoing.data.uint8[1] = 0x01;
  outgoing.data.uint8[2] = dataa;
  outgoing.data.uint8[3] = 0xaa;
  outgoing.data.uint8[4] = 0xaa;
  outgoing.data.uint8[5] = 0xaa;
  outgoing.data.uint8[6] = 0xaa;
  outgoing.data.uint8[7] = 0xaa;
  CAN0.sendFrame(outgoing);

///while(!canCallBackFlag);////wait for callback funcation responce
//canCallBackFlag=LOW;
}
void requestCar_DTC_clear(void)               {CAN_FRAME outgoing;
if(EXTENDED_FLAG == 0){outgoing.id = CAN_REQST_ID;}else{outgoing.id = CAN_REQST_ID_EXT;}
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x01;
  outgoing.data.uint8[1] = 0x04;
  outgoing.data.uint8[2] = 0x00;
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;
  CAN0.sendFrame(outgoing);
}
void requestCar_DTC      (int dataa, int type){CAN_FRAME outgoing;
if(EXTENDED_FLAG == 0){outgoing.id=canIDReqEx;}else{outgoing.id=canIDReq;}
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x01;
  outgoing.data.uint8[1] = type;
  outgoing.data.uint8[2] = 0x00;
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;
  CAN0.sendFrame(outgoing);
}
void requestCar_VIN      (int dataa)          {CAN_FRAME outgoing;
if(EXTENDED_FLAG == 0){outgoing.id =canIDReq;}else{outgoing.id =canIDReqEx;}
//Serial.print("Vin ID  : ");Serial.println(outgoing.id,HEX);
///////////////////////////////////
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
/////////////////////////////////////  
if(vinFlag==1){
  outgoing.data.uint8[0] = 0x02;///number of frame
  outgoing.data.uint8[1] = 0x09;///mode//service
  outgoing.data.uint8[2] = 0x02;///pid
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;
//Serial.println(">>0");
}else if(vinFlag==2){
  outgoing.data.uint8[0] = 0x30;
  outgoing.data.uint8[1] = 0x00;
  outgoing.data.uint8[2] = 0x00;
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;

//Serial.println(">>2");
}//else{/*Serial.println("Vin End");*/}
  CAN0.sendFrame(outgoing);
}

void requestCar_odo         (int dataa)          {CAN_FRAME outgoing;
if(EXTENDED_FLAG==0){outgoing.id = canIDReq;}else{outgoing.id = canIDReqEx;}
//Serial.print("ID");Serial.print(CAN_REQST_ID_EXT);
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;
  outgoing.data.uint8[1] = 0x01;
  outgoing.data.uint8[2] = dataa;
  outgoing.data.uint8[3] = 0xaa;
  outgoing.data.uint8[4] = 0xaa;
  outgoing.data.uint8[5] = 0xaa;
  outgoing.data.uint8[6] = 0xaa;
  outgoing.data.uint8[7] = 0xaa;
  CAN0.sendFrame(outgoing);
  Serial.println("odo-working....................");
}

void requestCar_odometer (int dataaFlag)      {CAN_FRAME outgoing;
if(EXTENDED_FLAG==0){outgoing.id=canIDReq;}else{outgoing.id=canIDReqEx;}
////////////////////////////////////////
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
////////////////////////////////////////
Serial.print("dataaFlag    ");Serial.println(dataaFlag);
if(dataaFlag==1){  
  outgoing.data.uint8[0] = 0x02;////number of frame
  outgoing.data.uint8[1] = 0x01;///mode
  outgoing.data.uint8[2] = CANPID_ODOMETER;///0xA6////0x22
  outgoing.data.uint8[3] = 0x00;///0xA6////0x22
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;
Serial.println("<<<<1");
}else if(dataaFlag==2){
  outgoing.data.uint8[0] = 0x30;
  outgoing.data.uint8[1] = 0x00;
  outgoing.data.uint8[2] = 0x00;
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;
Serial.println("<<<<2");
}
//else{/*Serial.println("Vin End");*/}
CAN0.sendFrame(outgoing);
Serial.print("odo_done");
Serial.print(dataaFlag);

}
void requestCar_four_byte(int dataa){
  CAN_FRAME outgoing;
  if (EXTENDED_FLAG == 0)
  {
    outgoing.id = CAN_REQST_ID;
  }
  else
  {
    outgoing.id = CAN_REQST_ID_EXT;
  }
  outgoing.length = 8;
  outgoing.extended = EXTENDED_FLAG;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;
  outgoing.data.uint8[1] = 0x01;
  outgoing.data.uint8[2] = dataa;
  outgoing.data.uint8[3] = 0xaa;
  outgoing.data.uint8[4] = 0xaa;
  outgoing.data.uint8[5] = 0xaa;
  outgoing.data.uint8[6] = 0xaa;
  outgoing.data.uint8[7] = 0xaa;
  CAN0.sendFrame(outgoing);
}
///////////////
