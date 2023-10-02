void gpsBTprint();
void mpuBTprint();
void pidBTPrint();
/////////
void pidSerialPrint();
void mpuSerialprint();
void tempSerialprint();
void tempBTprint();


/////////////////////////////////
///////////////////////////////////////
//usama
// void mpuSerialprint(){
// Serial.println("-----------");
// Serial.print("X: ");  Serial.print(angleX);
// Serial.print(", Y: ");Serial.print(angleY);
// Serial.print(", Z: ");Serial.print(angleZ);
// Serial.print(", M: ");Serial.print(mag);
// Serial.print(", T: ");Serial.println(tempT);
// Serial.println("-----------");
// }
//usama
/*
void mpuBTprint(){
SerialBT.println("-----------");
SerialBT.print("X: ");  SerialBT.print(angleX);
SerialBT.print(", Y: ");SerialBT.print(angleY);
SerialBT.print(", Z: ");SerialBT.print(angleZ);
SerialBT.print(", M: ");SerialBT.print(mag);
SerialBT.print(", T: ");SerialBT.println(tempT);
SerialBT.println("-----------");
}*/
//usama
// void tempSerialprint(){
// Serial.print("Temperature.    : ");Serial.println(boardTemp);
// Serial.print("Humdity.        : ");Serial.println(boardHum);  
// } usama
/*
void tempBTprint(){
SerialBT.print("Temperature.    : ");SerialBT.println(boardTemp);
SerialBT.print("Humdity.        : ");SerialBT.println(boardHum);  
}
void gpsBTprint(){
SerialBT.print("GNSS Mod: ");SerialBT.println(getGnssM);
SerialBT.print("Lat.    : ");SerialBT.println(GPSlatitude);
SerialBT.print("Lng.    : ");SerialBT.println(GPSlongitude);
SerialBT.print("Speed   : ");SerialBT.println(GPSspeed);
SerialBT.print("Altitude: ");SerialBT.println(GPSaltitude);
SerialBT.print("V Sattel: ");SerialBT.println(vsat);
SerialBT.print("U Sattel: ");SerialBT.println(usat);
SerialBT.print("Accuracy: ");SerialBT.println(accuracy);
SerialBT.print("Date    : ");SerialBT.print(GPSdate);SerialBT.print("/");
                             SerialBT.print(GPSmonth);SerialBT.print("/");
                             SerialBT.println(GPSyear);
SerialBT.print("Time    : ");SerialBT.print(GPShours);SerialBT.print("-");
                             SerialBT.print(GPSminutes);SerialBT.print("-");
                             SerialBT.println(GPSseconds);
SerialBT.println("----------------------------");
}*/
/*
void pidBTPrint(){
SerialBT.print(" Fuel Status    : "); SerialBT.println(fuelStatus);
SerialBT.print(" Engine Load    : "); SerialBT.println(engineload);
SerialBT.print(" Coolant Temp   : "); SerialBT.println(coolantTemp);
SerialBT.print(" Sf1            : ");SerialBT.println(sf1);
SerialBT.print(" Lf1            : ");SerialBT.println(lf1);
SerialBT.print(" Sf2            : ");SerialBT.println(sf2);
SerialBT.print(" Lf2            : ");SerialBT.println(lf2);
SerialBT.print(" Gauge Pressure : ");SerialBT.println(gasPressure);
SerialBT.print(" ManiFold Press.: ");SerialBT.println(manifold_press);
SerialBT.print(" Engine RPM     : "); SerialBT.println(rpm);
SerialBT.print(" Vehicle Speed  : "); SerialBT.println(vehicle_speed);
SerialBT.print(" Timing Advance : ");SerialBT.println(timing_adv);
SerialBT.print(" intake air Temp: ");SerialBT.println(intakeTemp);
SerialBT.print(" Mass Air Flow  : ");SerialBT.println(maf);
SerialBT.print(" Throttle Pos.  : ");SerialBT.println(throttle);
SerialBT.print(" Fuel Type      : ");SerialBT.println(fuelT);
SerialBT.print(" HBat Remaning  : ");SerialBT.println(hybridBattRem);
///////////////////////////////////////////////////
SerialBT.print(" Odometer       : ");SerialBT.println(odometer);
SerialBT.print(" HBat Voltage   : ");SerialBT.println(hybrid_Bvolt);
SerialBT.print(" Fuel Percentage: ");SerialBT.println(fuel_percentage);
SerialBT.print(" VIN:");
for(int i=0;i<17;i++){SerialBT.print(vin_arr[i]);}SerialBT.println("");
SerialBT.print(" DTC :");
for(int i=0;i<17;i++){SerialBT.print(dtc_arr[i], HEX);}SerialBT.println("");
SerialBT.println(F("-----------------------------------"));
}
*/
////

void pidSerialPrint(){

  Serial.println("in canprint");
Serial.println("/////////////////////////////////////////");
#if (0)
Serial.print(" Fual Status    : "); Serial.print(fuelStatus);Serial.print("    ||  Engine Load    : "); Serial.println(engineload);
Serial.print(" Coolent Temp   : "); Serial.print(coolantTemp);Serial.print("      ||  Sf1            : ");Serial.println(sf1);
Serial.print(" Lf1            : ");Serial.print(lf1);Serial.print("  ||  Sf2            : ");Serial.println(sf2);
Serial.print(" Lf2            : ");Serial.print(lf2);Serial.print("   ||  Gauge Pressure : ");Serial.println(gasPressure);
Serial.print(" ManiFold Press : ");Serial.print(manifold_press);Serial.print("       ||  Engine Speed   : "); Serial.println(rpm);
Serial.print(" Vehicle Speed  : "); Serial.print(vehicle_speed);Serial.print("      ||  Timing Advance : ");Serial.println(timing_adv);
Serial.print(" intake air Temp: ");Serial.print(intakeTemp);Serial.print("   ||  Mass Air Flow  : ");Serial.println(maf);
Serial.print(" Throttle Pos   : ");Serial.print(throttle);Serial.print("    ||  Fuel Type      : ");Serial.println(fuelT);
Serial.print(" HBat Remaning  : ");Serial.print(hybridBattRem);Serial.print("    ||  HBat Voltage   : ");Serial.println(hybrid_Bvolt);
Serial.print(" Fuel Percentage: ");Serial.print(fuel_percentage);Serial.print("    ||  Vehical Bat    : ");Serial.println(veh_bat);

Serial.print(" Odomete        : ");Serial.println(odometer);
Serial.print(" VIN :");
for(int i=0;i<17;i++){Serial.print(vin_arr[i]);}Serial.println("");
Serial.print(" DTC :");
for(int i=0;i<17;i++){Serial.print(dtc_arr[i]);Serial.print(" ");}Serial.println("");
Serial.println("//////////////////////////////////////////n/n/n");
#endif
Serial.print("this is odo: ");Serial.println(odometer_val);
Serial.println("//////////////////////////////////////////n/n/n");

// for (int counter=0; counter<can_rec_data_counter;counter++)
// {
//   Serial.print("can ID info: ");
//   Serial.print(resultarray[can_rec_data_counter][0], HEX);
//   Serial.print(resultarray[can_rec_data_counter][1], HEX);
//   Serial.print(resultarray[can_rec_data_counter][2], HEX);
//   Serial.print(resultarray[can_rec_data_counter][3], HEX);
//   Serial.print(resultarray[can_rec_data_counter][4], HEX);
//   Serial.print(resultarray[can_rec_data_counter][5], HEX);
//   Serial.print(resultarray[can_rec_data_counter][6], HEX);
//   Serial.print(resultarray[can_rec_data_counter][7], HEX);
//   Serial.println(resultarray[can_rec_data_counter][8], HEX);
// }


// Serial.print("odo can ID: ");Serial.println(odometer_can_id);
// Serial.print("odo possible can IDs: ");
// Serial.println(possible_odo_ids[0]);
// Serial.println(possible_odo_ids[1]);
// Serial.println(possible_odo_ids[2]);
// Serial.println(possible_odo_ids[3]);
// Serial.println(possible_odo_ids[4]);
// Serial.println(possible_odo_ids[5]);
// Serial.println(possible_odo_ids[6]);
// Serial.println(possible_odo_ids[7]);
// Serial.println(possible_odo_ids[8]);
// Serial.println(possible_odo_ids[9]);


}
