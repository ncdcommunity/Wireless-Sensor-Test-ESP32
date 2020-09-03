///this code is written and tested for ncd.io wireless temperature humidity sensor with arduino due
///sensor data structure can be found here https://ncd.io/long-range-iot-wireless-temperature-humidity-sensor-product-manual/
/// sesnro can be found here https://store.ncd.io/product/industrial-long-range-wireless-temperature-humidity-sensor/

  uint8_t data[100];
 uint8_t tval16[4];
  int length = 100;
  int k = 10;
  int i;
int ncd_sensor_type;
void setup()
{
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // pins 16 rx2, 17 tx2, 19200 bps, 8 bits no parity 1 stop bit​
  Serial.begin(9600);
  Serial.println("ncd.io IoT Wireless Sensors");
}

int32_t sign_extend_24_32(int32_t x)
{
  int32_t ret; 
  x &= 0x00FFFFFF; 
  ret = x; 
  if(x & (1<<23))
  {
    /*Negative number*/
    /*Set MSByte*/
    ret |= 0xFF000000; 
  }
  else 
  {
    /*Positive Number*/
    /*Do nothing*/
  }
  return ret;
}

int sensor_type_5(void)

{
  Serial.println("ncd.io IoT Wireless Accelero Gyro Magneto Temperature Sensor");
     Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      }         
      Serial.println(" ");

  float acc_x = ((uint32_t)(((data[24])<<16) + ((data[25])<<8) + (data[26])));
  acc_x = sign_extend_24_32(acc_x);
  acc_x = acc_x/100;
  
  float acc_y = ((uint32_t)(((data[27])<<16) + ((data[28])<<8) + (data[29])));
  acc_y = sign_extend_24_32(acc_y);
  acc_y = acc_y/100;
  
  float acc_z = ((uint32_t)(((data[30])<<16) + ((data[31])<<8) + (data[32])));
  acc_z = sign_extend_24_32(acc_z);
  acc_z = acc_z/100;

  float mag_x = ((uint32_t)(((data[33])<<16) + ((data[34])<<8) + (data[35])));
  mag_x = sign_extend_24_32(mag_x);
  mag_x = mag_x/100;
  
  float mag_y = ((uint32_t)(((data[36])<<16) + ((data[37])<<8) + (data[38])));
  mag_y = sign_extend_24_32(mag_y);
  mag_y = mag_y/100;
  
  float mag_z = ((uint32_t)(((data[39])<<16) + ((data[40])<<8) + (data[41])));
  mag_z = sign_extend_24_32(mag_z);
  mag_z = mag_z/100;
  

  float dps_x = ((uint32_t)(((data[42])<<16) + ((data[43])<<8) + (data[44])));
  dps_x = sign_extend_24_32(dps_x);
  
  float dps_y = ((uint32_t)(((data[45])<<16) + ((data[46])<<8) + (data[47])));
  dps_y = sign_extend_24_32(dps_y);
  
  float dps_z = ((uint32_t)(((data[48])<<16) + ((data[49])<<8) + (data[50])));
  dps_z = sign_extend_24_32(dps_z);
  
  int cTemp = ((((data[51]) * 256) + data[52]));
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Temperature in Celsius :");
  Serial.print(cTemp);
  Serial.println(" C");
  
  Serial.print("Acceleration in X-axis :");
  Serial.print(acc_x,3);
  Serial.println(" mg");
  Serial.print("Acceleration in Y-axis :");
  Serial.print(acc_y,3);
  Serial.println(" mg");
  Serial.print("Acceleration in Z-axis :");
  Serial.print(acc_z,3);
  Serial.println(" mg");

  Serial.print("Magnetic Field in X-axis :");
  Serial.print(mag_x,3);
  Serial.println(" mgauss");
  Serial.print("Magnetic Field in Y-axis :");
  Serial.print(mag_y,3);
  Serial.println(" mgauss");
  Serial.print("Magnetic Field in Z-axis :");
  Serial.print(mag_z,3);
  Serial.println(" mgauss"); 

  Serial.print("Gyro Rate in X-axis :");
  Serial.print(dps_x,3);
  Serial.println(" mdps");
  Serial.print("Gyro Rate  in Y-axis :");
  Serial.print(dps_y,3);
  Serial.println(" mdps");
  Serial.print("Gyro Rate  in Z-axis :");
  Serial.print(dps_z,3);
  Serial.println(" mdps"); 

  
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_8(void)

{
  Serial.println("ncd.io IoT Wireless Vibration Accelero Gyro Magneto Temperature Sensor");
     Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      }         
 Serial.println(" ");
 float rms_x = ((int32_t)(((data[24])<<16) + ((data[25])<<8) + (data[26])));
  rms_x = sign_extend_24_32(rms_x);
  rms_x = rms_x/100;
  float rms_y = ((int32_t)(((data[27])<<16) + ((data[28])<<8) + (data[29])));
  rms_y = sign_extend_24_32(rms_y);
  rms_y = rms_y/100;
  float rms_z = ((int32_t)(((data[30])<<16) + ((data[31])<<8) + (data[32])));
  rms_z = sign_extend_24_32(rms_z);
  rms_z = rms_z/100;

  float max_x = ((int32_t)(((data[33])<<16) + ((data[34])<<8) + (data[35])));
  max_x = sign_extend_24_32(max_x);
  max_x = max_x/100;
  
  float max_y = ((int32_t)(((data[36])<<16) + ((data[37])<<8) + (data[38])));
  max_y = sign_extend_24_32(max_y);
  max_y = max_y/100;
  
  float max_z = ((int32_t)(((data[39])<<16) + ((data[40])<<8) + (data[41])));
  max_z = sign_extend_24_32(max_z);
  max_z = max_z/100;
  

  float min_x = ((int32_t)(((data[42])<<16) + ((data[43])<<8) + (data[44])));
  min_x = sign_extend_24_32(min_x);
  min_x = min_x/100;
  
  float min_y = ((int32_t)(((data[45])<<16) + ((data[46])<<8) + (data[47])));
  min_y = sign_extend_24_32(min_y);
  min_y = min_y/100;
  
  float min_z = ((int32_t)(((data[48])<<16) + ((data[49])<<8) + (data[50])));
  min_z = sign_extend_24_32(min_z);
  min_z = min_z/100;

  
  int cTemp = ((((data[51]) * 256) + data[52]));
 
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(data[22]);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Temperature in Celsius :");
  Serial.print(cTemp);
  Serial.println(" C");
  
  Serial.print("RMS vibration in X-axis :");
  Serial.print(rms_x);
  Serial.println(" mg");
  Serial.print("RMS vibration in Y-axis :");
  Serial.print(rms_y);
  Serial.println(" mg");
  Serial.print("RMS vibration in Z-axis :");
  Serial.print(rms_z);
  Serial.println(" mg");

  Serial.print("Max vibration in X-axis :");
  Serial.print(max_x,2);
  Serial.println(" mg");
  Serial.print("Max vibration in Y-axis :");
  Serial.print(max_y,2);
  Serial.println(" mg");
  Serial.print("Max vibration in Z-axis :");
  Serial.print(max_z,2);
  Serial.println(" mg");
  
  Serial.print("Min vibration in X-axis :");
  Serial.print(min_x);
  Serial.println(" mg");
  Serial.print("Min vibration in Y-axis :");
  Serial.print(min_y);
  Serial.println(" mg");
  Serial.print("Min vibration in Z-axis :");
  Serial.print(min_z);
  Serial.println(" mg"); 
}

int sensor_type_44(void)
{
  Serial.println("ncd.io IoT Wireless CO2,Temperature, and Humidity Sensor");
       Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      } 
       Serial.println(" ");
    float CO2 = ((((data[24])<<24) + (data[25] <<16)+ (data[26] <<8)+ data[27]) /100.0);        
  float humidity = ((((data[28]) * 256) + data[29]) /100.0);
  //int16_t cTempint = (((uint16_t)(data[26])<<8)| data[27]);
  float cTemp = (((data[30])<<8)| data[31]) /100.0;
  float fTemp = cTemp * 1.8 + 32;
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Relative Humidity :");
  Serial.print(humidity);
  Serial.println(" %RH");
  Serial.print("Temperature in Celsius :");
  Serial.print(cTemp);
  Serial.println(" 'C");
  Serial.print("Temperature in Fahrenheit :");
  Serial.print(fTemp);
  Serial.println(" 'F");
  Serial.print("Co2 in ppm :");
  Serial.print(CO2);
  Serial.println(" ppm");
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_49(void)
{
  Serial.println("ncd.io IoT Wireless 6 Channel Temperature Sensor");
       Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      } 
       Serial.println(" ");
    float temp1 = ((((data[24])<<24) + (data[25] <<16)+ (data[26] <<8)+ data[27]) /100.0);
    float temp2 = ((((data[28])<<24) + (data[29] <<16)+ (data[30] <<8)+ data[31]) /100.0);
    float temp3 = ((((data[32])<<24) + (data[33] <<16)+ (data[34] <<8)+ data[35]) /100.0);
    float temp4 = ((((data[36])<<24) + (data[37] <<16)+ (data[38] <<8)+ data[39]) /100.0);
    float temp5 = ((((data[40])<<24) + (data[41] <<16)+ (data[42] <<8)+ data[43]) /100.0);
    float temp6 = ((((data[44])<<24) + (data[45] <<16)+ (data[46] <<8)+ data[47]) /100.0);        
//      00 00 0A D0, 00 00 0A DD, 00 00 0A 08, 99 00 0A 85, 00 00 0A A5, 00 00 0A C4
  float fTemp1 = temp1 * 1.8 + 32;
  float fTemp2 = temp2 * 1.8 + 32;
  float fTemp3 = temp3 * 1.8 + 32;
  float fTemp4 = temp4 * 1.8 + 32;
  float fTemp5 = temp5 * 1.8 + 32;
  float fTemp6 = temp6 * 1.8 + 32;
  
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Temperature one in Celsius :");
  Serial.print(temp1);
  Serial.println(" 'C");
  Serial.print("Temperature one in Fahrenheit :");
  Serial.print(fTemp1);
  Serial.println(" 'F");
   Serial.print("Temperature Two in Celsius :");
  Serial.print(temp2);
  Serial.println(" 'C");
  Serial.print("Temperature Two in Fahrenheit :");
  Serial.print(fTemp2);
  Serial.println(" 'F");
    Serial.print("Temperature Three in Celsius :");
  Serial.print(temp3);
  Serial.println(" 'C");
  Serial.print("Temperature Three in Fahrenheit :");
  Serial.print(fTemp3);
  Serial.println(" 'F");
    Serial.print("Temperature Four in Celsius :");
  Serial.print(temp4);
  Serial.println(" 'C");
  Serial.print("Temperature Four in Fahrenheit :");
  Serial.print(fTemp4);
  Serial.println(" 'F");
    Serial.print("Temperature Five in Celsius :");
  Serial.print(temp5);
  Serial.println(" 'C");
  Serial.print("Temperature Five in Fahrenheit :");
  Serial.print(fTemp5);
  Serial.println(" 'F");
    Serial.print("Temperature Six in Celsius :");
  Serial.print(temp6);
  Serial.println(" 'C");
  Serial.print("Temperature Six in Fahrenheit :");
  Serial.print(fTemp6);
  Serial.println(" 'F");
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_51(void)
{
  Serial.println("ncd.io IoT Wireless 6 Channel AC Current Sensor");
       Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      } 
       Serial.println(" ");
    float current1 = (((((data[24])<<16)+ (data[25])<<8)  + (data[26])) /1000.0); 
    float current2 = (((((data[28])<<16)+ (data[29])<<8)  + (data[30])) /1000.0);
    float current3 = (((((data[32])<<16)+ (data[33])<<8)  + (data[34])) /1000.0);
    float current4 = (((((data[36])<<16)+ (data[37])<<8)  + (data[38])) /1000.0);
    float current5 = (((((data[40])<<16)+ (data[41])<<8)  + (data[42])) /1000.0);
    float current6 = (((((data[44])<<16)+ (data[45])<<8)  + (data[46])) /1000.0);
     

  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Channel 1 AC Current in Amp :");
  Serial.print(current1,3);
  Serial.println(" Amp");
  Serial.print("Channel 2 AC Current in Amp :");
  Serial.print(current2,3);
  Serial.println(" Amp");
  Serial.print("Channel 3 AC Current in Amp :");
  Serial.print(current3,3);
  Serial.println(" Amp");
  Serial.print("Channel 4 AC Current in Amp :");
  Serial.print(current4,3);
  Serial.println(" Amp");
  Serial.print("Channel 5 AC Current in Amp :");
  Serial.print(current5,3);
  Serial.println(" Amp");
  Serial.print("Channel 6 AC Current in Amp :");
  Serial.print(current6,3);
  Serial.println(" Amp");
   
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}
int sensor_type_52(void)
{
  Serial.println("ncd.io IoT Wireless 2 Channel 4-20mA Current Receiver");
       Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      } 
       Serial.println(" ");
  int adc_ch1 = ((data[24] * 256) + data[25]);   
  int adc_ch2 = ((data[26] * 256) + data[27]); 
  float mA1 = adc_ch1 * 0.0006863;
  float mA2 = adc_ch2 * 0.0006863;
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Analog input at Channel One :");
  Serial.println(adc_ch1);
  Serial.print("Analog input at Channel Two :");
  Serial.println(adc_ch2);
  Serial.print("Channel 1 4-20mA value in mA :");
  Serial.println(mA1,3);
  Serial.print("Channel 2 4-20mA value in mA :");
  Serial.println(mA2,3);
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}  

int sensor_type_53(void)
{
 Serial.println("ncd.io IoT Wireless Particulate Matter, CO2,Temperature, and Humidity Sensor");
       Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      } 
       Serial.println(" ");
    float MassConcentrationPM1_0 = (((data[24])<<24)+((data[25])<<16)+((data[26])<<8)+(data[27]));
  float MassConcentrationPM2_5 = (((data[28])<<24)+((data[29])<<16)+((data[30])<<8)+(data[31]));
  float MassConcentrationPM4_0 = (((data[32])<<24)+((data[33])<<16)+((data[34])<<8)+(data[35]));
  float MassConcentrationPM10_0 = (((data[36])<<24)+((data[37])<<16)+((data[38])<<8)+(data[39]));
  float NumberConcentrationPM0_5 = (((data[40])<<24)+((data[41])<<16)+((data[42])<<8)+(data[43]));
  float NumberConcentrationPM1_0 = (((data[44])<<24)+((data[45])<<16)+((data[46])<<8)+(data[47]));
  float NumberConcentrationPM2_5 = (((data[48])<<24)+((data[49])<<16)+((data[50])<<8)+(data[51]));
  float NumberConcentrationPM4_0 = (((data[52])<<24)+((data[53])<<16)+((data[54])<<8)+(data[55]));
  float NumberConcentrationPM10_0 = (((data[56])<<24)+((data[57])<<16)+((data[58])<<8)+(data[59]));
  float TypicalParticleSize = (((data[60])<<24)+((data[61])<<16)+((data[62])<<8)+(data[63]));
  
  float CO2 = ((((data[68])<<24) + (data[69] <<16)+ (data[70] <<8)+ data[71]) /100.0);        
  float humidity = ((((data[64]) * 256) + data[65]) /100.0);
  //int16_t cTempint = (((uint16_t)(data[26])<<8)| data[27]);
  float cTemp = (((data[66])<<8)| data[67]) /100.0;
  float fTemp = cTemp * 1.8 + 32;
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);
  Serial.print("Relative Humidity :");
  Serial.print(humidity);
  Serial.println(" %RH");
  Serial.print("Temperature in Celsius :");
  Serial.print(cTemp);
  Serial.println(" 'C");
  Serial.print("Temperature in Fahrenheit :");
  Serial.print(fTemp);
  Serial.println(" 'F");
  Serial.print("Co2 in ppm :");
  Serial.print(CO2);
  Serial.println(" ppm");
  Serial.print("MassConcentrationPM1.0 :");
  Serial.print(MassConcentrationPM1_0/100.00);
  Serial.println(" ");
  Serial.print("MassConcentrationPM2.5 :");
  Serial.print(MassConcentrationPM2_5/100.00);
  Serial.println(" ");
  Serial.print("MassConcentrationPM4.0 :");
  Serial.print(MassConcentrationPM4_0/100.00);
  Serial.println(" ");
  Serial.print("MassConcentrationPM10.0 :");
  Serial.print(MassConcentrationPM10_0/100.00);
  Serial.println(" ");
  
  Serial.print("NumberConcentrationPM0.5 :");
  Serial.print(NumberConcentrationPM0_5/100.00);
  Serial.println(" ");
  Serial.print("NumberConcentrationPM1.0 :");
  Serial.print(NumberConcentrationPM1_0/100.00);
  Serial.println(" ");
  Serial.print("NumberConcentrationPM2.5 :");
  Serial.print(NumberConcentrationPM2_5/100.00);
  Serial.println(" ");
  Serial.print("NumberConcentrationPM4.0 :");
  Serial.print(NumberConcentrationPM4_0/100.00);
  Serial.println(" ");
  Serial.print("NumberConcentrationPM10.0 :");
  Serial.print(NumberConcentrationPM10_0/100.00);
  Serial.println(" ");
  Serial.print("TypicalParticleSize :");
  Serial.print(TypicalParticleSize/100.00);
  Serial.println(" ");
  
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_61(void)

{
  Serial.println("ncd.io IoT Wireless pH and Temperature Sensor");
     Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      }         

  float pH = ((((data[24]) * 256) + data[25]) /100.0);
  int16_t cTempint_pH = (((uint16_t)(data[26])<<8)| data[27]);
  float cTemp_pH = (float)cTempint_pH /100.0;
  float fTemp_pH = cTemp_pH * 1.8 + 32;
  
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.println(" ");
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);

  Serial.println("////////////////// ");  
  Serial.print("Solution pH :");
  Serial.print(pH);
  Serial.println(" pH");
  Serial.print("pH Temperature in Celsius :");
  Serial.print(cTemp_pH);
  Serial.println(" C");
  Serial.print("pH Temperature in Fahrenheit :");
  Serial.print(fTemp_pH);
  Serial.println(" F");
  Serial.println("////////////////// "); 
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_62(void)

{
  Serial.println("ncd.io IoT Wireless ORP and Temperature Sensor");
     Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      }         

  float ORP = (((data[24]) * 256) + data[25]);
  int16_t cTempint_ORP = (((uint16_t)(data[26])<<8)| data[27]);
  float cTemp_ORP = (float)cTempint_ORP /100.0;
  float fTemp_ORP = cTemp_ORP * 1.8 + 32;

  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.println(" ");
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);

  Serial.println("////////////////// ");  
  Serial.print("Solution ORP :");
  Serial.print(ORP);
  Serial.println(" mV");
  Serial.print("ORP Temperature in Celsius :");
  Serial.print(cTemp_ORP);
  Serial.println(" C");
  Serial.print("pH Temperature in Fahrenheit :");
  Serial.print(fTemp_ORP);
  Serial.println(" F");
  
  Serial.println("////////////////// "); 
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_63(void)

{
  Serial.println("ncd.io IoT Wireless ORP pH and Temperature Sensor");
     Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      }         

  float ORP = (((data[24]) * 256) + data[25]);
  int16_t cTempint_ORP = (((uint16_t)(data[26])<<8)| data[27]);
  float cTemp_ORP = (float)cTempint_ORP /100.0;
  float fTemp_ORP = cTemp_ORP * 1.8 + 32;
  
  float pH = ((((data[28]) * 256) + data[29]) /100.0);
  int16_t cTempint_pH = (((uint16_t)(data[30])<<8)| data[31]);
  float cTemp_pH = (float)cTempint_pH /100.0;
  float fTemp_pH = cTemp_pH * 1.8 + 32;
    
  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.println(" ");
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);

  Serial.println("////////////////// ");  
  Serial.print("Solution ORP :");
  Serial.print(ORP);
  Serial.println(" mV");
  Serial.print("ORP Temperature in Celsius :");
  Serial.print(cTemp_ORP);
  Serial.println(" C");
  Serial.print("pH Temperature in Fahrenheit :");
  Serial.print(fTemp_ORP);
  Serial.println(" F");
  Serial.print("Solution pH :");
  Serial.print(pH);
  Serial.println(" pH");
  Serial.print("pH Temperature in Celsius :");
  Serial.print(cTemp_pH);
  Serial.println(" C");
  Serial.print("pH Temperature in Fahrenheit :");
  Serial.print(fTemp_pH);
  Serial.println(" F");  
  Serial.println("////////////////// "); 
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

int sensor_type_502(void)

{
  Serial.println("ncd.io IoT Wireless Vibration Accelero Gyro Magneto Temperature Sensor");
     Serial.print("Sensor MAC Address  ");
      for( int i = 4; i < 12 ; i ++)
      {
        Serial.print(data[i],HEX);
      }         
 Serial.println(" ");
  float rms_x = ((int32_t)(((data[24])<<16) + ((data[25])<<8) + (data[26])));
  rms_x = sign_extend_24_32(rms_x);
  rms_x = rms_x/100;
  float rms_y = ((int32_t)(((data[27])<<16) + ((data[28])<<8) + (data[29])));
  rms_y = sign_extend_24_32(rms_y);
  rms_y = rms_y/100;
  float rms_z = ((int32_t)(((data[30])<<16) + ((data[31])<<8) + (data[32])));
  rms_z = sign_extend_24_32(rms_z);
  rms_z = rms_z/100;

  float max_x = ((int32_t)(((data[33])<<16) + ((data[34])<<8) + (data[35])));
  max_x = sign_extend_24_32(max_x);
  max_x = max_x/100;
  Serial.println(data[33]);Serial.println(data[34]);Serial.println(data[35]);
  
  float max_y = ((int32_t)(((data[36])<<16) + ((data[37])<<8) + (data[38])));
  max_y = sign_extend_24_32(max_y);
  max_y = max_y/100;
  
  float max_z = ((int32_t)(((data[39])<<16) + ((data[40])<<8) + (data[41])));
  max_z = sign_extend_24_32(max_z);
  max_z = max_z/100;
  

  float min_x = ((int32_t)(((data[42])<<16) + ((data[43])<<8) + (data[44])));
  min_x = sign_extend_24_32(min_x);
  min_x = min_x/100;
  
  float min_y = ((int32_t)(((data[45])<<16) + ((data[46])<<8) + (data[47])));
  min_y = sign_extend_24_32(min_y);
  min_y = min_y/100;
  
  float min_z = ((int32_t)(((data[48])<<16) + ((data[49])<<8) + (data[50])));
  min_z = sign_extend_24_32(min_z);
  min_z = min_z/100;
  
  int cTemp = ((((data[51]) * 256) + data[52]));


  float raw_acc_x = ((uint32_t)(((data[53])<<16) + ((data[54])<<8) + (data[55])));
  raw_acc_x = sign_extend_24_32(raw_acc_x);
  raw_acc_x = raw_acc_x/100;
  
  float raw_acc_y = ((int32_t)(((data[56])<<16) + ((data[57])<<8) + (data[58])));
  raw_acc_y = sign_extend_24_32(raw_acc_y);
  raw_acc_y = raw_acc_y/100;
  
 // int data_in = ((uint32_t)(((data[59])<<16) + ((data[60])<<8) + (data[61])));
 // int raw_acc_z = sign_extend_24_32(data_in);
  float raw_acc_z = ((uint32_t)(((data[59])<<16) + ((data[60])<<8) + (data[61])));
  raw_acc_z = sign_extend_24_32(raw_acc_z);
  raw_acc_z = raw_acc_z/100;
  
  float mag_x = ((uint32_t)(((data[62])<<16) + ((data[63])<<8) + (data[64])));
  mag_x = sign_extend_24_32(mag_x);
  mag_x = mag_x/100;
  
  float mag_y = ((uint32_t)(((data[65])<<16) + ((data[66])<<8) + (data[67])));
  mag_y = sign_extend_24_32(mag_y);
  mag_y = mag_y/100;
  
  float mag_z = ((uint32_t)(((data[68])<<16) + ((data[69])<<8) + (data[70])));
  mag_z = sign_extend_24_32(mag_z);
  mag_z = mag_z/100;

  float gyro_x = ((uint32_t)(((data[71])<<16) + ((data[72])<<8) + (data[73])));
  gyro_x = sign_extend_24_32(gyro_x);
  //gyro_x = gyro_x/100;
  
  float gyro_y = ((uint32_t)(((data[74])<<16) + ((data[75])<<8) + (data[76])));
  gyro_y = sign_extend_24_32(gyro_y);
  //gyro_y = gyro_y/100;
  
  float gyro_z = ((uint32_t)(((data[77])<<16) + ((data[78])<<8) + (data[79])));
  gyro_z = sign_extend_24_32(gyro_z);
  //gyro_z = gyro_z/100;

  float battery = ((data[18] * 256) + data[19]);
  float voltage = 0.00322 * battery;
  Serial.println(" ");
  Serial.print("Wireless Transmission Counter  ");
  Serial.println(data[20]);
  Serial.print("Sensor Number  ");
  Serial.println(data[16]);
  Serial.print("Sensor Type  ");
  Serial.println(ncd_sensor_type);
  Serial.print("Firmware Version  ");
  Serial.println(data[17]);

  Serial.println("////////////////// ");  
  Serial.print("RMS vibration in X-axis :");
  Serial.print(rms_x,2);
  Serial.println(" mg");
  Serial.print("RMS vibration in Y-axis :");
  Serial.print(rms_y,2);
  Serial.println(" mg");
  Serial.print("RMS vibration in Z-axis :");
  Serial.print(rms_z,2);
  Serial.println(" mg");
  
  Serial.print("Max vibration in X-axis :");
  Serial.print(max_x,2);
  Serial.println(" mg");
  Serial.print("Max vibration in Y-axis :");
  Serial.print(max_y,2);
  Serial.println(" mg");
  Serial.print("Max vibration in Z-axis :");
  Serial.print(max_z,2);
  Serial.println(" mg");
  Serial.print("Min vibration in X-axis :");
  Serial.print(min_x,2);
  Serial.println(" mg");
  Serial.print("Min vibration in Y-axis :");
  Serial.print(min_y,2);
  Serial.println(" mg");
  Serial.print("Min vibration in Z-axis :");
  Serial.print(min_z,2);
  Serial.println(" mg"); 

  Serial.print("Raw Acceleration in X-axis :");
  Serial.print(raw_acc_x,2);
  Serial.println(" mg");
  Serial.print("Raw Acceleration in Y-axis :");
  Serial.print(raw_acc_y,2);
  Serial.println(" mg");
  Serial.print("Raw Acceleration in Z-axis :");
  Serial.print(raw_acc_z,2);
  Serial.println(" mg");
    
  Serial.print("Raw Magneto in X-axis :");
  Serial.print(mag_x,2);
  Serial.println(" ");
  Serial.print("Raw Magneto in Y-axis :");
  Serial.print(mag_y,2);
  Serial.println(" ");
  Serial.print("Raw Magneto in Z-axis :");
  Serial.print(mag_z,2);
  Serial.println(" ");

  Serial.print("Raw gyro in X-axis :");
  Serial.print(gyro_x,2);
  Serial.println(" ");
  Serial.print("Raw gyro in Y-axis :");
  Serial.print(gyro_y,2);
  Serial.println(" ");
  Serial.print("Raw gyro in Z-axis :");
  Serial.print(gyro_z,2);
  Serial.println(" ");

  Serial.print("Temperature in Celsius :");
  Serial.print(cTemp);
  Serial.println(" C");
  Serial.print("ADC value:");
  Serial.println(battery);
  Serial.print("Battery Voltage:");
  Serial.print(voltage);
  Serial.println("\n");
  if (voltage < 1)
          {
    Serial.println("Time to Replace The Battery");
          }
}

void loop()
{
  
  if (Serial1.available())
  {
    data[0] = Serial1.read();
    delay(k);
   if(data[0]==0x7E)
    {
    while (!Serial1.available());
    for ( i = 1; i< length; i++)
      {
      data[i] = Serial1.read();
      delay(1);
      }
    if(data[15]==0x7F)  /////// to check if the recive data is correct
 {
  ncd_sensor_type = data[21]*256 + data[22];
  if(ncd_sensor_type==5)  //////// make sure the sensor type is correct
  {
  sensor_type_5();
  }

    else if(ncd_sensor_type==8)  //////// make sure the sensor type is correct
  {
  sensor_type_8();
  }
  
  else if(ncd_sensor_type==61)  //////// make sure the sensor type is correct
  {
  sensor_type_61();
  }

  else if(ncd_sensor_type==44)  //////// make sure the sensor type is correct
  {
  sensor_type_44();
  }
  else if(ncd_sensor_type==49)  //////// make sure the sensor type is correct
  {
  sensor_type_49();
  }
  else if(ncd_sensor_type==51)  //////// make sure the sensor type is correct
  {
  sensor_type_51();
  }
  else if(ncd_sensor_type==52)  //////// make sure the sensor type is correct
  {
  sensor_type_52();
  } 
  else if(ncd_sensor_type==53)  //////// make sure the sensor type is correct
  {
  sensor_type_53();
  }
  else if(ncd_sensor_type==62)  //////// make sure the sensor type is correct
  {
  sensor_type_62();
  }
  else if(ncd_sensor_type==63)  //////// make sure the sensor type is correct
  {
  sensor_type_63();
  } 
  else if(ncd_sensor_type==502)  //////// make sure the sensor type is correct
  {
  sensor_type_502();
  } 
 }
else
{
      for ( i = 0; i< 29; i++)
    {
      Serial.print(data[i]);
      Serial.print(" , ");
      delay(1);
    }
}
    }
  }
 
}
