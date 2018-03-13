//
//
// Sample demo of simpler_INA219
//
#include <Wire.h>
#include <simpler_INA219.h>


simpler_INA219 *sensor219; // Declare and instance of INA219

#define VERBOSE 1
/**
 * 
 */
void setup(void) 
{
      
  Serial.begin(57600);    
  Serial.print("Start");
  Serial.print("Sensor initialization\n");
  sensor219=new simpler_INA219(INA219_ADDRESS,20);  // we use 20 mOhm shunt
  sensor219->setMultiSampling(2);                   // 4 samples
  Serial.println("Setup done\n");  
#if 0 // if you want to do autozero...
  Serial.print("Autozeroing\n");  
  sensor219->autoZero();                            // /!\ Must have zero current here!!!
  Serial.print("Autozeroing done\n"); 
#else
  sensor219->setZeroOffset(0,-1); // or if you know them.... beware they can change a bit over time
  sensor219->setZeroOffset(1,-1); // the error is then dependant on the shunt, see getResolutionMicroAmp()
  sensor219->setZeroOffset(2,-1);
  sensor219->setZeroOffset(3,-1);
#endif
}
/**
 * 
 */
void float2str(char *s,float f,const char *unit)
{
  float scale=1;
  const char *ext=" ";
  if(f<1.1)
  {
    f=f*1000.;
     sprintf(s,"%04dm%s",(int)f,unit);
     return;
  }
  
  int zleft=(int)(f);
  float g=f-(float)(zleft);
  int right=(int)(g*10.);
  sprintf(s,"%02d",zleft);
  s[2]='.';
  s+=3;
  sprintf(s,"%01d",right);
  s+=1;
  sprintf(s,"%s%s",ext,unit);
  
}
/**
 * 
 */
void loop(void) 
{
  float busVoltage = 0;
  int current = 0; // Measure in milli amps
  

  char stA[32];
  char stV[32];


  busVoltage = sensor219->getBusVoltage_V();
  current = sensor219->getCurrent_mA();
  
  sprintf(stA,"Scale=%d",sensor219->getCurrentScaler()); // current scale
  Serial.println(stA);

  sprintf(stA,"uA per bit=%d",sensor219->getResolutionMicroAmp()); // get accuracy with current scale
  Serial.println(stA);
  
  sprintf(stA,"mA =%d",current); // get accuracy with current scale
  Serial.println(stA);
    
 
  float2str(stV,busVoltage,"V");  
  Serial.println(stV);
  Serial.println("-----------------\n");
 
  
  int raw=sensor219->getShuntVoltage_raw();
  sprintf(stA,"RAW = %d\n",(int)raw);
  Serial.println(stA);
  delay(500);
}
// EOF

