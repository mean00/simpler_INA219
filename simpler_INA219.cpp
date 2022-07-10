/**************************************************************************/
/*! 
    @file     simpler_INA219.cpp
    @author   K.Townsend (Adafruit Industries)
    @author   modified by mean00@gmail.com

	@license  BSD (see license.txt)

	
	Driver for the INA219 current sensor

	This is a library for the Adafruit INA219 breakout
	----> https://www.adafruit.com/products/???
		
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "simpler_INA219.h"
#include "simpler_INA219_internal.h"



/**
 */
typedef struct 
{
    int   registerValue;
    float millivoltRange;
}ina219Scaler;
/**
 */
static const ina219Scaler scaler[4]=
{
    {INA219_CONFIG_GAIN_1_40MV,40.},
    {INA219_CONFIG_GAIN_2_80MV,80.},
    {INA219_CONFIG_GAIN_4_160MV,160.},
    {INA219_CONFIG_GAIN_8_320MV,320.},
};


/**************************************************************************/
/*! 
    @brief  Instantiates a new INA219 class
*/
/**************************************************************************/
simpler_INA219::simpler_INA219( lnI2C *i2c, uint8_t addr, int shunt) 
{
  ina219_i2caddr = addr;
  ina219_shuntValueMillOhm=shunt;  
  setScaler(0);
  for(int i=0;i<4;i++)
    ina219_zeros[i]=0;
  multiSampling=0;
  highVoltageScale=0;
  xAssert(i2c);
  _i2c=i2c;
  reconfigure();
}
/**
 */
void simpler_INA219::setScaler(int nw) 
{
#if 0
    char bfer[40];
    sprintf(bfer,"%d => %d",ina219_currentScale,nw);
    Serial.println(bfer);
#endif    
    ina219_currentScale=nw;    
}
/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void simpler_INA219::wireWriteRegister (uint8_t reg, uint16_t value)
{   
    uint8_t datas[3]={reg,(uint8_t)(value>>8),(uint8_t)(value&0xff)};
    _i2c->write(ina219_i2caddr,3,datas);
}

/**************************************************************************/
/*! 
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void simpler_INA219::wireReadRegister(uint8_t reg, uint16_t *value)
{
uint8_t datas[2];
    _i2c->write(ina219_i2caddr,1,&reg);
    _i2c->read(ina219_i2caddr,2, datas);
    *value=(datas[0]<<8)+datas[1];
}

/**
 * 
 */
void simpler_INA219::reconfigure(void)
{   
#if 0
  // Reset chip
  wireWriteRegister(INA219_REG_CONFIG,INA219_CONFIG_RESET);
  delay(1);
#endif
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V*highVoltageScale |
                    scaler[ina219_currentScale].registerValue |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(INA219_REG_CONFIG, config);
  
  
  multiplier=(scaler[0].millivoltRange/(float)ina219_shuntValueMillOhm);
  multiplier/=4.096;
  
}


/**************************************************************************/
/*! 
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t simpler_INA219::getBusVoltage_raw() 
{
  uint16_t value;
  wireReadRegister(INA219_REG_BUSVOLTAGE, &value);
  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((value >> 3) * 4);
}

/**************************************************************************/
/*! 
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int simpler_INA219::getShuntVoltage_raw() 
{
  uint16_t value;
  wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}


/**************************************************************************/
/*! 
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
float simpler_INA219::getBusVoltage_V() 
{
  int16_t value = getBusVoltage_raw();
  
  float r=(float)value*0.001; // 1mv / tick
  if(highVoltageScale) 
  {
      if(r<13.)
      {
          highVoltageScale=0; // switch to low voltage
          reconfigure();           
      }
  }
  // low 16 v scale, more accurate ?
  else 
  {
      if(r>14.)
      {
           highVoltageScale=1;
           reconfigure();           // next one will be more accurate       
      }
  }
  return r; 
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
int simpler_INA219::getOneCurrent_mA() 
{
again:    
  int raw=getShuntVoltage_raw();  
  int araw=raw;
  if(araw<0) araw=-raw;
  int range=0;
  if(araw>15000) range=3;
  else if(araw>7000) range=2;
  else if(araw>3000) range=1;
  
  if(range!=ina219_currentScale)
  {
      setScaler(range);
      reconfigure();
      goto again;
  }
  raw-=ina219_zeros[ina219_currentScale];
  float valueDec=raw;
  valueDec*=multiplier;
  return (int)valueDec;
}
/**
 * 
 */
int simpler_INA219::getCurrent_mA() 
{
    if(!multiSampling) return getOneCurrent_mA();
    float sum=0;
    int linear=1<<multiSampling;
    for(int i=0;i<linear;i++)
    {
        sum+=getOneCurrent_mA();
        delay(1);
    }
    sum/=(float)linear;
    return (int)sum;
}

/**
 * 
 * @return 
 */
int simpler_INA219::getResolutionMicroAmp()
{
    return (int)(1000.*multiplier);
}
/**
 * 
 */
void simpler_INA219::autoZero()
{
    int old=ina219_currentScale;
    for(int i=0;i<4;i++)
    {
        ina219_currentScale=i;
        reconfigure();
        int val=0;
        for(int j=0;j<8;j++)
        {
            val+=getShuntVoltage_raw();
            delay(1);
        }
        val=val/8;
        ina219_zeros[i]=val;
        
    }
    setScaler(old);
}
// EOF