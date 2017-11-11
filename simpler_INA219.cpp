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
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

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
simpler_INA219::simpler_INA219(uint8_t addr,int shunt) 
{
  ina219_i2caddr = addr;
  ina219_shuntValueMillOhm=shunt;  
  ina219_currentScale=3;
  for(int i=0;i<4;i++)
    ina219_zeros[i]=0;
  multiSampling=0;
  reconfigure();
}

/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void simpler_INA219::wireWriteRegister (uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(ina219_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
    Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
    Wire.write(value & 0xFF);              // Lower 8-bits
  #else
    Wire.send(reg);                        // Register
    Wire.send(value >> 8);                 // Upper 8-bits
    Wire.send(value & 0xFF);               // Lower 8-bits
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void simpler_INA219::wireReadRegister(uint8_t reg, uint16_t *value)
{

  Wire.beginTransmission(ina219_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
  #else
    Wire.send(reg);                        // Register
  #endif
  Wire.endTransmission();
  
  delay(1); // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(ina219_i2caddr, (uint8_t)2);  
  #if ARDUINO >= 100
    // Shift values to create properly formed integer
    *value = ((Wire.read() << 8) | Wire.read());
  #else
    // Shift values to create properly formed integer
    *value = ((Wire.receive() << 8) | Wire.receive());
  #endif
}

/**
 * 
 */
void simpler_INA219::reconfigure(void)
{
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    scaler[ina219_currentScale].registerValue |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(INA219_REG_CONFIG, config);
  
  multiplier=1<<ina219_currentScale;
  multiplier=(scaler[ina219_currentScale].millivoltRange/(float)ina219_shuntValueMillOhm)/multiplier;
  multiplier/=4.096;
  
}


/**************************************************************************/
/*! 
    @brief  Setups the HW (defaults to 32V and 2A for calibration values)
*/
/**************************************************************************/
void simpler_INA219::begin() 
{
  Wire.begin();    
  reconfigure();
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
float simpler_INA219::getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return value * 0.001; // 10mv / tick
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
int simpler_INA219::getOneCurrent_mA() {
again:    
  int raw=getShuntVoltage_raw();  
  // Check if we are below 1/4 of the amp ...
  if(raw<=1024 && ina219_currentScale )
  {
      ina219_currentScale--;
      reconfigure();
      goto again;
  }
  // same for upper boundary
  if(raw>1024*3 && ina219_currentScale<4)
  {
      ina219_currentScale++;
      reconfigure();
      goto again;
  }
  
  raw-=ina219_zeros[ina219_currentScale];
  float valueDec=raw;
  valueDec*=multiplier;
  return valueDec;
}

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
        for(int i=0;i<8;i++)
        {
            val+=getShuntVoltage_raw();
            delay(1);
        }
        val=val/8;
        ina219_zeros[i]=val;
        
    }
    ina219_currentScale=old;    
}