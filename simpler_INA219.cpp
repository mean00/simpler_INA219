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




/**************************************************************************/
/*! 
    @brief  Instantiates a new INA219 class
*/
/**************************************************************************/
simpler_INA219::simpler_INA219(  lnI2C *i2c, int maxCurrentinA,uint8_t addr, int shunt) 
{
   xAssert(i2c);
  _i2c=i2c;
  _i2cAdr = addr;
  _shuntValueMillOhm=shunt;  
  _multiSampling=1;
  _highVoltageScale=false;
  
  
  for(int i=0;i<4;i++)
    _zeros[i]=0;
  
  // 12 bits ADC on a 40 mv Range
  _multiplier=40./(float)(_shuntValueMillOhm);
  _multiplier/=4.096;
  // select right PGA
  int maxShuntVoltage=shunt*maxCurrentinA; // result in mv
  PGA divider;
  if(maxShuntVoltage>320) {xAssert(0);} // wrong configuration
  else if(maxShuntVoltage>160) divider=PGA8;
    else if(maxShuntVoltage>80) divider=PGA4;
        else if(maxShuntVoltage>40) divider=PGA2;
            else divider=PGA1;

  setScaler(divider);


  //  compute cal  
  float calf=0.04096;
  calf/=(float)shunt/1000.;
  calf/=(float)maxCurrentinA/32768.; // max 4A
  int cal=calf;
  
  writeRegister(INA219_REG_CALIBRATION,cal); // 4A max, 20 Ohm
  reconfigure();
}
/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void simpler_INA219::writeRegister (uint8_t reg, uint16_t value)
{   
    uint8_t datas[3]={reg,(uint8_t)(value>>8),(uint8_t)(value&0xff)};
    _i2c->write(_i2cAdr,3,datas);
}

/**************************************************************************/
/*! 
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t simpler_INA219::readRegister(uint8_t reg)
{
uint8_t datas[2];
    _i2c->write(_i2cAdr,1,&reg);
    _i2c->read(_i2cAdr,2, datas);
    return (datas[0]<<8)+datas[1];
}

/**
 * 
 */
void simpler_INA219::reconfigure(void)
{   
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V*_highVoltageScale |
                    INA_PGA(_currentIScale) |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS |
                    (10<<3) | // both I & V = 12 bits, 4 samples
                    (10<<7)                    
                    ;
  writeRegister(INA219_REG_CONFIG, config);
}


/**************************************************************************/
/*! 
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t simpler_INA219::getBusVoltage_raw() 
{
  uint16_t value;
  return (int16_t)(readRegister(INA219_REG_BUSVOLTAGE));
}

/**
 *  \fn value2volt
 */
static float value2volt(int value)
{
  float flat=(float)(value>>3)*4.; // remove overflow & ready, in 1 mv step
  flat=flat/1000.;
  return flat;
}
/**************************************************************************/
/*! 
    @fn     getVoltage_V
    @brief  Get Bus voltage
*/
/**************************************************************************/
float simpler_INA219::getVoltage_V() 
{
  uint16_t value = getBusVoltage_raw();
  
  float flat=value2volt(value);
  bool redo=false;

  if(!_highVoltageScale && flat > 15.) // switch to high voltage
  {
        _highVoltageScale=true;
        redo=true;   
  }
  if(!redo && _highVoltageScale && flat < 13.)
  {
        _highVoltageScale=false;
        redo=true;
  }
  if(redo)
  {
        reconfigure();    
        xDelay(5);
        value = getBusVoltage_raw();
        flat=value2volt(value);
  }
  return flat;
}


/**************************************************************************/
/*! 
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
static const uint16_t pgaMask[4]={0x8FFF,0x9FFF,0xBFff,0xFFFF};
int16_t simpler_INA219::getShuntVoltage_raw() 
{
  uint16_t value;
  value=readRegister(INA219_REG_SHUNTVOLTAGE);
  value &= pgaMask[_currentIScale]; // remove the multiple sign
#warning Negative current is not handled properly!
  return (int16_t) value;
}

const int MAX_SCALE[4]={
    (1<<12)-1,
    (1<<13)-1,
    (1<<14)-1,
    (1<<15)-1,
};
/**************************************************************************/
/*! 
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/

int simpler_INA219::getCurrent_mA() 
{
  uint16_t current=  readRegister(INA219_REG_CURRENT);

  if(current & (1<<15)) // sign
  {
    return 0;
  }
  current= (current+5)/10;
  if(current>_zeros[_currentIScale])
    current-=_zeros[_currentIScale];
  if(current<6) return 0;
  return current;
}
/**
 * 
 */
void simpler_INA219::setZero(int offetMa)
{
     for(int i=0;i<4;i++) _zeros[i]=offetMa;
}
/**
 * \fn autoZero
 */
void simpler_INA219::autoZero()
{
    PGA old=_currentIScale;
    for(int i=0;i<4;i++)
    {
        _currentIScale=(PGA)i;
        reconfigure();
        xDelay(10);
        int val=0;
        for(int j=0;j<8;j++)
        {
            val+=  readRegister(INA219_REG_CURRENT);
            xDelay(10);
        }
        val=val/8;
        _zeros[i]=val;
        
    }
    setScaler(old);
}
// EOF