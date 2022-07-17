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
simpler_INA219::simpler_INA219( lnI2C *i2c, uint8_t addr, int shunt) 
{
   xAssert(i2c);
  _i2c=i2c;
  _i2cAdr = addr;
  _shuntValueMillOhm=shunt;  
  _multiSampling=1;
  _highVoltageScale=false;
  setScaler(PGA8);
  
  for(int i=0;i<4;i++)
    _zeros[i]=0;
  
  // 12 bits ADC on a 40 mv Range
  _multiplier=40./(float)(_shuntValueMillOhm);
  _multiplier/=4.096;
  //  compute cal
  // 
  float calf=0.04096;
  calf/=(float)shunt/1000.;
  calf/=4./32768.; // max 4A
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
void simpler_INA219::readRegister(uint8_t reg, uint16_t *value)
{
uint8_t datas[2];
    _i2c->write(_i2cAdr,1,&reg);
    _i2c->read(_i2cAdr,2, datas);
    *value=(datas[0]<<8)+datas[1];
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
                    (3<<3) | // both I & V = 12 bits
                    (3<<7)                    
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
  readRegister(INA219_REG_BUSVOLTAGE, &value);  
  return (int16_t)(value );
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
  readRegister(INA219_REG_SHUNTVOLTAGE, &value);
  value &= pgaMask[_currentIScale]; // remove the multiple sign
#warning Negative current is not handled properly!
  return (int16_t) value;
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/

int MAX_SCALE[4]={
    (1<<12)-1,
    (1<<13)-1,
    (1<<14)-1,
    (1<<15)-1,
};

int simpler_INA219::getOneCurrent_mA() 
{
again:    
    uint16_t current;
    
  readRegister(INA219_REG_CURRENT,&current);

  if(current & (1<<15))
  {
    return 0;
  }
  return (current+5)/10;

  int cur=simpler_INA219::getBusVoltage_raw();
  Logger("scale = %d Overflow = %x Ready= %x\n",_currentIScale, cur & 1,cur & 2);
  uint16_t aa ;
  readRegister(4,&aa);
  Logger("Internal current %d\n",aa);


  int raw=getShuntVoltage_raw();    
  bool redo=false;

  Logger("Raw %d 0x%x\n",raw,raw);  
#if 0
  if(raw>MAX_SCALE[_currentIScale] && _currentIScale!=PGA8)
  {
    _currentIScale=(simpler_INA219::PGA)(_currentIScale+1);
    redo=true;
  }
  if(redo && raw<(MAX_SCALE[_currentIScale]>>1) && _currentIScale!=PGA1)
  {
    _currentIScale=(simpler_INA219::PGA)(_currentIScale-1);
    redo=true;
  }
  if(redo)
  {
    reconfigure();
    xDelay(10);
    goto again;
  }
#endif  
  raw-=_zeros[_currentIScale];  
 
  float valueDec=raw;
  valueDec*=_multiplier;
  return (int)valueDec;
}
/**
 * \fn getCurrent_mA
 */
int simpler_INA219::getCurrent_mA() 
{
    return getOneCurrent_mA();   
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
            int raw=getShuntVoltage_raw();
            raw<<=_currentIScale; 
            val+=raw;
            xDelay(10);
        }
        val=val/8;
        _zeros[i]=val;
        
    }
    setScaler(old);
}
// EOF