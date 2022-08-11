/**************************************************************************/
/*! 

Mean00: Derived from the following :


    @file     Adafruit_INA219.h
    @author   K. Townsend (Adafruit Industries)
    @author   modified by mean00@gmail.com
	@license  BSD (see license.txt)
	
	This is a library for the Adafruit INA219 breakout board
	----> https://www.adafruit.com/products/???
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
#pragma once
#include "lnArduino.h"
#include "lnI2C.h"
#define INA219_ADDRESS                         (0x40)    // 1000000 (A0+A1=GND)

/**
 * \class simpler_INA219
 * \brief 
 * @param addr
 * @param shutResistorMilliOhm
 */
class simpler_INA219
{
 protected:
  enum PGA          // Pre amp divider, the signal is divided by 1,2,4 or 8
  {
      PGA1=0,
      PGA2=1,
      PGA4=2,
      PGA8=3
  };


protected:
  uint8_t _i2cAdr;               // default is INA219_ADDRESS
  int     _shuntValueMillOhm;    // default is 100 mOhm
  PGA     _currentIScale;        // 0: PGA=1, 1: PG2=2, 2: PGA=4, 3: PGA=8
  bool    _highVoltageScale;     // false = 0..16v, true 0..32v
  int     _zeros[4];       // offset correction for current
  float   _multiplier;            // current = multiplier * shuntvalue  
  lnI2C   *_i2c;                 // interface to use
  int     _multiSampling;
  
  
protected: 
  void    reconfigure(void);
  void    writeRegister(uint8_t reg, uint16_t value);
  uint16_t readRegister(uint8_t reg);
  int16_t getBusVoltage_raw(void);
  int     getOneCurrent_mA(void);
  void    setScaler(PGA nw)  {_currentIScale=nw; }
  
public: // for debug
  PGA     getCurrentScaler() {return _currentIScale;}
  int16_t getShuntVoltage_raw(void);
  int     getShuntVoltage_mV(void);  /// Returns voltage across the shunt

public:  // normal API
           simpler_INA219(lnI2C *i2c, int maxCurrentinA=4, uint8_t addr = INA219_ADDRESS, int shutResistorMilliOhm=100); // 0.1 ohm by default, like Adafruit board
  float    getVoltage_V(void);     /// Returns bus voltage in volt
  int      getCurrent_mA(void);       /// Returns current in mA, /!\ we dont deal with the sign
  void     setZero(int offetMa);

  void     setMultiSampling(int v){_multiSampling=v;reconfigure();} // this is 2^n sample : 0=>1 sample, 1=> 2 sample....4=>16 samples
  void     autoZero(); // Call this automatically set the zero offset, BE 100% SURE NO CURRENT IS FLOWING !
};

// EOF
