/**************************************************************************/
/*! 
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

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define INA219_ADDRESS                         (0x40)    // 1000000 (A0+A1=GND)


/**
 * \class simpler_INA219
 * \brief 
 * @param addr
 * @param shutResistorMilliOhm
 */
class simpler_INA219
{
 public:
           simpler_INA219(uint8_t addr = INA219_ADDRESS, int shutResistorMilliOhm=100,TwoWire *w=NULL); // 0.1 ohm by default, like Adafruit board
  float    getBusVoltage_V(void);
  int      getShuntVoltage_mV(void);
  int      getCurrent_mA(void);
  int      setMultiSampling(int v){multiSampling=v;} // this is 2^n sample : 0=>1 sample, 1=> 2 sample....4=>16 samples
  void     autoZero(); // Call this automatically set the zero offset, BE 100% SURE NO CURRENT IS FLOWING !
        
  
  /**
   * \fn  setZeroOffset
   * \brief call this to set the zero value if you know it, i.e. the value returned by getShuntVoltage_raw when current =0
   * \param scale : 0-> 40mv Scale, ....3 ->320 mv scale
   * \param zero  : The value measured by getShuntVoltage_raw when not current flows
  */
  void     setZeroOffset(int scale,int zero) 
            {
                    ina219_zeros[scale]=zero;
            }

 protected:
  uint8_t ina219_i2caddr;
  int     ina219_shuntValueMillOhm;
  int     ina219_currentScale;
  int     ina219_zeros[4];
  float   multiplier;
  int     multiSampling;
  TwoWire *_w;
  
  
 protected: 
  void    reconfigure(void);
  void    wireWriteRegister(uint8_t reg, uint16_t value);
  void    wireReadRegister(uint8_t reg, uint16_t *value);
  int16_t getBusVoltage_raw(void);
  int     getOneCurrent_mA(void);
  void    setScaler(int nw) ;
 public:  
   // These are for debug and calibration
  int     getCurrentScaler() {return ina219_currentScale;}
  int     getShuntVoltage_raw(void);
  int     getResolutionMicroAmp();
  bool    highVoltageScale;
};
