#include "lnArduino.h"
#include "lnGPIO.h"
#include "simpler_INA219.h"

#define LED LN_SYSTEM_LED

void setup()
{
    lnPinMode(LED,lnOUTPUT);
    lnPinMode(PB11,lnALTERNATE_OD); // SDA2
    lnPinMode(PB10,lnALTERNATE_PP); // SCLK2
}
/**
 * 
 */


      int val;
void loop()
{
   
    lnI2C *i2c=new lnI2C(1); // I2C2
    
    i2c->begin();
#if 0
    while(1)
    {
        for(int i=63;i<65;i++)
        {
            xDelay(100);
            Logger("Scanning %d\n",i);
            
            i2c->setAddress(i);
        //   while(1)
            {
                if(!i2c->write(0,NULL))
                {
                    xDelay(10);
                    continue;
                }else
                {
                    Logger(">>>found something at address %d (0x%0x)\n",i,i);
                }
            }
            
        }
    }
#endif


    simpler_INA219 *ina=new simpler_INA219(i2c);
    

    while(1)
    {
        float volt=ina->getBusVoltage_V();
        int current=ina->getCurrent_mA();

        Logger("Volage=%2.2f Amp=%d\n",volt,current);
        lnDigitalToggle(LED);
        xDelay(1000);

    }
}
