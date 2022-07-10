simpler_INA219
===============

This is a INA219 sensor driver for lnArduino, derived from the ADAFruit one.
It is quite different though.

The INA219 setup is complicated because the driver lets the INA219 compute automatically the current and power.
It means it has to be fed with the correct multipliers to do that. Computing these multipliers is complicated
and half hardcoded into the driver (i.e. set up once for all at the beginning).

On the other hand, once you get the shuntvoltage, computing the current is just a division that can be
done on the MCU, relatively cheaply.

The original driver does a multiplication anyway, so it's not really worth it to have that  complicated setup CPU wise.

This driver is much more simple and have the following features :

* Adjustable shunt resistor in the constructor (default is 100 mOhm)
* Adjustable i2c interface in the constructor (default is Wire, i.e. default hw i2c)
* Tested on  Arduino-STM32 and Arduino-Avr
* Auto scale change (it will pick the best scale automatically, on the fly)
* Software Multisampling enabled
* Auto zero offset if you are sure no current is flowing, or manual offset compensation if you know the values
* Gives the accuracy with the current scale. For reference, with a 20 mOhm shunt, the best accuracy is ~ 0.5 mA. 
With a 100 mOhm shunt, it's 2.5 mA in theory....

The time taken to do a sampling can vary depending on multisampling and auto range.
The order is 1ms * Nb of samples, can be worse if we change scale multiple time. 

NB : 
* The main thing we lose is the averaging done by the INA219 itself over several samples.
* As far as bus voltage is concerned, the voltage drop across the shunt itself is ignored,  you have to compensate for it externally.
It is on purpose, in case you have other wiring restistance to compensate for, you do it once globally.
