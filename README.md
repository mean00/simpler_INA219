simpler_INA219
===============

This is a INA219 sensor driver for arduino, derived from the ADAFruit one.
It is quite different though.
Most of the complexity of the INA is to let it compute current and power.

They are just a multiplication, we can do it easily on the arduino side, with much more flexibility and at
practically zero cost (the original driver does multiplication anyway).
So we just keed the ADC part and adds some features :

* Adjustable shunt resistor in the constructor (default is 100 mOhm)
* Auto scale change (it will pick the best scale automatically, on the fly)
* Software Multisampling enabled
* Auto zero offset if you are sure no current is flowing, or manual offset compensation if you know the values
* No power consumption computation, and we do the current internally without the help of the ina
* Gives the accuracy with the current scale. For reference, with a 20 mOhm shunt, the best accuracy is ~ 0.5 mA. 
With a 100 mOhm shunt, it's 2.5 mA in theory....

The time taken to do a sampling can vary depending on multisampling and auto range.
The order is 1ms * Nb of samples, can be worse if we change scale multiple time. 
