simpler_INA219
===============

This is a INA219 sensor driver for arduino, derived from the ADAFruit one.
It is quite different though :

* Adjustable shunt resistor
* Auto zero
* Auto scale change (it will pick the best scale automatically, on the fly)
* Power and current computation by the chip removed, we do it in software
* Multisampling enabled
