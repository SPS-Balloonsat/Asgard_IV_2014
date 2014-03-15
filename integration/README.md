Odin Sensor payload - Code Integration
======================================

Asgard-IV HAB flight
--------------------


Written for Teensy 3.1 (ARM Cortex based board; Freescale MK200X256)

Sensors used:

* Invensense MPU-6050 motion sensor on SparkFun breakout board
* IST Ag. HYT-271 humidity sensor
* Freescale MPX4115A pressure sensor
* uBlox MAX-7Q GPS unit on HAB Supplies breakout board (w/ level convertors as initially intended for use with Arduino Pro Mini 5V)
* Custom gamma ray detector board; based on Maxim Semiconductor [application note] http://www.maximintegrated.com/app-notes/index.mvp/id/2236


The codenames used throughout are fairly arbitrary; the programs themselves are named (in what is becoming my custom) in the following fashion:
```

"Place Name" + "Fish Type"

```

For instance:
    "Arctic Char"
	
The title of the overall sensor payload - Odin - was chosen because the Balloon-sat mission name is "Asgard - n" and so Odin was felt to be appropriate!

