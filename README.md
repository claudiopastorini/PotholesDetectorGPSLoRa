# PotholesDetectorGPSLoRa

This repository contains all information regarding the code of [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) and [Adafruit Feather M0 Radio with LoRa Radio Module](https://www.adafruit.com/product/3178) for the [Pothole Detector project](https://github.com/onaralili/pothole-detector-project).

These board is part of a bigger system. The [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) will be connected to the [NUCLEO-F401RE](http://www.st.com/en/ecosystems/x-nucleo-iks01a2.html) board via I²C and it will send data through LoRa to the [Adafruit Feather M0 Radio with LoRa Radio Module](https://www.adafruit.com/product/3178) that is attached to a capable server with the [node-serialport-reader](https://github.com/onaralili/node-serialport-reader).

The code sends via LoRa information regarding the position, date/time and the number of potholes detected by the NUCLEO board.

## Instructions

### 1. Board connections

<center>

<img src="https://raw.githubusercontent.com/Mickyleitor/STM32F401RE-PotholeDetector/master/Docs/Board-connections.png" width="692">

| Seeeduino LoRaWAN w/ GPS | STM32F401RE (X-NUCLEO-IKS01A2) |
|           :---:          |          :---:                 |
|         PC5 (SCL)        |     PA8 (I2C3 SCL)             |
|         PC4 (SDA)        |     PC9  (I2C3 SDA)            |
|            +5V           |           +5V                  |
|            GND           |           GND                  |
  
</center>

### 2. Download the board definition
Using the Android IDE you have to add the `Additional Boards Manager URLs` for the Seeduino and Adafruit boards into the `Preferences` section: 

* https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
* https://raw.githubusercontent.com/Seeed-Studio/Seeed_Platform/master/package_seeeduino_boards_index.json

Now you can download the board definition via the `Boards Manager`:

* `Seeduino SAMD` for the Seeduino board
* `Adafruit SAMD` for the Adafruit board

### 2. Upload the code inside the boards
Upload the code to the devices (the same code works with both boards, it autodetects during compiling phase the right code to upload)

### 3. Testing board
You will see on the serial monitor of the Seeduino all the messages received via **I²C**, the collected position with date and time via **GPS** and the message to send over the **LoRa**.

### 4. Demo Mode
Because it is very difficult to take **GPS** position inside, there is a `demo mode` that uses constant positions in order to simulate a route.

Otherwise there is a `fake mode` that generates random positions in Rome around our departement, [DIAG](https://www.dis.uniroma1.it/en), you have only to change the value of the variable `IS_GPS_DATA_FAKE` inside the code.

## Credits

Claudio Pastorini: [LinkedIn](https://www.linkedin.com/in/claudio-pastorini/) | [WebSite](https://claudiopastorini.github.io) | [GitHub](https://github.com/claudiopastorini)
