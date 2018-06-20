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

### 3. Upload the code inside the boards
Upload the code to the devices (the same code works with both boards, it autodetects during compiling phase the right code to upload)

### 4. Testing board
You will see on the serial monitor of the Seeduino all the messages received via **I²C**, the collected position with date and time via **GPS** and the message to send over the **LoRa**.

### 5. Demo Mode
Because it is very difficult to take **GPS** position inside, there is a `demo mode` that uses constant positions in order to simulate a route.

Otherwise there is a `fake mode` that generates random positions in Rome around our departement, [DIAG](https://www.dis.uniroma1.it/en), you have only to change the value of the variable `IS_GPS_DATA_FAKE` inside the code.

## How it works

### I²C

The [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) and the [NUCLEO-F401RE](http://www.st.com/en/ecosystems/x-nucleo-iks01a2.html) are attached togher via I²C.

The [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) board is set as a slave to the address `4` of the I²C bus. 
Every time the [NUCLEO-F401RE](http://www.st.com/en/ecosystems/x-nucleo-iks01a2.html) founds one (or with the `Timed mode` more then one) pothole, it sends the number of detection to the [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html).

So for example for the following number of potholes detected in the `Burst mode`:

| Number of potholes |
|:------------------:|
|          2         |

The [NUCLEO-F401RE](http://www.st.com/en/ecosystems/x-nucleo-iks01a2.html) will simply send:

`2`

The [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) board, using an interrupt routine attached to the I²C bus, retrieves this number and, using its **GPS** capability, it fetches the position and the time of the occurred event.
With all this information it creates a message that sends through the **LoRa** radio.

### LoRa communication

The [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) and the [Adafruit Feather M0 Radio with LoRa Radio Module](https://www.adafruit.com/product/3178) are linked by a **P2P LoRa communication link**. In order to chat each other the two boards have to use the same configuration.

For our project we use the following configuration:

| Frequency (MHz) | Spreading Factor | Spreading Factor (chips/symbol) | Bandwidth (kHz) | Coding Rate | TX power |
|:---------------:|:----------------:|:-------------------------------:|:---------------:|:-----------:|:--------:|
|       433       |         6        |               128               |       125       |     4/5     |    20    |

> **N.B** The use of the `433 MHz` depends on the state where you reside. In Italy (and in general in Europe) the free bands for use **LoRa** are `433 MHz` and `868MHz`.

In this way the two boards can listen each other but, in order to communicate, they must follow the same message protocol.

For our project we use a simple `CSV` format, and so, every value is divide by a `,` (comma).

`latitude,longitude,date,time,number_of_potholes_detected`

So for example for the following data:

|   Latitude  |  Longitude |    Date    |   Time   | Number of potholes |
|:-----------:|:----------:|:----------:|:--------:|:------------------:|
|  41.891004  |  12.50143  | 20/06/2018 | 20:15.10 |          2         |

The [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) would produce the message:

`41.891004,12.50143,20062018,201510,2`

When the [Adafruit Feather M0 Radio with LoRa Radio Module](https://www.adafruit.com/product/3178) receives this message, it will immediatelly send over **Serial** in order to be parsed.

The [Adafruit Feather M0 Radio with LoRa Radio Module](https://www.adafruit.com/product/3178) uses the [RadioHead Driver Library](http://www.airspayce.com/mikem/arduino/RadioHead/), the [Seeduino LoRaWAN](https://www.seeedstudio.com/Seeeduino-LoRaWAN-p-2780.html) instead use **AT commands over serial communication**.

For further practical information about the **LoRa** protocol see the [PingPong example](https://github.com/claudiopastorini/PingPong) in particular way the [exaustive presentation](https://www.slideshare.net/ClaudioPastorini/adafruit-feather-m0-with-lora-radio-with-handson-example) about this technology using the [Adafruit Feather M0 Radio with LoRa Radio Module](https://www.adafruit.com/product/3178) board.

## Credits

Claudio Pastorini: [LinkedIn](https://www.linkedin.com/in/claudio-pastorini/) | [WebSite](https://claudiopastorini.github.io) | [GitHub](https://github.com/claudiopastorini)
