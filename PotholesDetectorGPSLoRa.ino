#include <LoRaWan.h>
#include <TinyGPS++.h>
#include <Wire.h>

#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RH_RF95_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_RF95_MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_RF95_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_RF95_MAX_MESSAGE_LEN
  #define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_MAX_PAYLOAD_LEN - RH_RF95_HEADER_LEN)
#endif

// All messages sent and received by this RH_RF95 Driver conform to this packet format:
//
// - LoRa mode:
// - 8 symbol PREAMBLE
// - Explicit header with header CRC (handled internally by the radio)
// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
// - 0 to 251 octets DATA 
// - CRC (handled internally by the radio)
// So we need to send 4 octect of header in order to allow the RF95 to understand the message
unsigned char TXBuffer[RH_RF95_MAX_MESSAGE_LEN] = {0xFF, 0xFF, 0x0, 0x0};

// Counter of sent messages
uint8_t sentCounter = 0;

// Object that contains last GPS data collected
TinyGPSPlus gps;

// I2C buffer
char buffer[100];
// I2C array position
volatile int position = 0;
// I2C flag for message complete
volatile bool IS_MESSAGE_COMPLETE = true;

//TODO creates fake data to feed the device for demo
char *gpsStreamArray[] =  {
  "$GPGGA,132508.101,4153.685,N,01230.144,E,1,12,1.0,0.0,M,0.0,M,,*6A\r\n"
  "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n"
  "$GPRMC,132508.101,A,4153.685,N,01230.144,E,,,160518,000.0,W*75\r\n"
};

int i = 0;

void setup() {
  Serial.begin(9600);
  SERIAL.begin(115200);
  lora.init();
  // Inits the Seeduino LoRaWAN board passing frequency, spreadingFactor (125 chips/symbol), bandwidth, txPreamble, rxPreamble, power
  lora.initP2PMode(433, SF7, BW125, 8, 8, 20);

  delay(500);
  SERIAL.println("Seeduino LoRa radio init OK!");

  // Turns on I2C in slave mode
  Wire.begin(8);
  Wire.onReceive(receive);

  delay(500);
  SERIAL.println("Seeduino LoRa I2C init OK!");
}

void loop() {

  // Checks if the message arrived from SPI
  if (IS_MESSAGE_COMPLETE) {
    // Prints the message
    SERIAL.println(buffer);

    char *message;
    
    // Callback function for getGPSData function
    void (*callback)(TinyGPSPlus *) = &generateAndSendPotholeInfo;
    // Collects GPS data
    getGPSData(callback);

    // Gets ready for an other interrupt
    position = 0;
    IS_MESSAGE_COMPLETE = false;
  }
}

// I2C interrupt routine
void receive(int howMany) {
  while (Wire.available() > 0) { 
    char c = Wire.read();
    SERIAL.print(c);

    buffer[position] = c;
    position++;
  }
  // Puts the terminator string
  buffer[position] = '\0';
  // Sets to complete
  IS_MESSAGE_COMPLETE = true;
}

void getGPSData(void (*callback)(TinyGPSPlus *)) {
  SERIAL.println("getGPSData");
  // Reads from the serial
  while (Serial.available() > 0) {
    
    // Feeds the TinyGPSPlus object with the reading
    if (gps.encode(Serial.read())) {  
      // If the location is valid and it is updated 
      if (gps.location.isValid() && gps.location.isUpdated()) {
        // If there is a callback
        if (callback != NULL) {
          // Call the callback
          (*callback)(&gps);
        // Otherwise  
        } else {
          // Print the GPS info
          printGPSInfo(&gps);
        }
      } else {
        SERIAL.println("No data, use the fake one");

        char *gpsStream = gpsStreamArray[i];
        while (*gpsStream) {
          if (gps.encode(*gpsStream++)) {
            // Print the GPS info
            printGPSInfo(&gps);  
          }
        }
        i++;
        if (i == (sizeof(gpsStreamArray) / sizeof(gpsStreamArray[0]))) {
          i = 0;
        }
        delay(100);
      }
    }
  }
}

void sendMessage(char *message, void (*callback)(char *)) {
  // Clears sender buffer for outcoming message
  memset(TXBuffer + RH_RF95_HEADER_LEN, 0, RH_RF95_MAX_MESSAGE_LEN);
  
  // Adds message into the buffer
  memcpy(TXBuffer + RH_RF95_HEADER_LEN, message, RH_RF95_MAX_MESSAGE_LEN);
  
  lora.transferPacketP2PMode(TXBuffer, RH_RF95_HEADER_LEN + strlen(message) + 1);

  // Increase counter of sent messages
  sentCounter++;

  if (callback != NULL) {
    callback(message);
  }
}

void printGPSInfo(TinyGPSPlus *gps_p) {
  TinyGPSPlus gps = *gps_p;
  
  SERIAL.print(F("Location: ")); 
  if (gps.location.isValid()) {
    SERIAL.print(gps.location.lat(), 6);
    SERIAL.print(F(","));
    SERIAL.print(gps.location.lng(), 6);
  } else {
    SERIAL.print(F("INVALID"));
  }

  SERIAL.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    SERIAL.print(gps.date.day());
    SERIAL.print(F("/"));
    SERIAL.print(gps.date.month());
    SERIAL.print(F("/"));
    SERIAL.print(gps.date.year());
  } else {
    SERIAL.print(F("INVALID"));
  }

  SERIAL.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) SERIAL.print(F("0"));
    SERIAL.print(gps.time.hour());
    SERIAL.print(F(":"));
    if (gps.time.minute() < 10) SERIAL.print(F("0"));
    SERIAL.print(gps.time.minute());
    SERIAL.print(F(":"));
    if (gps.time.second() < 10) SERIAL.print(F("0"));
    SERIAL.print(gps.time.second());
    SERIAL.print(F("."));
    if (gps.time.centisecond() < 10) SERIAL.print(F("0"));
    SERIAL.print(gps.time.centisecond());
  } else {
    SERIAL.print(F("INVALID"));
  }

  SERIAL.println();
}

void printTransmittedMessage(char *message) {
  SERIAL.print("\t\t\t\t\t\t\t\t\t>>>>>>>>>>>> "); SERIAL.print(sentCounter); SERIAL.println(" >>>>>>>>>>>>");
  SERIAL.print("\t\t\t\t\t\t\t\t\tSending: "); 
  SERIAL.println(message);
  SERIAL.println("\t\t\t\t\t\t\t\t\tSending...");
  SERIAL.print("\t\t\t\t\t\t\t\t\t>>>>>>>>>>>> "); SERIAL.print(sentCounter); SERIAL.println(" >>>>>>>>>>>>");
  SERIAL.println();
}

void generateAndSendPotholeInfo(TinyGPSPlus *gps_p) {
  TinyGPSPlus gps = *gps_p;
  char *message;
  
  if (gps.location.isValid()) {
    sprintf(message, "%06d,%06d", gps.location.lat(), gps.location.lng());
  } else {
    strcat(message, "-91,-181");
  }

  strcat(message, ",");
  if (gps.date.isValid()) {
    sprintf(message, "%02d", gps.date.value());
  } else {
    strcat(message, "0");
  }

  strcat(message, ",");
  if (gps.time.isValid()) {
    sprintf(message, "%02d", gps.time.value());
  } else {
    strcat(message, "0");
  }
  
  strcat(message, "\n");
  SERIAL.println(message);

  // Callback function for getGPSData function
  void (*callback)(char *) = &printTransmittedMessage;
  // Sends message
  sendMessage(message, callback);
}
