#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  #include <LoRaWan.h>
  #include <TinyGPS++.h>
  #include <Wire.h>
  
  #include <time.h>
  #include <stdlib.h>
  
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
#elif defined(ARDUINO_SAMD_FEATHER_M0)
  #include <SPI.h>
  #include <RH_RF95.h>

  // Chip select pin
  #define RFM95_CS 8
  // Reset pin
  #define RFM95_RST 4
  // Interrupt pin
  #define RFM95_INT 3
  
  // Must match the frequency of others nodes
  #define RF95_FREQ 433

  // One second in milliseconds
  #define MILLIS 1000
  
  // Singleton instance of the radio driver
  RH_RF95 rf95(RFM95_CS, RFM95_INT);
#endif

#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  unsigned char RXBuffer[RH_RF95_MAX_MESSAGE_LEN];
  
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
  
  // Object that contains last GPS data collected
  TinyGPSPlus gps;
  
  // I2C potholes_detected
  int potholes_detected;
  // I2C flag for message complete
  volatile bool IS_MESSAGE_COMPLETE = false;
#else
  char RXBuffer[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(RXBuffer);
  unsigned char TXBuffer[RH_RF95_MAX_MESSAGE_LEN];
#endif
  
// Counter of sent messages
uint8_t sentCounter = 0;
// Counter of received messages
uint8_t receivedCounter = 0;

//TODO creates fake data to feed the device for demo
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n"; 

void setup() {
#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  Serial.begin(9600);
  SERIAL.begin(115200);
  lora.init();
  // Inits the Seeduino LoRaWAN board passing frequency, spreadingFactor (125 chips/symbol), bandwidth, txPreamble, rxPreamble, power
  lora.initP2PMode(433, SF7, BW125, 8, 8, 20);

  delay(500);
  SERIAL.println("Seeduino LoRa radio init OK!");

  // Turns on I2C in slave mode
  Wire.begin(4);
  Wire.onReceive(receive);

  delay(500);
  SERIAL.println("Seeduino LoRa I2C init OK!");

  srand(time(NULL));
#elif defined(ARDUINO_SAMD_FEATHER_M0)
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  SERIAL.begin(115200);
  while (!SERIAL) {
    delay(1);
  }

  delay(100);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    SERIAL.println("Feather LoRa radio init failed");
    while (1);
  }
  SERIAL.println("Feather LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    SERIAL.println("setFrequency failed");
    while (1);
  }
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
#else
 SERIAL.println("NOT A VALID BOARD");
#endif
}

void loop() {

#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  // Checks if the message arrived from I2C
  if (IS_MESSAGE_COMPLETE) {
    // Prints the message
    SERIAL.println("Into the loop");
    SERIAL.println();
    
    // Callback function for getGPSData function
    void (*callback)(TinyGPSPlus *) = &generateAndSendPotholeInfo;
    // Collects GPS data
    getGPSData(callback);

    // Gets ready for an other interrupt
    IS_MESSAGE_COMPLETE = false;
  }
  
  delay(1500);
#elif defined(ARDUINO_SAMD_FEATHER_M0)
  char *message = NULL;
  short rssi = 0;
  
  // Receives a message
  if (!receiveMessage(&message, &rssi, 2)) {
    return;
  }
  
  // Prints out the received message
  //printReceivedMessage(message, rssi);
  SERIAL.println(message);
#endif
}


#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
// I2C interrupt routine
void receive(int howMany) {
  SERIAL.println("Into I2C receive callback");
  
  while (Wire.available() > 0) { 
    potholes_detected = Wire.read();
    SERIAL.print("Pothoes detected: ");
    SERIAL.println(potholes_detected);
    SERIAL.println();
  }
  
  // Sets to complete
  IS_MESSAGE_COMPLETE = true;
}

void getGPSData(void (*callback)(TinyGPSPlus *)) {
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
        SERIAL.println("No data");      
      }
    }
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

void generateAndSendPotholeInfo(TinyGPSPlus *gps_p) {
  TinyGPSPlus gps = *gps_p;
  String message;

  // Adds the position from the GPS   
  if (gps.location.isValid()) {
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);
  } else {
    message += "-91,-181";
  }

  // Adds the date from the GPS
  message += ",";
  if (gps.date.isValid()) {
    message += gps.date.value();
  } else {
    message += "0";
  }

  // Adds the time from the GPS
  message += ",";
  if (gps.time.isValid()) {
    message += gps.time.value();
  } else {
    message += "0";
  }

  message += ",";
  // Adds the number of the potholes detected
  message += potholes_detected;

  SERIAL.print("Message to send: ");
  SERIAL.println(message);
  
  // Declare a buffer
  char buf[100];

  // Copy this string into it
  snprintf(buf, sizeof(buf)-1, "%s", message.c_str());
  
  // Ensure we're terminated
  buf[sizeof(buf)] = '\0';

  // Callback function for sendMessage function
  void (*callback)(char *) = &printTransmittedMessage;
  // Sends message
  sendMessage(buf, callback);
}
#endif

void sendMessage(char *message, void (*callback)(char *)) {
#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  // Clears sender buffer for outcoming message
  memset(TXBuffer + RH_RF95_HEADER_LEN, 0, RH_RF95_MAX_MESSAGE_LEN);
  
  // Adds message into the buffer
  memcpy(TXBuffer + RH_RF95_HEADER_LEN, message, RH_RF95_MAX_MESSAGE_LEN);
  
  lora.transferPacketP2PMode(TXBuffer, RH_RF95_HEADER_LEN + strlen(message) + 1);
#elif defined(ARDUINO_SAMD_FEATHER_M0)
  // Clears sender buffer for outcoming message
  memset(TXBuffer, 0, RH_RF95_MAX_MESSAGE_LEN);

  // Adds message into the buffer
  memcpy(TXBuffer, message, RH_RF95_MAX_MESSAGE_LEN);
  
  // Sends
  rf95.send((uint8_t *)TXBuffer, strlen(message));

  // Waits for complete transmission
  delay(10);
  rf95.waitPacketSent(); 
#else
  SERIAL.println("NOT A VALID BOARD");
#endif

  if (callback != NULL) {
    callback(message);
  }

  // Increase counter of sent messages
  sentCounter++;
}

bool receiveMessage(char **message, short *rssi, uint8_t seconds) {
#if defined(ARDUINO_ARCH_SEEEDUINO_SAMD)
  // Clears receive buffer for incoming message
  memset(RXBuffer, 0, RH_RF95_FIFO_SIZE);
  
  // Receives
  if (lora.receivePacketP2PMode(RXBuffer, RH_RF95_FIFO_SIZE,  rssi, seconds) <= 0) {
    SERIAL.println("No message, is there a transmitter around?");
    return false;
  }

  // Gets only the message payload to print without the header
  *message = (char *) (RXBuffer + 4);
#elif defined(ARDUINO_SAMD_FEATHER_M0)
  // Waits for a message
  if (!rf95.waitAvailableTimeout(1 * MILLIS)) { 
    //SERIAL.println("No message, is there a transmitter around?");
    return false;
  }
  
  // Should be a reply message for us now   
  if (!rf95.recv((uint8_t *) RXBuffer, &len)) {  
    SERIAL.println("Receive failed");
    return false;
  }

  *message = (char *) RXBuffer;
  *rssi = rf95.lastRssi();
#else
  SERIAL.println("NOT A VALID BOARD");
#endif
  
  // Increase counter of received messages
  receivedCounter++;
  
  return true;
}

void printTransmittedMessage(char *message) {
  SERIAL.print("\t\t\t\t\t\t\t\t\t>>>>>>>>>>>>>>>>>>>>>>>> "); SERIAL.print(sentCounter); SERIAL.println(" >>>>>>>>>>>>>>>>>>>>>>>>");
  SERIAL.print("\t\t\t\t\t\t\t\t\tSending: "); 
  SERIAL.println(message);
  SERIAL.println("\t\t\t\t\t\t\t\t\tSending...");
  SERIAL.print("\t\t\t\t\t\t\t\t\t>>>>>>>>>>>>>>>>>>>>>>>> "); SERIAL.print(sentCounter); SERIAL.println(" >>>>>>>>>>>>>>>>>>>>>>>>");
  SERIAL.println();
}

void printReceivedMessage(char *message, int8_t rssi) {
  SERIAL.println();
  SERIAL.print(">>>>>>>>>>>>>>>>>>>>>>>> ");
  SERIAL.print(receivedCounter);
  SERIAL.println(" >>>>>>>>>>>>>>>>>>>>>>>>");
  SERIAL.print("Got: ");
  SERIAL.println(message);
  SERIAL.print("Lenght: ");
  SERIAL.println(strlen(message));
  SERIAL.print("RSSI: ");
  SERIAL.println(rssi, DEC);
  SERIAL.print(">>>>>>>>>>>>>>>>>>>>>>>> ");
  SERIAL.print(receivedCounter);
  SERIAL.println(" >>>>>>>>>>>>>>>>>>>>>>>>");
  SERIAL.println();
}
