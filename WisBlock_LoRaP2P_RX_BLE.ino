#include <Arduino.h>
#include <bluefruit.h>
#include <SX126x-RAK4630.h>
#include <SPI.h>
#include <U8g2lib.h> // https://github.com/olikraus/u8g2

// Function declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void hexDump(uint16_t len);

#ifdef NRF52_SERIES
#define LED_BUILTIN 35
#endif

// Define LoRa parameters
double RF_FREQUENCY = 868125000;	// Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 1 // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 10 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static uint8_t RcvBuffer[256];
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
BLEUart bleuart;
bool bleUARTisConnected = false;

void hexDump(uint16_t len) {
  // hexDump method specific to RcvBuffer
  String s = "|", t = "| |";
  Serial.println(F("\n  |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f |"));
  Serial.println(F("  +------------------------------------------------+ +----------------+"));
  for (uint16_t i = 0; i < len; i += 16) {
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j >= len) {
        s = s + "   "; t = t + " ";
      } else {
        char c = RcvBuffer[i + j];
        if (c < 16) s = s + "0";
        s = s + String(c, HEX) + " ";
        if (c < 32 || c > 127) t = t + ".";
        else t = t + (String(c));
      }
    }
    uint8_t index = i / 16;
    Serial.print(index, HEX); Serial.write('.');
    Serial.println(s + t + "|");
    s = "|"; t = "| |";
  }
  Serial.println(F("  +------------------------------------------------+ +----------------+"));
}

/**
   @brief  Callback when client connects
   @param  conn_handle: Connection handle id
*/
void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;
  bleUARTisConnected = true;
  Serial.println("BLE client connected");
}

/**
   @brief  Callback invoked when a connection is dropped
   @param  conn_handle: connection handle id
   @param  reason: disconnect reason
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  bleUARTisConnected = false;
  Serial.println("BLE client disconnected");
}

void setupBLE() {
  Serial.println("================================");
  Serial.println("     RAK4631 BLE UART setup");
  Serial.println("================================");
  u8g2.drawStr(3, 30, "BLE setup");
  u8g2.sendBuffer(); // transfer internal memory to the display
  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);
  Bluefruit.begin(1, 0);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  // Set the BLE device name
  Bluefruit.setName("RAK4631_LoRa_UART");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  // Configure and Start BLE Uart Service
  bleuart.begin();
  // Set up and start advertising
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval: fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)
     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30); // number of seconds in fast mode
  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds
}

void setupLoRa() {
  Serial.println("\n=====================================");
  Serial.println("        LoRaP2P Rx Setup");
  Serial.println("=====================================");
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
  u8g2.drawStr(3, 15, "LoRa setup");
  u8g2.sendBuffer(); // transfer internal memory to the display
  // Initialize the Radio callbacks
  RadioEvents.TxDone = NULL;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = NULL;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = NULL;
  // Initialize the Radio
  Radio.Init(&RadioEvents);
  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);
  // Set Radio RX configuration
  Radio.SetRxConfig(
    MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  // Start LoRa
  Serial.println("Starting Radio.Rx");
  Radio.Rx(RX_TIMEOUT_VALUE);
  SX126xSetTxParams(22, RADIO_RAMP_40_US);
}

/**@brief Function to be executed on Radio Rx Done event
*/
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memset(RcvBuffer, 0, 256);
  sprintf((char*)RcvBuffer, "Rssi=%d dBm", rssi);
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
  u8g2.drawStr(3, 15, (char*)RcvBuffer);
  memset(RcvBuffer, 0, 256);
  sprintf((char*)RcvBuffer, "Snr=%d", snr);
  u8g2.drawStr(3, 30, (char*)RcvBuffer);
  String s = String(RF_FREQUENCY / 1e6, 3) + " MHz";
  u8g2.drawStr(3, 45, s.c_str());
  u8g2.sendBuffer(); // transfer internal memory to the display
  sprintf((char*)RcvBuffer, "Rssi=%d dBm Snr=%d", rssi, snr);
  Serial.println((char*)RcvBuffer);
  bleuart.print((char*)RcvBuffer);

  memcpy(RcvBuffer, payload, size);
  hexDump(size);
  Serial.println("");
  Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Timeout event
*/
void OnRxTimeout(void) {
  //  u8g2.clearBuffer(); // clear the internal memory
  //  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
  //  u8g2.drawStr(3, 15, "OnRxTimeout");
  //  u8g2.sendBuffer(); // transfer internal memory to the display
  // Serial.println("OnRxTimeout");
  Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Error event
*/
void OnRxError(void) {
  Serial.println("OnRxError");
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void setup() {
  // Initialize built in green LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // Initialize LoRa chip.
  lora_rak4630_init();
  // Initialize Serial for debug output
  Serial.begin(115200);
  delay(3000);
  u8g2.begin();
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
  u8g2.sendBuffer(); // transfer internal memory to the display
  setupLoRa();
  setupBLE();
  u8g2.drawStr(3, 55, "Ready");
  u8g2.sendBuffer(); // transfer internal memory to the display
  Serial.println("READY");
}

void loop() {
  // Handle Radio events
  Radio.IrqProcess();
  // We are on FreeRTOS, give other tasks a chance to run
  delay(100);
  yield();
  // Forward anything received from USB Serial to BLE UART
  if (Serial.available() && bleUARTisConnected) {
    bleuart.print(Serial.readString());
  }
  // Forward anything received from BLE UART to USB Serial
  if (bleuart.available()) {
    String s = bleuart.readString();
    Serial.println(s);
    uint16_t ln = s.length() + 1;
    char cmd[ln];
    s.toCharArray(cmd, ln);
    char c0 = cmd[0], c1 = cmd[1];
    if (c0 == '/') {
      // commands start with /
      if (c1 == 'F') {
        // Frequency
        // /F868.125
        uint32_t fq = (uint32_t)(atof(cmd + 2) * 1e6);
        // 862 to 1020 MHz frequency coverage
        if (fq < 862e6 || fq > 1020e6) {
          Serial.println("Requested frequency (" + String(cmd + 2) + ") is invalid!");
        } else {
          RF_FREQUENCY = fq;
          Radio.SetChannel(RF_FREQUENCY);
          delay(100);
          Serial.println("Frequency set to " + String(RF_FREQUENCY / 1e6, 3) + " MHz");
          String s = String(RF_FREQUENCY / 1e6, 3) + " MHz";
          bleuart.print("Set to " + s);
          u8g2.clearBuffer(); // clear the internal memory
          u8g2.drawStr(3, 15, "Frequency:");
          u8g2.drawStr(3, 30, s.c_str());
          u8g2.sendBuffer();
        }
        return;
      }
      if (c1 == 'S') {
        // Send packet
        // I haven't installed AES & JSON yet, so we'll send a raw packet.
        uint8_t ln = strlen((char*)cmd) - 2;
        Serial.print("Sending ");
        Serial.print(cmd + 2);
        Serial.print("...");
        Radio.Send((uint8_t*)cmd + 2, ln);
        Serial.println(" done!");
        u8g2.drawStr(3, 45, "Sent...");
        u8g2.sendBuffer();
        bleuart.print("Message sent.");
        return;
      }
    }
  }
}
