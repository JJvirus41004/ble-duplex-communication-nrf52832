/* Central that connects reliably to peripheral named "NRF_Hello"
   - Active scan enabled (so scan-response is requested)
   - Connects by name first, falls back to checking for UART service UUID
   - Debug prints to Serial
*/

#include <bluefruit.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

BLEClientUart clientUart;   // client helper for UART service
BLEClientDis  clientDis;    // optional: device info client (keeps example compatible)

//==============for LCD=============================
LiquidCrystal_I2C lcd(0x27, 16, 2);
//==============for LCD=============================

// Add near other globals
bool hadConnectedOnce = false;   // becomes true after first successful connect

//=============================================
static bool reconnectShown = false;

//=========for touch================================
#define TOUCH_PIN 15

//bool touched
bool lastTouch = false;  // remembers previous touch state
bool ledState = false;   // this is our ON/OFF switch
//=========for touch================================

// void bleuart_rx_callback(BLEClientUart &uart_svc) {
//   // Called when peripheral sends data (TX -> notify)
//   while (uart_svc.available()) {
//     Serial.print((char)uart_svc.read());
    
//     lcd.clear();

//     lcd.setCursor(0, 0);
//     lcd.print((char)uart_svc.read());


//   }
//   Serial.println();
// }

// // Recommended: accumulate until newline, then update LCD
// void bleuart_rx_callback(BLEClientUart &uart_svc) {
//   static char lineBuf[64];
//   static uint8_t idx = 0;

//   while (uart_svc.available()) {
//     char ch = (char)uart_svc.read();   // read once
//     Serial.print(ch);

//     if (ch != '\r' && ch != '\n') {
//       if (idx < (sizeof(lineBuf) - 1)) {
//         lineBuf[idx++] = ch;
//         lineBuf[idx] = '\0';
//       }
//     } else {
//       if (idx > 0) {
//         // show up to 16 chars on first row
//         lcd.clear();
//         lcd.setCursor(0, 0);
//         for (uint8_t i = 0; i < 16 && lineBuf[i] != '\0'; i++) {
//           lcd.print(lineBuf[i]);
//         }
//         // reset buffer for next line
//         idx = 0;
//         lineBuf[0] = '\0';
//       }
//     }
//   }
//   Serial.println();
// }


// CONFIG
#define LINEBUF_SIZE 64
#define FLUSH_MS 60     // idle timeout to treat buffer as complete
#define LCD_COLS 16
#define LCD_ROWS 2
#define MAX_DISPLAY (LCD_COLS * LCD_ROWS)  // 32 for 16x2

// call this in your BLE RX callback
void bleuart_rx_callback(BLEClientUart &uart_svc) {
  static char lineBuf[LINEBUF_SIZE];
  static uint8_t idx = 0;
  static unsigned long lastRecv = 0;

  // Read all available bytes
  while (uart_svc.available()) {
    char ch = (char)uart_svc.read();
    Serial.print(ch);               // keep serial unchanged
    lastRecv = millis();

    if (ch == '\r') continue;       // ignore CR
    if (ch == '\n') {
      // explicit newline -> flush now
      if (idx > 0) {
        lineBuf[idx] = '\0';
        showTwoLineLCD(lineBuf);
        idx = 0;
      }
      continue;
    }

    // append char if room
    if (idx < (LINEBUF_SIZE - 1)) {
      lineBuf[idx++] = ch;
      lineBuf[idx] = '\0';
    } else {
      // buffer full: flush what we have then start new
      lineBuf[idx] = '\0';
      showTwoLineLCD(lineBuf);
      idx = 0;
    }
  }

  // idle flush: if we have data and no new byte for FLUSH_MS, flush it
  if (idx > 0 && (millis() - lastRecv) > FLUSH_MS) {
    lineBuf[idx] = '\0';
    showTwoLineLCD(lineBuf);
    idx = 0;
  }

  Serial.println();
}

// Helper: print first 16 chars to row0 and next up to 16 chars to row1.
// If total <= 16 then row1 is cleared (keeps it unused).
void showTwoLineLCD(const char *s) {
  size_t len = strlen(s);

  char row0[17] = {0}; // 16 chars + null
  char row1[17] = {0};

  // Copy first 16 chars safely
  size_t n0 = len < 16 ? len : 16;
  memcpy(row0, s, n0);
  row0[n0] = '\0';

  // Copy next 16 chars safely if they exist
  if (len > 16) {
    size_t n1 = len - 16;
    if (n1 > 16) n1 = 16;
    memcpy(row1, s + 16, n1);
    row1[n1] = '\0';
  }

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(row0);

  lcd.setCursor(0, 1);
  if (row1[0] != '\0') {
    lcd.print(row1);
  } else {
    // Keep 2nd row unused if text ≤ 16
    lcd.print("                "); // 16 spaces
  }
}



// Helper: extract local name from advertisement (returns name length, fills buf)
int getAdvName(const ble_gap_evt_adv_report_t* report, char* buf, int bufsize) {
  if (!report || !report->data.p_data || report->data.len <= 0) {
    if (buf && bufsize>0) buf[0]=0;
    return 0;
  }

  const uint8_t* data = report->data.p_data;
  int len = report->data.len;
  int idx = 0;

  while (idx < len) {
    uint8_t field_len = data[idx];            // length of this AD structure (doesn't include this length byte)
    if (field_len == 0) break;
    if (idx + field_len >= len) break;        // malformed guard

    uint8_t ad_type = data[idx + 1];          // AD type
    if (ad_type == 0x09 || ad_type == 0x08) { // Complete Local Name or Shortened Local Name
      int name_len = field_len - 1;           // minus the ad_type byte
      if (name_len > bufsize - 1) name_len = bufsize - 1;
      memcpy(buf, &data[idx + 2], name_len);
      buf[name_len] = '\0';
      return name_len;
    }

    // move to next AD structure: field_len + 1 (length byte itself)
    idx += field_len + 1;
  }

  if (buf && bufsize>0) buf[0] = '\0';
  return 0;
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Debug: show RSSI, type (safe), and data len
  Serial.print("Adv packet: rssi=");
  Serial.print(report->rssi);

  // Safely extract the first byte of report->type regardless of its actual type
  uint8_t adv_type_byte = 0;
  memcpy(&adv_type_byte, &report->type, sizeof(adv_type_byte));
  Serial.print(" type=");
  Serial.print(adv_type_byte);

  Serial.print(" len=");
  Serial.println((int)report->data.len);

  // (rest of your code unchanged...)
  char advName[32];
  int nameLen = getAdvName(report, advName, sizeof(advName));
  if (nameLen > 0) {
    Serial.print("  Name found: ");
    Serial.println(advName);
    if (strcmp(advName, "NRF_Hello") == 0) {
      Serial.println(" -> Name matches NRF_Hello. Connecting...");
      Bluefruit.Central.connect(report);
      return;
    }
  } else {
    Serial.println("  No name in advert/scan-response");
  }

  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) ) {
    Serial.println(" -> UART service UUID found. Connecting...");
    Bluefruit.Central.connect(report);
    return;
  }

  Serial.println(" -> no match");

    // Show "Reconnecting" on LCD only *after* we've connected once,
  // and only if we're currently disconnected — avoid spamming the LCD.
  //static bool reconnectShown = false;

  if (! Bluefruit.Central.connected() && hadConnectedOnce) {
    if (!reconnectShown) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reconnecting");
      lcd.setCursor(0, 1);
      lcd.print("To BLE....");
      reconnectShown = true;
    }
  // } else {
  //   // either we're connected, or we've never connected before (initial boot).
  //   // clear the flag so reconnect message can show next time a disconnect happens.
  //   reconnectShown = false;
  }


  Bluefruit.Scanner.resume();
}


void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected. Discovering services...");
  hadConnectedOnce = true;        //1st Connected


  // discover Device Info (optional)
  if ( clientDis.discover(conn_handle) ) {
    Serial.println("Device Info found");
  }

  // Discover UART service on remote and enable notifications for TXD
  if ( clientUart.discover(conn_handle) ) {
    Serial.println("BLE UART service discovered.");
    clientUart.setRxCallback(bleuart_rx_callback); // set callback BEFORE enabling TXD
    clientUart.enableTXD();                        // enable peripheral -> central notifications
    Serial.println("Notifications enabled. Ready.");
    Serial.println("To Start Ultrasonic: Press Touch Sensor");
    //===============For LCD============================================   
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Ble Connected");
    reconnectShown = false;
    //===============Improvement================================
    delay(500);

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Press to ");

    lcd.setCursor(0, 1);
    lcd.print("Start/Stop");
    // lcd.setCursor(0, 1);
    // lcd.print("Press again");
    //===============For LCD============================================

  } else {
    Serial.println("BLE UART NOT found. Disconnecting.");
    Bluefruit.disconnect(conn_handle);
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle; (void) reason;
  Serial.println("Disconnected. Restarting scan.");
  Bluefruit.Scanner.start(0); // restart scanning
}

//============for touch============================
bool readStable() {
  int countHigh = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(TOUCH_PIN) == HIGH) countHigh++;
    delay(5);
  }
  return (countHigh >= 3);
}

// void touch(){

//   touched = readStable();

//   // Toggle only when touch becomes NEW (not while holding)
//   if (touched && !lastTouch) {
//     ledState = !ledState;  // flip ON <-> OFF
//     Serial.println(ledState ? "1" : "0");
//   }

//   lastTouch = touched; // remember last touch
//   delay(50);
// }
//============for touch============================



void setup() {
  Serial.begin(115200);
//=============================For LCD================================================
  delay(100);

  Wire.setPins(6, 7);  // ✅ Set SDA = 6, SCL = 7 for nRF52
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Connecting");

  lcd.setCursor(0, 1);
  lcd.print("To BLE....");
 
  Serial.println("LCD ready!");

  // lcd.setCursor(0, 1);
  // lcd.print("nRF52 I2C Ready");

//=============================For LCD================================================

  while (!Serial) ; // wait for Serial on native-USB boards

  Serial.println("Bluefruit Central - scanning for BLE UART peripherals (name OR UUID)");

  // Central only: Peripheral = 0, Central = 1
  Bluefruit.begin(0, 1);
  Bluefruit.setName("Bluefruit_Central");

  // init client helpers
  clientDis.begin();
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback); // receives incoming UART bytes

  // register central callbacks
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  // scanner setup
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // recommended values
  Bluefruit.Scanner.useActiveScan(true);   // <-- IMPORTANT: request scan-response (name/UUID in scan response)
  Bluefruit.Scanner.start(0); // scan forever
//================for touch======================================
  pinMode(TOUCH_PIN, INPUT_PULLDOWN);
  
  //Serial.println("To Stop Ultrasonic: Press 0");

//================for touch======================================

}

// Minimal robust central loop: send single chars from Serial to peripheral UART
// Assumes `clientUart` is the BLE UART client object created during discovery
// and that clientUart.discovered() is true when the UART service/char is ready.

void loop() {
  bool touched = readStable();

  // Toggle only when touch becomes NEW (not while holding)
  if (touched && !lastTouch) {
    ledState = !ledState;            // flip ON <-> OFF
    clientUart.print(ledState ? "1\r\n" : "0\r\n");
  }
  lastTouch = touched; // remember last touch

  // --- send to peripheral if connected and UART service discovered ---
  if (Bluefruit.Central.connected() && clientUart.discovered()) {
    // send single bytes immediately (no blocking readBytes)
    while (Serial.available()) {
      char c = (char)Serial.read();      // grab one char from USB serial
      // only forward '0' or '1' (guard against stray chars)
      if (c == '0' || c == '1') {
        // use write(buffer, len) so we send raw bytes exactly
        clientUart.write(&c, c);
        Serial.print("Sent to peripheral: ");
        Serial.println(c);
      } else {
        // optionally ignore CR/LF or echo them
      }
      delay(2); // tiny spacing so BLE stack has time (optional)
    }
  }

  delay(10); // keep loop cooperative
}
