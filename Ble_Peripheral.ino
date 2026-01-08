#include <bluefruit.h>

#define LED 25

//Ultrasonic
#define TRIG 28
#define ECHO 29

// BLE UART instance
BLEUart bleuart;

volatile int i = 0;

//volatile uint8_t cmdState = 0; // 1 => ON, 2 => OFF

// ---------- minimal rx callback ----------
// void rx_callback(uint16_t conn_handle) {
//   while (bleuart.available()) {
//     int c = bleuart.read();

//     if (c == '1') { cmdState = 1; }   // start measuring
//     else if (c == '0') { cmdState = 2; } // stop measuring
//     // optional: echo to Serial for debug
//     Serial.println("Command Recieved");
//     Serial.print("Switch: ");
//     Serial.write((char)c);
//     Serial.println();
//   }
//   Serial.println();
// }


volatile uint8_t cmdState = 0; // 1=ON, 2=OFF

void rx_callback(uint16_t conn_handle) {
  while (bleuart.available()) {
    int c = bleuart.read();
    if (c <= 0) continue;

    // Ignore common line terminators
    if (c == '\r' || c == '\n') continue;

    // Only handle '1' or '0'
    if (c == '1') {
      cmdState = 1;
      Serial.println("Command Received: 1 (ON)");
      Serial.println(); // keep a blank line like before
    } else if (c == '0') {
      cmdState = 2;
      Serial.println("Command Received: 0 (OFF)");
      Serial.println();
    } else {
      // optional: show unexpected bytes
      Serial.print("Ignored byte: 0x");
      Serial.println(c, HEX);
    }
  }
}

//=============================BLE Function=====================================================
void ble(){

  Serial.println("Starting BLE peripheral...");

  // Initialize Bluefruit
  Bluefruit.begin();
  Bluefruit.setName("NRF_Hello");              // BLE device name
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addName();             //Adds name to the device when being discovered by other devices
  Bluefruit.Advertising.addTxPower();          //Adds the function to determine the signal strength to determine the distance of this peripheral device
  
  bleuart.begin();
  bleuart.setRxCallback(rx_callback);

  Bluefruit.Advertising.addService(bleuart);   // add UART service
  Bluefruit.Advertising.start(0);              // advertise forever

  // Initialize UART service
  
  Serial.println("BLE UART started. Waiting for connections...");

}

//========================Ultrasonic Function==============================
void ultrasonic() {
  // Trigger pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read echo pulse
  long duration = pulseIn(ECHO, HIGH, 60000); // 60ms timeout

  // Convert to cm
  float distance = duration * 0.0343 / 2;

  i = distance;
  // Serial.print("Distance: ");
  // Serial.print(i);
  // Serial.println(" cm");

  delay(200);
}
//=============================================================================


void setup() {
  //Serial.begin(115200);
  //pinMode(IR_PIN, INPUT);

  Serial.begin(115200);
  while (!Serial);
  ble();                          //BLE setup
  //===================Ultrasonic Setup=======================
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  //==========================================================
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

}

void loop() {
  static bool measuring = false;
  char buf[32];

  // consume single-shot command
  if (cmdState == 1){
    measuring = true; 
    //digitalWrite(LED_PIN, HIGH); 
    bleuart.println("ACK:1"); 
    bleuart.println("Ultrasonic is on");
    delay(1000);
    bleuart.println("Touch again to stop");
    delay(1000);
    digitalWrite(LED, HIGH);
    cmdState = 0; 
    delay(200);
  }
  else if (cmdState == 2) { 
    measuring = false; 
    //digitalWrite(LED_PIN, LOW); 
    bleuart.println("ACK:0"); 
    bleuart.println("Ultrasonic is off");
    delay(1000);
    bleuart.println("Touch again to start");  
    delay(1000);
    digitalWrite(LED, LOW);
    cmdState = 0; 
    delay(200);
  }

  if(Bluefruit.connected()){
    if (measuring) {
      ultrasonic();                       // your function that sets 'i'
      snprintf(buf, sizeof(buf), "Ultrasonic=%dcm", i);
      bleuart.println(buf);
      Serial.println(buf);
      delay(200);
    } else {
      Serial.println("Ultrasonic = off");
      //bleuart.println("Ultrasonic is turned off");
      delay(200); // idle
    }
    
  }
  
}