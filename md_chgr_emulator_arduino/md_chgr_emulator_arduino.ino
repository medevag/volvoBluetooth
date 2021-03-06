
/* Melbus MDCHGR Emulator

 * The HU enables the MD-CHGR in its source-menu after a successful initialization procedure is accomplished.
 * The HU will remove the MD-CHGR every time the car starts if it wont get an response from MD-CHGR (second init-procedure).
 *
 * Karl HagstrÃ¶m 2015-11-04
 * mod by S. Zeller 2016-03-14
 * mod by M.E 2017-02
 *
 * This project went realy smooth thanks to these sources:
 * http://volvo.wot.lv/wiki/doku.php?id=melbus
 * https://github.com/festlv/screen-control/blob/master/Screen_control/melbus.cpp
 * http://forums.swedespeed.com/showthread.php?50450-VW-Phatbox-to-Volvo-Transplant-(How-To)&highlight=phatbox
 *
 * pulse train width=120us (15us per clock cycle), high phase between two pulse trains is 540us-600us
 */

#define SERDBG        (1)
#define CURRENT_SONG  (5)
#define CURRENT_DISC  (3)

const uint8_t MELBUS_CLOCKBIT_INT = 1;       // Interrupt numer (INT1) on DDR3
const uint8_t MELBUS_CLOCKBIT = 3;           // Pin D3 - CLK
const uint8_t MELBUS_DATA = 4;               // Pin D4 - Data
const uint8_t MELBUS_BUSY = 5;               // Pin D5 - Busy
const uint8_t ACTIVATE_BLUETOOTH = 7;         // Pin D7 - Activate Raspberry bluetooth search
const uint8_t DROP_BLUETOOTH_CONNECTIONS = 8; // Pin D8 - Drop all active bluetooth connections

volatile uint8_t melbus_ReceivedByte = 0;
volatile uint8_t melbus_CharBytes = 0;
volatile uint8_t melbus_OutByte = 0xFF;
volatile uint8_t melbus_LastReadByte[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t melbus_SendBuffer[9] = {0x00,0x02,0x00,0x01,0x80,0x01,0xC7,0x0A,0x02};
volatile uint8_t melbus_SendCnt=0;
volatile uint8_t melbus_DiscBuffer[6] = {0x00,0xFC,0xFF,0x4A,0xFC,0xFF};
volatile uint8_t melbus_DiscCnt=0;
volatile uint8_t melbus_Bitposition = 0x80;

volatile bool InitialSequence_ext = false;
volatile bool ByteIsRead = false;
volatile bool sending_byte = false;
volatile bool melbus_MasterRequested = false;
volatile bool melbus_MasterRequestAccepted = false;

volatile bool testbool = false;
volatile bool AllowInterruptRead = true;
volatile int incomingByte = 0; // For incoming serial data

// Startup seequence
void setup() {
  // Data is deafult input high
  pinMode(MELBUS_DATA, INPUT_PULLUP);

  // Activate interrupt on clock pin (INT1, D3)
  attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, FALLING);
  //Set Clockpin-interrupt to input
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);

  // Initiate pins for Raspberry communiction
  pinMode(DROP_BLUETOOTH_CONNECTIONS, OUTPUT);
  digitalWrite(DROP_BLUETOOTH_CONNECTIONS, LOW);

  pinMode(ACTIVATE_BLUETOOTH, OUTPUT);
  digitalWrite(ACTIVATE_BLUETOOTH, LOW);

#ifdef SERDBG
  // Initiate serial communication to debug via serial-usb (arduino)
  Serial.begin(230400);
  Serial.println("Initiating contact with Melbus:");
#endif
  // Call function that tells HU that we want to register a new device
  melbus_Init_MDCHRG();
}

// Main loop
void loop() {
  // Waiting for the clock interrupt to trigger 8 times to read one byte before evaluating the data
#ifdef SERDBG
  if (ByteIsRead) {
    // Reset bool to enable reading of next byte
    ByteIsRead=false;

    if (incomingByte == ' ') {

      if (melbus_CharBytes) {
        Serial.write(melbus_LastReadByte[1]);
        melbus_CharBytes--;
      } else {
        Serial.print(melbus_LastReadByte[1],HEX);
        Serial.write(' ');
      }
    }
  }
#endif


  // If BUSYPIN is HIGH => HU is in between transmissions
  if (digitalRead(MELBUS_BUSY) == HIGH) {
    // Make sure we are in sync when reading the bits by resetting the clock reader

#ifdef SERDBG
    if (melbus_Bitposition != 0x80) {
      Serial.println(melbus_Bitposition,HEX);
      Serial.println("\n not in sync! ");
    }
#endif
    if (incomingByte != 'k') {
      melbus_Bitposition = 0x80;
      melbus_OutByte = 0xFF;
      melbus_SendCnt = 0;
      melbus_DiscCnt = 0;
      DDRD &= ~(1<<MELBUS_DATA);
      PORTD |= (1<<MELBUS_DATA);
    }

  }
#ifdef SERDBG
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();


  }
  if (incomingByte == 'i') {
    melbus_Init_MDCHRG();
    Serial.println("\n forced init: ");
    incomingByte=0;

  }
#endif
  if ((melbus_Bitposition == 0x80) && (PIND & (1<<MELBUS_CLOCKBIT))) {
    delayMicroseconds(7);
    DDRD &= ~(1<<MELBUS_DATA);
    PORTD |= (1<<MELBUS_DATA);
  }
}

// Notify HU that we want to trigger the first initiate procedure to add a new device (MD-CHGR) by pulling BUSY line low for 1s
void melbus_Init_MDCHRG() {
  //Disabel interrupt on INT1 quicker then: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1<<INT1);

  // Wait untill Busy-line goes high (not busy) before we pull BUSY low to request init
  while(digitalRead(MELBUS_BUSY)==LOW){}
  delayMicroseconds(10);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);
  delay(1200);
  digitalWrite(MELBUS_BUSY, HIGH);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);

  // Enable interrupt on INT1, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1<<INT1);
}

// Global external interrupt that triggers when clock pin goes high after it has been low for a short time => time to read datapin
void MELBUS_CLOCK_INTERRUPT() {

  // Read status of Datapin and set status of current bit in recv_byte

  if(melbus_OutByte & melbus_Bitposition) {
    DDRD &= (~(1<<MELBUS_DATA));
    PORTD |= (1<<MELBUS_DATA);
  }
  // If bit [i] is "0" - make databpin low
  else {
    PORTD &= (~(1<<MELBUS_DATA));
    DDRD |= (1<<MELBUS_DATA);
  }


  if (PIND & (1<<MELBUS_DATA)) {
    melbus_ReceivedByte |= melbus_Bitposition; // Set bit nr [melbus_Bitposition] to "1"
  } else {
    melbus_ReceivedByte &=~melbus_Bitposition; // Set bit nr [melbus_Bitposition] to "0"
  }


  // If all the bits in the byte are read:
  if (melbus_Bitposition == 0x01) {

    // Move every lastreadbyte one step down the array to keep track of former bytes
    for(int i=11; i > 0; i--){
      melbus_LastReadByte[i] = melbus_LastReadByte[i-1];
    }

    if (melbus_OutByte != 0xFF) {
      melbus_LastReadByte[0] = melbus_OutByte;
      melbus_OutByte = 0xFF;
    } else {
      // Insert the newly read byte into first position of array
      melbus_LastReadByte[0] = melbus_ReceivedByte;
    }
    // Set bool to true to evaluate the bytes in main loop
    ByteIsRead = true;

    // Reset bitcount to first bit in byte
    melbus_Bitposition = 0x80;

    if(melbus_LastReadByte[2] == 0x07 && (melbus_LastReadByte[1] == 0x1A || melbus_LastReadByte[1] == 0x4A) && melbus_LastReadByte[0] == 0xEE) {
      InitialSequence_ext = true;
    }

    else if(melbus_LastReadByte[2] == 0x0 && (melbus_LastReadByte[1] == 0x1C || melbus_LastReadByte[1] == 0x4C) && melbus_LastReadByte[0] == 0xED) {

      InitialSequence_ext = true;
    }

    else if((melbus_LastReadByte[0] == 0xD8 || melbus_LastReadByte[0] == 0xD9) && InitialSequence_ext == true) {
      InitialSequence_ext = false;

      // Returning the expected byte to the HU, to confirm that the MD-CHGR is present (0xEE)! see "ID Response"-table here http://volvo.wot.lv/wiki/doku.php?id=melbus
      melbus_OutByte = 0xDE;
    }

    else if((melbus_LastReadByte[2] == 0xD8 || melbus_LastReadByte[2] == 0xD9) && (melbus_LastReadByte[1] == 0x1E || melbus_LastReadByte[1] == 0x4E) && melbus_LastReadByte[0] == 0xEF) {
      // CartInfo
      melbus_DiscCnt=6;
    }

    else if((melbus_LastReadByte[2] == 0xD8 || melbus_LastReadByte[2] == 0xD9) && (melbus_LastReadByte[1] == 0x19 || melbus_LastReadByte[1] == 0x49) && melbus_LastReadByte[0] == 0x22) {
      // Powerdown
      Serial.println("Power down");
      melbus_OutByte = 0x00; // Respond to powerdown;
      melbus_SendBuffer[1] = 0x02; // STOP
      melbus_SendBuffer[8] = 0x02; // STOP
    }

    else if((melbus_LastReadByte[2] == 0xD8 || melbus_LastReadByte[2] == 0xD9) && (melbus_LastReadByte[1] == 0x19 || melbus_LastReadByte[1] == 0x49) && melbus_LastReadByte[0] == 0x52) {
      Serial.println("RND");
    }

    else if((melbus_LastReadByte[2] == 0xD8 || melbus_LastReadByte[2] == 0xD9) && (melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x29)) {
      Serial.println("FF");
    }

    else if((melbus_LastReadByte[2] == 0xD8 || melbus_LastReadByte[2] == 0xD9) && (melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x2F)) {
      Serial.println("Start up");
      melbus_OutByte = 0x00; // Respond to start;
      melbus_SendBuffer[1] = 0x08; // START
      melbus_SendBuffer[8] = 0x08; // START
    }

    else if((melbus_LastReadByte[2] == 0xD8 || melbus_LastReadByte[2] == 0xD9) && (melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x26)) {
      Serial.println("FR");
    } else if((melbus_LastReadByte[3] == 0xD8 || melbus_LastReadByte[3] == 0xD9) && (melbus_LastReadByte[2] == 0x1A || melbus_LastReadByte[2] == 0x4A) && melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x01) {
      Serial.println("Previous disc");
      melbus_SendBuffer[CURRENT_DISC]--;
      melbus_SendBuffer[CURRENT_SONG] = 0x01; // Set current song to 1 when changing disc
    }

    else if((melbus_LastReadByte[3] == 0xD8 || melbus_LastReadByte[3] == 0xD9) && (melbus_LastReadByte[2] == 0x1A || melbus_LastReadByte[2] == 0x4A) && melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x41) {
      Serial.println("Next disc");
      melbus_SendBuffer[CURRENT_DISC]++;
      melbus_SendBuffer[CURRENT_SONG] = 0x01; // Set current song to 1 when changing disc
    }

    else if((melbus_LastReadByte[4] == 0xD8 || melbus_LastReadByte[4] == 0xD9) && (melbus_LastReadByte[3] == 0x1B || melbus_LastReadByte[3] == 0x4B) && melbus_LastReadByte[2] == 0x2D && melbus_LastReadByte[1] == 0x00 && melbus_LastReadByte[0] == 0x01) {
      Serial.println("Previous track");

      if(melbus_SendBuffer[CURRENT_SONG] > 0){
        melbus_SendBuffer[CURRENT_SONG]--;
      } else {
        melbus_SendBuffer[CURRENT_SONG] = 0x09;
      }
    }

    else if((melbus_LastReadByte[4] == 0xD8 || melbus_LastReadByte[4] == 0xD9) && (melbus_LastReadByte[3] == 0x1B || melbus_LastReadByte[3] == 0x4B) && melbus_LastReadByte[2] == 0x2D && melbus_LastReadByte[1] == 0x40 && melbus_LastReadByte[0] == 0x01) {
      Serial.println("Next track");
      if(melbus_SendBuffer[CURRENT_SONG] < 9){
        melbus_SendBuffer[CURRENT_SONG]++;
      } else {
        melbus_SendBuffer[CURRENT_SONG] = 0x01;
      }

    }

    else if((melbus_LastReadByte[4] == 0xD8 || melbus_LastReadByte[4] == 0xD9) && (melbus_LastReadByte[3] == 0x1B || melbus_LastReadByte[3] == 0x4B) && melbus_LastReadByte[2] == 0xE0  && melbus_LastReadByte[1] == 0x01 && melbus_LastReadByte[0] == 0x08 ) {
      // Playinfo
      melbus_SendCnt=9;
    }

    // Communication with RPI
    if((melbus_SendBuffer[CURRENT_SONG] = 0x02) && (melbus_SendBuffer[CURRENT_DISC] = 0x02)){
      digitalWrite(ACTIVATE_BLUETOOTH, HIGH);
    } else if ((melbus_SendBuffer[CURRENT_SONG] = 0x08) && (melbus_SendBuffer[CURRENT_DISC] = 0x08)){
      digitalWrite(ACTIVATE_BLUETOOTH, HIGH);
    } else {
      digitalWrite(ACTIVATE_BLUETOOTH, LOW);
      digitalWrite(ACTIVATE_BLUETOOTH, LOW);
    }

    if (melbus_SendCnt) {
      melbus_OutByte=melbus_SendBuffer[9-melbus_SendCnt];
      melbus_SendCnt--;
    } else if (melbus_DiscCnt) {
      melbus_OutByte=melbus_DiscBuffer[6-melbus_DiscCnt];
      melbus_DiscCnt--;
    } else {
      // Do nothing
    }
  } else {
    // Set bitnumber to address of next bit in byte
    melbus_Bitposition>>=1;
  }
  EIFR |= (1 << INTF1);
}
