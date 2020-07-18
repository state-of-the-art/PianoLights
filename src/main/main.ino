/*********************************/
/********** CONFIGS **************/
/*********************************/
#define POW_BUDGET_200 // Passive usb hub or something with no good power
//#define POW_BUDGET_500 // direct usb 2.0 0.5A max
//#define POW_BUDGET_1000 // direct usb 3.0 1A max
//#define POW_BUDGET_2000 // power adapter 2A

// Color transition depends on velocity
#define colorWheel(wheel_pos) colorWheelBGR(wheel_pos)

// Where is C3
#define KEYS_SHIFT 0x15

// Piano keys number
#define KEYS_NUM 88

// Number of leds per key
#define LEDS_PER_KEY 2

// Color of the ghost note (tip, sent from BLE side on 16th channel)
#define GHOST_COLOR pixelFromRGB(5, 5, 5)

/*********************************/
/*********** INIT ****************/
/*********************************/

#include "esp32_digital_led_lib.h"

#include <usbh_midi.h>
#include <usbhub.h>

#ifdef POW_BUDGET_200
#  define MAX_SIM_LEDS 40
#  define MAX_LED_POWER 32
#endif
#ifdef POW_BUDGET_500
#  define MAX_SIM_LEDS 40
#  define MAX_LED_POWER 32
#endif
#ifdef POW_BUDGET_1000
#  define MAX_SIM_LEDS 40
#  define MAX_LED_POWER 32
#endif
#ifdef POW_BUDGET_2000
#  define MAX_SIM_LEDS 40
#  define MAX_LED_POWER 32
#endif

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#define SERVICE_UUID        "03B80E5A-EDE8-4B33-A751-6CE34EC4C700"
#define CHARACTERISTIC_UUID "7772E5DB-3868-4112-A1A9-F2669D106BF3"

// Enum
// Taken from https://github.com/lathoub/midi-Common/blob/master/midi_Defs.h
enum MidiType : uint8_t {
  InvalidType           = 0x00,    ///< For notifying errors

  NoteOff               = 0x80,    ///< Note Off
  NoteOn                = 0x90,    ///< Note On
  AfterTouchPoly        = 0xA0,    ///< Polyphonic AfterTouch
  ControlChange         = 0xB0,    ///< Control Change / Channel Mode
  ProgramChange         = 0xC0,    ///< Program Change
  AfterTouchChannel     = 0xD0,    ///< Channel (monophonic) AfterTouch
  PitchBend             = 0xE0,    ///< Pitch Bend

  SystemExclusive       = 0xF0,    ///< System Exclusive
  SystemExclusiveStart  = SystemExclusive,
  SystemExclusiveEnd    = 0xF7,    ///< System Exclusive End

  TimeCodeQuarterFrame  = 0xF1,    ///< System Common - MIDI Time Code Quarter Frame
  SongPosition          = 0xF2,    ///< System Common - Song Position Pointer
  SongSelect            = 0xF3,    ///< System Common - Song Select
  TuneRequest           = 0xF6,    ///< System Common - Tune Request

  Clock                 = 0xF8,    ///< System Real Time - Timing Clock
  Tick                  = 0xF9,    ///< System Real Time - Tick
  Start                 = 0xFA,    ///< System Real Time - Start
  Continue              = 0xFB,    ///< System Real Time - Continue
  Stop                  = 0xFC,    ///< System Real Time - Stop
  ActiveSensing         = 0xFE,    ///< System Real Time - Active Sensing
  SystemReset           = 0xFF,    ///< System Real Time - System Reset
};

/*********************************/
/************ APP ****************/
/*********************************/

// Current pitch value: (0; 64; 127) to (-100; 0; 100)
int8_t _pitch = 0;

// Pressed keys: value is velocity 0-127
uint8_t _keys_user[KEYS_NUM];
bool _keys_ghost[KEYS_NUM];

// Number of active leds
uint8_t _leds_active = 0;

// Definition of the led string
strand_t LED_STRAND = {
  .rmtChannel = 0,
  .gpioNum = 13,
  .ledType = LED_WS2812B_V3,
  .brightLimit = 12,
  .numPixels =  KEYS_NUM * LEDS_PER_KEY
};

// BLE Device connected
bool _ble_dev_connected = false;

/********** TRANSITIONS **********/

inline pixelColor_t colorWheelBG(int8_t wheel_pos) {
  int8_t color_val = max(0, min(MAX_LED_POWER, wheel_pos * MAX_LED_POWER / 127));
  return pixelFromRGB(0, color_val, MAX_LED_POWER - color_val);
}

inline pixelColor_t colorWheelGR(int8_t wheel_pos) {
  int8_t color_val = max(0, min(MAX_LED_POWER, wheel_pos * MAX_LED_POWER / 127));
  return pixelFromRGB(color_val, MAX_LED_POWER - color_val, 0);
}

inline pixelColor_t colorWheelBR(int8_t wheel_pos) {
  int8_t color_val = max(0, min(MAX_LED_POWER, wheel_pos * MAX_LED_POWER / 127));
  return pixelFromRGB(color_val, 0, MAX_LED_POWER - color_val);
}

pixelColor_t colorWheelBGR(int8_t wheel_pos) {
  int8_t color_val = 0;
  byte b = 0;
  byte g = 0;
  byte r = 0;
  if( wheel_pos < 64 ) {
    color_val = wheel_pos * MAX_LED_POWER / 64;
    b = MAX_LED_POWER - color_val;
    g = color_val;
  } else {
    wheel_pos -= 64;
    color_val = wheel_pos * MAX_LED_POWER / 64;
    g = MAX_LED_POWER - color_val;
    r = color_val;
  }

  return pixelFromRGB(r, g, b);
}

/********** MAIN LOGIC **********/

USB Usb;
USBH_MIDI Midi(&Usb);

#if defined(ARDUINO) && ARDUINO >= 100
  // No extras
#elif defined(ARDUINO) // pre-1.0
  // No extras
#elif defined(ESP_PLATFORM)
  #include "arduinoish.hpp"
#endif

// Required by LED framework to operate
strand_t * STRANDS[1];

void espPinMode(int pinNum, int pinDir) {
  // Enable GPIO32 or 33 as output. Doesn't seem to work though.
  // https://esp32.com/viewtopic.php?t=9151#p38282
  if (pinNum == 32 || pinNum == 33) {
    uint64_t gpioBitMask = (pinNum == 32) ? 1ULL<<GPIO_NUM_32 : 1ULL<<GPIO_NUM_33;
    gpio_mode_t gpioMode = (pinDir == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = gpioMode;
    io_conf.pin_bit_mask = gpioBitMask;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
  } else pinMode(pinNum, pinDir);
}

void gpioSetup(int gpioNum, int gpioMode, int gpioVal) {
  #if defined(ARDUINO) && ARDUINO >= 100
    espPinMode(gpioNum, gpioMode);
    digitalWrite (gpioNum, gpioVal);
  #elif defined(ESP_PLATFORM)
    gpio_num_t gpioNumNative = static_cast<gpio_num_t>(gpioNum);
    gpio_mode_t gpioModeNative = static_cast<gpio_mode_t>(gpioMode);
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
  #endif
}

boolean initStrand() {
  digitalLeds_initDriver();

  gpioSetup(LED_STRAND.gpioNum, OUTPUT, LOW);
  STRANDS[0] = &LED_STRAND;

  int rc = digitalLeds_addStrands(STRANDS, 1);
  if( rc ) {
    Serial.print("Init rc = ");
    Serial.println(rc);
    return false;
  }

  Serial.print("Strand ");
  Serial.print((uint32_t)(LED_STRAND.pixels), HEX);
  Serial.println();

  return true;
}

void initAnimation() {
  pixelColor_t enabled = pixelFromRGB(12, 12, 12);
  pixelColor_t disabled = pixelFromRGB(0, 0, 0);

  for( int i = 0; i < LED_STRAND.numPixels; i++ ) {
    if( i > 0 )
      LED_STRAND.pixels[i-1] = disabled;
    LED_STRAND.pixels[i] = enabled;
    digitalLeds_drawPixels(STRANDS, 1);
  }
  for( int i = LED_STRAND.numPixels - 1; i >= 0; i-- ) {
    LED_STRAND.pixels[i+1] = disabled;
    LED_STRAND.pixels[i] = enabled;
    digitalLeds_drawPixels(STRANDS, 1);
  }
  LED_STRAND.pixels[0] = disabled;
  digitalLeds_drawPixels(STRANDS, 1);
}

BLECharacteristic *ble_characteristic;

// Structures to proxy from BLE to USB
portMUX_TYPE usb_midi_packet_mux = portMUX_INITIALIZER_UNLOCKED;
uint16_t usb_midi_packet_size = 0;
uint8_t usb_midi_packet[512] = {
  //0x00,  // 4b Channel + type
  //0x00,  // status
  //0x3c,  // note, 0x3c == 60 == middle c
  //0x00   // velocity
};

// Structure to proxy from USB to BLE
uint8_t ble_midi_packet[512] = {
  0x80,  // header (1 + 1 + 6 bytes of upper timestamp)
  //0x80,  // 1 + 7 bytes of lower timestamp, not implemented 
  //0x00,  // status
  //0x3c,  // note, 0x3c == 60 == middle c
  //0x00   // velocity
};

class MyBLEServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("BLE device connected");
    _ble_dev_connected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.println("BLE device disconnected");
    _ble_dev_connected = false;
  }
};

class MyBLECharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string rxValue = characteristic->getValue();
    if( rxValue.length() > 0 )
      processWrite((uint8_t *)(rxValue.c_str()), rxValue.length());
  }

  void processWrite(uint8_t *buffer, uint8_t bufferSize) {
    // Logic taken from https://github.com/lathoub/Arduino-BLE-MIDI/blob/master/src/Ble_esp32.h
    // Pointers used to search through payload.
    uint8_t lPtr = 0;
    uint8_t rPtr = 0;

    // lastStatus used to capture runningStatus
    uint8_t lastStatus;

    // Decode first packet -- SHALL be "Full MIDI message"
    lPtr = 2; // Start at first MIDI status -- SHALL be "MIDI status"

    // TODO: seems like the usb_midi_packet buffer realization I'm using is not
    // working properly without some debug... Need to figure out what's wrong.
    Serial.printf("BLE data received %i: ", bufferSize);
    for( uint8_t i = 0; i<bufferSize; i++ )
      Serial.printf(" %02X", buffer[i]);
    Serial.println("");

    // While statement contains incrementing pointers and breaks when buffer size exceeded.
    while(1) {
      lastStatus = buffer[lPtr];
      if( (buffer[lPtr] < 0x80) ){
        // Status message not present, bail
        break;
      }
      // Point to next non-data byte
      rPtr = lPtr;
      while( (buffer[rPtr + 1] < 0x80)&&(rPtr < (bufferSize - 1)) ){
        rPtr++;
      }

      // look at l and r pointers and decode by size.
      if( rPtr - lPtr < 1 ) {
        // Time code or system
        processMIDI(lastStatus);
      } else if( rPtr - lPtr < 2 ) {
        processMIDI(lastStatus, buffer[lPtr + 1]);
      } else if( rPtr - lPtr < 3 ) {
        processMIDI(lastStatus, buffer[lPtr + 1], buffer[lPtr + 2]);
      } else {
        // Too much data
        // If not System Common or System Real-Time, send it as running status
        switch( buffer[lPtr] & 0xF0 )
        {
          case NoteOff:
          case NoteOn:
          case AfterTouchPoly:
          case ControlChange:
          case PitchBend:
            for(int i = lPtr; i < rPtr; i = i + 2)
              processMIDI(lastStatus, buffer[i + 1], buffer[i + 2]);
            break;
          case ProgramChange:
          case AfterTouchChannel:
            for(int i = lPtr; i < rPtr; i = i + 1)
              processMIDI(lastStatus, buffer[i + 1]);
            break;
          default:
            break;
        }
      }
      // Point to next status
      lPtr = rPtr + 2;
      if( lPtr >= bufferSize ) {
          // end of packet
          break;
      }
    }
  }

  void processMIDI(uint8_t data0, uint8_t data1 = 0, uint8_t data2 = 0) {
    //Serial.printf("BLE->USB data: %2X %2X %2X\n", data0, data1, data2);
    // Proxy to USB
    portENTER_CRITICAL_ISR(&usb_midi_packet_mux);
      usb_midi_packet[usb_midi_packet_size++] = Midi.lookupMsgSize(data0); // Proxy using cable 0
      usb_midi_packet[usb_midi_packet_size++] = data0;
      usb_midi_packet[usb_midi_packet_size++] = data1;
      usb_midi_packet[usb_midi_packet_size++] = data2;
    portEXIT_CRITICAL_ISR(&usb_midi_packet_mux);

    // Check the 16th channel on key lights
    if( (data0 & 0b1111) == 0xF ) {
      switch( data0 & 0xF0 )
      {
        case NoteOff:
          signalNoteGhostOff(data1);
          break;
        case NoteOn:
          if( data2 == 0 )
            signalNoteGhostOff(data1);
          else
            signalNoteGhostOn(data1);
          break;
        case AfterTouchPoly:
        case ControlChange:
        case PitchBend:
        case ProgramChange:
        case AfterTouchChannel:
        default:
          break;
      }
    }
  }
};

void initBLEMIDI() {
  BLEDevice::init("PianoLights BLE");

  // Create the BLE Server
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new MyBLEServerCallbacks());

  // Create the BLE Service
  BLEService *service = server->createService(BLEUUID(SERVICE_UUID));

  // Create a BLE Characteristic
  ble_characteristic = service->createCharacteristic(
    BLEUUID(CHARACTERISTIC_UUID),
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  // Create a BLE Descriptor
  ble_characteristic->addDescriptor(new BLE2902());
  ble_characteristic->setCallbacks(new MyBLECharacteristicCallbacks());

  // Start the service
  service->start();

  // Start advertising
  BLEAdvertising *advertising = server->getAdvertising();
  advertising->addServiceUUID(service->getUUID());
  advertising->start();

  Serial.println("BLE Init done");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing PianoLights...");

  // Reset USB
  pinMode( 15, OUTPUT);
  digitalWrite(15, 1);
  digitalWrite(15, 0);
  delay(10);
  digitalWrite(15, 1);

  // Init components
  if( !initStrand() || Usb.Init() == -1 ) {
    Serial.println("Init FAILURE: halting");
    while( true );
  }

  for( uint8_t k = 0; k < KEYS_NUM; k++ ) {
    _keys_user[k] = 0;
    _keys_ghost[k] = false;
  }

  initBLEMIDI();

  delay(100);
  Serial.println("Init completed");

  initAnimation();
}

void setLeds(uint8_t key) {
  pixelColor_t color;
  if( _keys_user[key] > 0 )
    color = colorWheel(_keys_user[key]);
  else if( _keys_ghost[key] )
    color = GHOST_COLOR;
  else
    color = pixelFromRGB(0, 0, 0);

  LED_STRAND.pixels[key*LEDS_PER_KEY] = color;
  LED_STRAND.pixels[key*LEDS_PER_KEY+1] = color;
}

void updateLeds(int8_t key = -1) {
  if( key < 0 ) {
    for( uint8_t k = 0; k < KEYS_NUM; k++ )
      setLeds(k);
  } else
    setLeds(key);
}

void pitchSet(int8_t velocity) {
  // TODO: Use pitch to slide the LEDs
  /*if( velocity > 0x40 )
    _pitch = (velocity-0x40) * 16/10;
  else
    _pitch = (velocity-0x40) * 157/100;
  updateLeds();*/
}

void signalNoteOff(uint8_t note) {
  uint8_t key = note-KEYS_SHIFT;
  if( _keys_user[key] != 0x00 ) {
    // Disable only user key - the ghost key will be disabled when the user press key
    _keys_user[key] = 0x00;
    if( !_keys_ghost[key] )
      _leds_active -= LEDS_PER_KEY;
  } else if( _keys_ghost[key] ) {
    _keys_ghost[key] = false;
    _leds_active -= LEDS_PER_KEY;
  } else
    return; // Is not set already
  updateLeds(key);
}

void signalNoteGhostOff(uint8_t note) {
  uint8_t key = note-KEYS_SHIFT;
  if( _keys_ghost[key] ) {
    _keys_ghost[key] = false;
    if( _keys_user[key] == 0x00 )
      _leds_active -= LEDS_PER_KEY;
  } else
    return; // Leave the user key as is
  updateLeds(key);
}

void signalNoteOn(uint8_t note, uint8_t velocity = 0x40) {
  // Check the power budget
  if( _leds_active >= MAX_SIM_LEDS )
    return;

  uint8_t key = note-KEYS_SHIFT;
  if( _keys_ghost[key] )
    _keys_ghost[key] = false; // Disable ghost if user pressed the key
  else if( _keys_user[key] == 0x00 )
    _leds_active += LEDS_PER_KEY;
  else
    return; // Already set - no need to reset it

  _keys_user[key] = velocity;
  updateLeds(key);
}

void signalNoteGhostOn(uint8_t note) {
  if( _leds_active >= MAX_SIM_LEDS )
    return;

  uint8_t key = note-KEYS_SHIFT;
  if( _keys_ghost[key] )
    return; // Already set - no need to reset it
  else if( _keys_user[key] == 0x00 )
    _leds_active += LEDS_PER_KEY;

  _keys_ghost[key] = true;
  updateLeds(key);
}

//**************************************************************************//
// Poll USB MIDI Controler
void USBMIDI_poll() {
  uint8_t outBuf[4]; // TODO: Actually should be 3, but for some reason it crashed when dual voicing enabled
  uint8_t size;
  uint16_t ble_midi_packet_size = 0;

  do {
    if( (size = Midi.RecvData(outBuf)) == 0 )
      break;

    // Processing lights
    if( outBuf[0] == 0xFE ) { // Timing clock
      continue; // Ignore it
    }
    switch( outBuf[0] & 0xF0 )
    {
      case NoteOff:
        // TODO: release velocity
        signalNoteOff(outBuf[1]);
        break;
      case NoteOn:
        if( outBuf[2] == 0 )
          signalNoteOff(outBuf[1]);
        else
          signalNoteOn(outBuf[1], outBuf[2]);
        break;
      case PitchBend:
        // From: 0x15
        // To: 0x6C
        pitchSet(outBuf[2]);
        break;
      case AfterTouchPoly:
      case ControlChange:
      case ProgramChange:
      case AfterTouchChannel:
      default:
        break;
    }
    //Serial.printf("USB->BLE data: %2X %2X %2X\n", outBuf[0], outBuf[1], outBuf[2]);

    // Proxy to BLE
    if( _ble_dev_connected ) {
      // Check is not running MIDI status
      if( !(ble_midi_packet_size > 2 && ble_midi_packet[ble_midi_packet_size-2] == outBuf[0]) ) {
        ble_midi_packet[++ble_midi_packet_size] = 0x80; // timestamp
        ble_midi_packet[++ble_midi_packet_size] = outBuf[0]; // Status
      }
      ble_midi_packet[++ble_midi_packet_size] = outBuf[1]; // Velocity
      ble_midi_packet[++ble_midi_packet_size] = outBuf[2]; // Note
    }
  } while( size > 0 );

  if( _ble_dev_connected && ble_midi_packet_size > 0 ) {
    //Serial.printf("USB->BLE sending: %i\n", ble_midi_packet_size+1);
    ble_characteristic->setValue(ble_midi_packet, ble_midi_packet_size+1);
    ble_characteristic->notify();
    //Serial.printf("USB->BLE data sent: %i\n", ble_midi_packet_size+1);
  }
}

// Delay time (max 16383 us)
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime) {
  uint32_t t3;

  t3 = t2 - t1;
  if( t3 < delayTime ) {
    delayMicroseconds(delayTime - t3);
  }
}

void sendBLEUSB() {
    // Proxy BLE->USB
    if( usb_midi_packet_size > 0 ) {
      /*Serial.printf("BLE->USB data sending %i: ", usb_midi_packet_size);
      for( uint8_t i = 0; i<usb_midi_packet_size; i++ )
        Serial.printf(" %02X", usb_midi_packet[i]);
      Serial.println("");*/
      portENTER_CRITICAL(&usb_midi_packet_mux);
        Midi.SendRawData(usb_midi_packet_size, usb_midi_packet);
        usb_midi_packet_size = 0;
      portEXIT_CRITICAL(&usb_midi_packet_mux);
    }
}

//**************************************************************************//
void loop() {
  Usb.Task();
  uint32_t t1 = (uint32_t)micros();

  if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) { // if( Midi ) is not working
    USBMIDI_poll();
    sendBLEUSB();
  }

  // The leds calculations could be improper - so update if it's overflowing
  if( _leds_active > MAX_SIM_LEDS ) {
    Serial.printf("WARN: too much led is used - resetting %i to ", _leds_active);
    uint8_t leds_act = 0;
    for( uint8_t k = 0; k < KEYS_NUM; k++ ) {
      if( _keys_user[k] > 0x00 || _keys_ghost[k] )
        leds_act += LEDS_PER_KEY;
    }
    Serial.printf("%i \r\n", leds_act);
    if( leds_act > MAX_SIM_LEDS ) {
      Serial.printf("ERROR: Still too much - resetting the led strip & keys");
      for( uint8_t k = 0; k < KEYS_NUM; k++ ) {
        _keys_user[k] = 0x00;
        _keys_ghost[k] = false;
      }
      updateLeds();
      _leds_active = 0;
    } else
      _leds_active = leds_act;
  }

  digitalLeds_drawPixels(STRANDS, 1);

  //delay(1ms)
  doDelay(t1, (uint32_t)micros(), 1000);
}
