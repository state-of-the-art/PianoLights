#include "esp32_digital_led_lib.h"

#include <usbh_midi.h>
#include <usbhub.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

/*********************************/
/********** CONFIGS **************/
/*********************************/
#define POW_BUDGET_200 // Passive usb hub or something with no good power
//#define POW_BUDGET_500 // direct usb 2.0 0.5A max
//#define POW_BUDGET_1000 // direct usb 3.0 1A max
//#define POW_BUDGET_2000 // power adapter 2A

#define colorWheel(wheel_pos) colorWheelBG(wheel_pos)

#define KEYS_SHIFT 0x15

#define KEYS_NUM 88

#define LEDS_PER_KEY 2

#ifdef POW_BUDGET_200
// Total max is 50x32 for green channel with USB host and no BT
#  define MAX_SIM_LEDS 40
#  define MAX_LED_POWER 32
#endif
#ifdef POW_BUDGET_500
#endif
#ifdef POW_BUDGET_1000
#endif
#ifdef POW_BUDGET_2000
#endif

// Defs for BLE MIDI
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
uint8_t _keys[KEYS_NUM]; 

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

uint8_t midi_packet[] = {
   0x80,  // header (1 + 1 + 6 bytes of upper timestamp)
   0x80,  // 1 + 7 bytes of lower timestamp, not implemented 
   0x00,  // status
   0x3c,  // note, 0x3c == 60 == middle c
   0x00   // velocity
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
    // TODO: Proxy to USB raw, not working this way
    //if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) // if( Midi ) is not working
    //  Midi.SendRawData(bufferSize-2, &(buffer[2]));

    // Logic taken from https://github.com/lathoub/Arduino-BLE-MIDI/blob/master/src/Ble_esp32.h
    // Pointers used to search through payload.
    uint8_t lPtr = 0;
    uint8_t rPtr = 0;

    // lastStatus used to capture runningStatus
    uint8_t lastStatus;

    // Decode first packet -- SHALL be "Full MIDI message"
    lPtr = 2; // Start at first MIDI status -- SHALL be "MIDI status"

    // While statement contains incrementing pointers and breaks when buffer size exceeded.
    while(1) {
      lastStatus = buffer[lPtr];
      if( (buffer[lPtr] < 0x80) ){
        // Status message not present, bail
        return;
      }
      // Point to next non-data byte
      rPtr = lPtr;
      while( (buffer[rPtr + 1] < 0x80)&&(rPtr < (bufferSize - 1)) ){
        rPtr++;
      }
      // look at l and r pointers and decode by size.
      if( rPtr - lPtr < 1 ){
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
      if(lPtr >= bufferSize){
          // end of packet
          return;
      }
    }
  }

  void processMIDI(uint8_t data0, uint8_t data1 = 0, uint8_t data2 = 0) {
    Serial.printf("BLE data: %2X %2X %2X\n", data0, data1, data2);
    // Proxy to USB
    if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) { // if( Midi ) is not working
      uint8_t data[] = { data0, data1, data2 };
      Midi.SendData(data, 3);
    }
  }
};

void initBLEMIDI() {
  BLEDevice::init("PianoLights");

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

  for( uint8_t k = 0; k < KEYS_NUM; k++ )
    _keys[k] = 0;

  initBLEMIDI();
  
  delay(100);
  Serial.println("Init completed");

  initAnimation();
}

void setLeds(uint8_t key) {
  pixelColor_t color = _keys[key] > 0 ? colorWheel(_keys[key]) : pixelFromRGB(0, 0, 0);
  LED_STRAND.pixels[key*2] = color;
  LED_STRAND.pixels[key*2+1] = color;
}

void updateLeds(int8_t key = -1) {
  if( key < 0 ) {
    for( uint8_t k; k < KEYS_NUM; k++ )
      setLeds(k);
  } else
    setLeds(key);
}

void pitchSet(int8_t velocity) {
  // TODO: Use pitch to slide the LEDs
  if( velocity > 0x40 )
    _pitch = (velocity-0x40) * 16/10;
  else
    _pitch = (velocity-0x40) * 157/100;
  updateLeds();
}

void noteOff(uint8_t note) {
  _keys[note-KEYS_SHIFT] = 0x0;
  updateLeds(note-KEYS_SHIFT);
}

void noteOn(uint8_t note, uint8_t velocity = 0x40) {
  // TODO: make sure POW_BUDGET is not excessed
  // TODO: use pitch to find the correct location
  // TODO: blue->green->red depends on the velocity
  _keys[note-KEYS_SHIFT] = velocity;
  updateLeds(note-0x15);
}

//**************************************************************************//
// Poll USB MIDI Controler
void USBMIDI_poll() {
  uint8_t outBuf[4]; // TODO: Actually should be 3, but for some reason it crashed when dual voicing enabled
  uint8_t size;

  do {
    if( (size = Midi.RecvData(outBuf)) > 0 ) {
      // Processing lights
      if( outBuf[0] == 0xFE ) { // Timing clock
        continue; // Ignore it
      }
      else if( outBuf[0] >= 0x80 && outBuf[0] < 0x90 ) { // 0x8x Note off
        // TODO: release velocity
        noteOff(outBuf[1]);
      }
      else if( outBuf[0] < 0xA0 ) { // 0x9x Note on
        if( outBuf[2] == 0 )
          noteOff(outBuf[1]);
        else
          noteOn(outBuf[1], outBuf[2]);
      }
      else if( outBuf[0] == 0xE0 ) { // 0xEx Pitch
        pitchSet(outBuf[2]);
      }
      // From: 0x15
      // To: 0x6C
      Serial.printf("USB data: %2X %2X %2X\n", outBuf[0], outBuf[1], outBuf[2]);

      // Proxy to BLE device
      // TODO: proxy raw buffer to speed-up the process
      if( _ble_dev_connected ) {
        midi_packet[2] = outBuf[0]; // Channel and up/down
        midi_packet[3] = outBuf[1]; // Velocity
        midi_packet[4] = outBuf[2]; // Note
        ble_characteristic->setValue(midi_packet, 5);
        ble_characteristic->notify();
      }
    }
  } while( size > 0 );
}

// Delay time (max 16383 us)
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime) {
  uint32_t t3;

  t3 = t2 - t1;
  if( t3 < delayTime ) {
    delayMicroseconds(delayTime - t3);
  }
}

//**************************************************************************//
void loop() {
  Usb.Task();
  uint32_t t1 = (uint32_t)micros();

  if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) { // if( Midi ) is not working
    USBMIDI_poll();
  }

  digitalLeds_drawPixels(STRANDS, 1);

  //delay(1ms)
  doDelay(t1, (uint32_t)micros(), 1000);
}
