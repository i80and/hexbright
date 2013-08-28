/*
    A bastardized monstrosity containing original code, factory firmware code, and
    dhiltonp's firmware framework.  For licensing details, please see the provided LICENSE file.
*/

#include <stdint.h>
#include <math.h>
#include <Wire.h>

// Settings
#define OVERTEMP                340
#define SAFE_TEMP               250

// Pin assignments
#define DPIN_RLED_SW            2
#define DPIN_GLED               5
#define DPIN_PWR                8
#define DPIN_DRV_MODE           9
#define DPIN_DRV_EN             10
#define APIN_TEMP               0
#define APIN_CHARGE             3

// Modes
#define INTENSITY_OFF 0
#define INTENSITY_LOW 1
#define INTENSITY_MED 2
#define INTENSITY_HIGH 3

#define CHARGING 2
#define CHARGED 1
#define BATTERY 0

// Voltage stuff
int band_gap_reading = 0;
int lowest_band_gap_reading = 1000;

#define APIN_BAND_GAP 14
unsigned int read_adc(unsigned char pin) {
    // a useful reference: http://www.protostack.com/blog/2011/02/analogue-to-digital-conversion-on-an-atmega168/

    // configure adc: use refs0, pin = some combination of MUX(0-3).
    // Setting _BV(ADLAR) could be useful, but it just saves 16 bytes and reduces resolution by four.
    // If you set _BV(ADLAR) here, return ADCH as an unsigned char.
    ADMUX = _BV(REFS0) | pin;

    // Wait for Vref to settle - this costs 2 bytes, and is crucial for APIN_BAND_GAP
    // 150 is usually enough for the band gap to stabilize, give us some room for error.
    delayMicroseconds(250);

    // Start analog to digital conversion (used to be the sbi macro)
    ADCSRA |= _BV(ADSC);

    // wait for the conversion to complete (ADSC bit of ADCSRA is cleared, aka ADSCRA & ADSC)
    while (bit_is_set(ADCSRA, ADSC));

    return ADC;
}

void read_avr_voltage(byte chargeState) {
    band_gap_reading = read_adc(APIN_BAND_GAP);
    if(chargeState==BATTERY)
        lowest_band_gap_reading = (band_gap_reading < lowest_band_gap_reading) ? band_gap_reading : lowest_band_gap_reading;
}

int get_avr_voltage() {
    // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    // this is the only place we actually convert to voltage, reducing the space used for most programs.
    return ((long)1023*1100) / band_gap_reading;
}

boolean low_voltage_state() {
    static boolean low = false;
    // lower band gap value corresponds to a higher voltage, trigger
    // low voltage state if band gap value goes too high.
    // I have a value of 2 for this to work (with a 150 ms delay in read_adc).
    // tighter control means earlier detection of low battery state
    if (band_gap_reading > lowest_band_gap_reading+2) {
        low = true;
    }
    return low;
}

class ChargeController {
public:
    ChargeController(): _currentChargeState(BATTERY), _oldChargeState(BATTERY), _chargeTime(0) {}

    void init() {
        _chargeTime = millis();
        _currentChargeState = getImmediateChargeState();
        _oldChargeState = _currentChargeState;
    }

    byte getChargeState(unsigned long time) {
        // Check the state of the charge controller
        byte chargeState = getImmediateChargeState();
        byte result = _currentChargeState;

        // The charging pin gives confusing results if the light is discharged while plugged in.
        // Consequently, we only acknowledge state changes lasting more than 100ms
        if((chargeState == _oldChargeState) && ((time-_chargeTime) > 100)) {
            result = chargeState;
            _currentChargeState = chargeState;
        }

        if(chargeState != _oldChargeState) {
            _chargeTime = time;
            _oldChargeState = chargeState;
        }

        return result;
    }

private:
    byte _currentChargeState;
    byte _oldChargeState;
    unsigned long _chargeTime;

    byte getImmediateChargeState() {
        int chargeState = analogRead(APIN_CHARGE);
        if (chargeState < 128) {
            return CHARGING;
        }
        else if (chargeState > 768) {
            return CHARGED;
        }
        else {
            return BATTERY;
        }
    }
} chargeController;

#define BLINK_RATE 0
#define LED_OFF 0
#define LED_BLINK 1
#define LED_ON 2
// Absolutely ridiculous and overwrought LED manager.  Templated over pin ID, and providing
// several features:
//   - Blinking (Mainloop must call handle())
//   - Counted command handling: provides context managers.  For instance, in the case of
//     blink(), blink(), stopBlink(), the LED should remain blinking.
template <int PIN>
class LED {
public:
    class Context {
    public:
        Context(LED& led): _led(led), _mode(LED_OFF) {}

        void on() {
            if(_mode == LED_BLINK) _led.decBlink();
            if(_mode != LED_ON) {
                _mode = LED_ON;
                _led.incOn();
            }
        }

        void blink() {
            if(_mode == LED_ON) _led.decOn();
            if(_mode != LED_BLINK) {
                _mode = LED_BLINK;
                _led.incBlink();
            }
        }

        void off() {
            if(_mode == LED_ON) _led.decOn();
            else if(_mode == LED_BLINK) _led.decBlink();
            _mode = LED_OFF;
        }

        ~Context() {
            off();
        }

    private:
        LED& _led;
        byte _mode;
    };

    LED(): _nOns(0), _nBlinks(0) {}

    void handle(long time) {
        if(_nOns > 0) {
            digitalWrite(PIN, HIGH);
        }
        else if(_nBlinks > 0) {
            digitalWrite(PIN, (time&(0x100<<BLINK_RATE))? LOW : HIGH);
        }
        else {
            digitalWrite(PIN, LOW);
        }
    }

    Context getContext() {
        return Context(*this);
    }

    friend Context;

private:
    byte _nOns;
    byte _nBlinks;

    void incOn() {
        _nOns += 1;
    }

    void decOn() {
        _nOns -= 1;
    }

    void incBlink() {
        _nBlinks += 1;
    }

    void decBlink() {
        _nBlinks -= 1;
    }
};

// Controls the green LED
typedef LED<DPIN_GLED> GreenLED;

// Wrapper controlling the red LED
class RedLED {
public:
    typedef LED<DPIN_RLED_SW>::Context Context;

    Context getContext() {
        return _led.getContext();
    }

    void handle(long time) {
        pinMode(DPIN_RLED_SW, OUTPUT);
        _led.handle(time);
    }

private:
    LED<DPIN_RLED_SW> _led;
};

// Structure controlling the light and LEDs
struct HexBright {
    HexBright(): _maxPower(255) {}

    void setHighPower(boolean high) {
        digitalWrite(DPIN_DRV_MODE, high ? HIGH : LOW);
    }

    void setPower(byte power) {
        if(power == 0) {
            digitalWrite(DPIN_DRV_EN, LOW);
            return;
        }

        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, HIGH);

        if(power > _maxPower) power = _maxPower;
        analogWrite(DPIN_DRV_EN, power);
    }

    void setIntensity(byte intensity) {
        unsigned int gamma = (intensity*intensity)>>8;
        if (gamma < 6) gamma = 6;
        setPower(gamma);
    }

    void setMaxPower(byte power) {
        _maxPower = power;
    }

    // Shut off the HexBright's power; resets program state
    void off() {
        Serial.println("Powering off");
        setPower(0);

        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, LOW);
    }

    GreenLED gled;
    RedLED rled;

private:
    byte _maxPower;
} hb;

// Abstract base class for all state machine handler objects
class Handler {
public:
    virtual void init() = 0;

    // Called during the mainloop after all events have been handled
    virtual void handle(long time) {}

    // Called when the button is first pressed
    virtual void onButtonDown() {}

    // Called when the button is released after a momentary press
    virtual void onButtonUp() {}

    // Called when the button has been held down for a period of time
    virtual void onButtonHold(long holdTime) {}

    // Called when the button has been released after being held down
    virtual void onButtonHoldRelease(long holdTime) {}

    // Called when the light is overheating.  This should usually repower the light
    virtual void onHighTemperature() {}

    // Return this handler's human-readable name: useful for debugging
    virtual const char* getName() = 0;
};

#define MIN_INTERVAL 20
#define HOLD_INTERVAL 250
class Dispatcher {
public:
    Dispatcher(): _eventHandler(NULL), _btnDown(false) {}

    void init();

    void dispatchTemperatureEvents() {
        _eventHandler->onHighTemperature();
    }

    void dispatchButtonEvents(long time) {
        byte newBtnDown = digitalRead(DPIN_RLED_SW);
        long buttonDownTime = (time - _btnTime);

        if(!_btnDown && newBtnDown) {
            _eventHandler->onButtonDown();
        }
        else if(_btnDown && !newBtnDown && (buttonDownTime > MIN_INTERVAL)) {
            if(buttonDownTime > HOLD_INTERVAL)
                _eventHandler->onButtonHoldRelease(buttonDownTime);
            else
                _eventHandler->onButtonUp();
        }
        else if(_btnDown && newBtnDown && (buttonDownTime > HOLD_INTERVAL)) {
            _eventHandler->onButtonHold(buttonDownTime);
        }

        _eventHandler->handle(time);

        // Remember button state so we can detect transitions
        if (newBtnDown != _btnDown) {
            _btnTime = time;
            _btnDown = newBtnDown;
            delay(50);
        }
    }

    void setHandler(Handler& newHandler) {
        _eventHandler = &newHandler;
        _eventHandler->init();
    }

private:
    Handler* _eventHandler;

    byte _btnDown;
    long _btnTime;
} dispatcher;

class OffHandler: public Handler {
public:
    virtual void init() {
        Serial.println("Mode = Off");
        hb.off();
    }

    virtual void onButtonHold(long holdTime);
    virtual void onButtonUp();

    virtual const char* getName() {return "Off";}
} offHandler;

class ToggleHandler: public Handler {
public:
    void init() {
        Serial.println("Mode = Toggle");
        _intensity = INTENSITY_LOW;
        _dirty = true;
    }

    void setIntensity(byte intensity) {
        _intensity = intensity;
        _dirty = true;
    }

    virtual void handle(long time) {
        if(_dirty) {
            switch(_intensity) {
                case INTENSITY_HIGH: {
                    Serial.println("Toggle = High");
                    hb.setHighPower(true);
                    hb.setPower(255);
                    break;
                }
                case INTENSITY_MED: {
                    Serial.println("Toggle = Medium");
                    hb.setHighPower(false);
                    hb.setPower(255);
                    break;
                }
                case INTENSITY_LOW: {
                    Serial.println("Toggle = Low");
                    hb.setHighPower(false);
                    hb.setPower(64);
                    break;
                }
                case INTENSITY_OFF: {
                    Serial.println("Toggle = Off");
                    hb.setPower(0);
                    break;
                }
            }

            _dirty = false;
        }
    }

    virtual void onButtonHold(long holdTime);
    virtual void onButtonHoldRelease(long holdTime);
    virtual void onButtonUp();

    virtual void onHighTemperature() {
        setIntensity(INTENSITY_LOW);
        _dirty = true;
    }

    virtual const char* getName() {return "Toggle";}

private:
    byte _intensity;
    boolean _dirty;
} toggleHandler;

class StrobeHandler: public Handler {
public:
    virtual void init() {
        Serial.println("Mode = Strobe");
        hb.setHighPower(true);
        hb.setPower(255);
    }

    virtual void handle(long time) {
        hb.setPower(((time%70)<11)? 255 : 0);
    }

    virtual void onButtonHoldRelease(long holdTime);

    virtual void onHighTemperature();

    virtual const char* getName() {return "Strobe";}
} strobeHandler;

static const byte MORSE[] = {
    0x02, 0x02, // A .-
    0x04, 0x01, // B -...
    0x04, 0x05, // C -.-.
    0x03, 0x01, // D -..
    0x01, 0x00, // E .
    0x04, 0x04, // F ..-.
    0x03, 0x03, // G --.
    0x04, 0x00, // H ....
    0x02, 0x00, // I ..
    0x04, 0x0E, // J .---
    0x03, 0x05, // K -.-
    0x04, 0x02, // L .-..
    0x02, 0x03, // M --
    0x02, 0x01, // N -.
    0x03, 0x07, // O ---
    0x04, 0x06, // P .--.
    0x04, 0x0B, // Q --.-
    0x03, 0x02, // R .-.
    0x03, 0x00, // S ...
    0x01, 0x01, // T -
    0x03, 0x04, // U ..-
    0x04, 0x08, // V ...-
    0x03, 0x06, // W .--
    0x04, 0x09, // X -..-
    0x04, 0x0D, // Y -.--
    0x04, 0x03, // Z --..
    0x05, 0x1F, // 0 -----
    0x05, 0x1E, // 1 .----
    0x05, 0x1C, // 2 ..---
    0x05, 0x18, // 3 ...--
    0x05, 0x10, // 4 ....-
    0x05, 0x00, // 5 .....
    0x05, 0x01, // 6 -....
    0x05, 0x03, // 7 --...
    0x05, 0x07, // 8 ---..
    0x05, 0x0F, // 9 ----.
    0x00, 0x00, // Bad character
};

#define MORSE_BAD_INDEX 255

class MorseHandler: public Handler {
public:
    MorseHandler(const char* text)
    : _text(text), _byteIndex(0), _pulseIndex(0), _nextWakeup(0), _nextSleepWakeup(0) {}

    virtual void init() {
        Serial.print("Mode = Morse(\"");
        Serial.print(_text);
        Serial.println("\")");

        _byteIndex = 0;
        _pulseIndex = 0;
        _nextWakeup = 0;
        _nextSleepWakeup = 0;
    }

    virtual void handle(long time) {
        if((_nextWakeup != 0) && (time < _nextWakeup)) return;
        if((_nextSleepWakeup != 0) && (time < _nextSleepWakeup)) {
            hb.setPower(0);
            return;
        }

        // Loop, with an implicit word seperator
        if(_text[_byteIndex] == '\0') {
            _byteIndex = 0;
            _pulseIndex = 0;

            _nextSleepWakeup = time + millisPerDash;
            return;
        }

        // Sleep between words
        if(_text[_byteIndex] == ' ') {
            _nextSleepWakeup = time + millisPerDash;
            return;
        }

        const byte codeIndex = getCodeIndex(_text[_byteIndex]);
        if(codeIndex == MORSE_BAD_INDEX) {
            Serial.println("Bad morse index: ");
            Serial.println(_text[_byteIndex]);
            _nextSleepWakeup = time + millisPerBeat;
            return;
        }

        const byte codeLen = MORSE[codeIndex];
        const byte code = MORSE[codeIndex + 1];

        // Go on to the next byte if the current pulse is done
        if((_pulseIndex >= codeLen)) {
            _byteIndex += 1;
            _pulseIndex = 0;
            return;
        }

        boolean isDash = ((code >> _pulseIndex) & 0x01);

        if(isDash) {
            _nextWakeup = time + millisPerDash;
            _nextSleepWakeup = _nextWakeup + millisPerBeat;
        }
        else {
            _nextWakeup = time + millisPerBeat;
            _nextSleepWakeup = _nextWakeup + millisPerBeat;
        }

        hb.setHighPower(true);
        hb.setPower(255);

        _pulseIndex += 1;
    }

    virtual void onButtonUp();

    virtual const char* getName() {return "Morse";}

private:
    const char* _text;
    byte _byteIndex;
    byte _pulseIndex;

    unsigned long _nextWakeup;
    unsigned long _nextSleepWakeup;

    static const unsigned long millisPerBeat = 250;
    static const unsigned long millisPerDash = millisPerBeat * 3;

    byte getCodeIndex(char ch) {
        byte i = 0;

        if (ch >= 'A' && ch <= 'Z') i = (ch - 'A');
        else if (ch >= 'a' && ch <= 'z') i = (ch - 'a');
        else if (ch >= '0' && ch <= '9') i = (ch - '0' - 26);
        else return MORSE_BAD_INDEX;

        return i * 2;
    }
} sosHandler("SOS");

void OffHandler::onButtonHold(long time) {
    if(time > 500) dispatcher.setHandler(strobeHandler);
}

void OffHandler::onButtonUp() {
    dispatcher.setHandler(toggleHandler);
}

void ToggleHandler::onButtonHold(long time) {
    if(_intensity != INTENSITY_HIGH) {
        setIntensity(INTENSITY_OFF);
    }
    else {
        dispatcher.setHandler(sosHandler);
    }
}

void ToggleHandler::onButtonHoldRelease(long holdTime) {
    dispatcher.setHandler(offHandler);
}

void ToggleHandler::onButtonUp() {
    if(_intensity == INTENSITY_HIGH) {
        setIntensity(INTENSITY_LOW);
        dispatcher.setHandler(offHandler);
    }
    else if(_intensity == INTENSITY_MED) {
        setIntensity(INTENSITY_HIGH);
    }
    else if(_intensity == INTENSITY_LOW) {
        setIntensity(INTENSITY_MED);
    }
}

void StrobeHandler::onButtonHoldRelease(long time) {
    dispatcher.setHandler(offHandler);
}

void StrobeHandler::onHighTemperature() {
    dispatcher.setHandler(offHandler);
}

void MorseHandler::onButtonUp() {
    dispatcher.setHandler(offHandler);
}

void Dispatcher::init() {
    setHandler(offHandler);

    _btnTime = millis();
    _btnDown = digitalRead(DPIN_RLED_SW);
}

// Main code
void setup()
{
    // We just powered on!  That means either we got plugged
    // into USB, or the user is pressing the power button.
    pinMode(DPIN_PWR,      INPUT);
    digitalWrite(DPIN_PWR, LOW);

    // Initialize GPIO
    pinMode(DPIN_RLED_SW,  INPUT);
    pinMode(DPIN_GLED,     OUTPUT);
    pinMode(DPIN_DRV_MODE, OUTPUT);
    pinMode(DPIN_DRV_EN,   OUTPUT);
    digitalWrite(DPIN_DRV_MODE, LOW);
    digitalWrite(DPIN_DRV_EN,   LOW);

    // Initialize serial busses
    Serial.begin(9600);
    Wire.begin();

    dispatcher.init();
    chargeController.init();

    Serial.println("Powered up!");
}

GreenLED::Context chargingLEDContext = hb.gled.getContext();

RedLED::Context temperatureLEDContext = hb.rled.getContext();
RedLED::Context batteryLEDContext = hb.rled.getContext();

void loop()
{
    static unsigned long lastTempTime;

    unsigned long time = millis();

    // Check the state of the charge controller
    byte chargeState = chargeController.getChargeState(time);

    if(chargeState == CHARGING) {
        chargingLEDContext.blink();
    }
    else if(chargeState == CHARGED) {
        chargingLEDContext.on();
    }
    else if(chargeState == BATTERY) {
        chargingLEDContext.off();

        read_avr_voltage(chargeState);
        if(low_voltage_state()) {
            batteryLEDContext.blink();
        }
        else {
            batteryLEDContext.off();
        }
    }

    // Check the temperature sensor
    if(time-lastTempTime > 1000) {
        lastTempTime = time;
        int temperature = analogRead(APIN_TEMP);

        //Serial.print("Temp: ");
        //Serial.println(temperature);
        if(temperature > OVERTEMP) {
            Serial.println("Overheating!");
            hb.setMaxPower(64);
            temperatureLEDContext.blink();
            dispatcher.dispatchTemperatureEvents();
        }
        else if(temperature < SAFE_TEMP) {
            hb.setMaxPower(255);
            temperatureLEDContext.off();
        }
    }

    // Periodically pull down the button's pin, since
    // in certain hardware revisions it can float.
    pinMode(DPIN_RLED_SW, OUTPUT);
    pinMode(DPIN_RLED_SW, INPUT);

    dispatcher.dispatchButtonEvents(time);

    hb.gled.handle(time);
    hb.rled.handle(time);
}
