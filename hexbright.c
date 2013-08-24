/*

A bastardized hybrid of the factory firmware with dhiltonp's firmware framework
    https://github.com/dhiltonp/hexbright/

*/

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

// State
unsigned long btnTime = 0;
boolean btnDown = false;

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

byte getChargeState() {
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

#define BLINK_RATE 0

#define LED_OFF 0
#define LED_BLINK 1
#define LED_ON 2
template <int PIN>
class LED {
public:
    class BlinkContext {
    public:
        BlinkContext(LED& led): _led(led), _isBlinking(false) {}

        void blink() {
            if(!_isBlinking) {
                _isBlinking = true;
                _led.incBlink();
            }
        }

        void stopBlink() {
            if(_isBlinking) {
                _isBlinking = false;
                _led.decBlink();
            }
        }

        ~BlinkContext() {
            stopBlink();
        }

    private:
        LED& _led;
        boolean _isBlinking;
    };

    class OnContext {
    public:
        OnContext(LED& led): _led(led), _isOn(false) {}

        void on() {
            if(!_isOn) {
                _isOn = true;
                _led.incOn();
            }
        }

        void off() {
            if(_isOn) {
                _isOn = false;
                _led.decOn();
            }
        }

        ~OnContext() {
            off();
        }

    private:
        LED& _led;
        boolean _isOn;
    };

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

    OnContext on() {
        return OnContext(*this);
    }

    BlinkContext blink() {
        return BlinkContext(*this);
    }

    friend BlinkContext;
    friend OnContext;

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

typedef LED<DPIN_GLED> GreenLED;

struct HexBright {
    HexBright(): _maxPower(255) {}

    void setHighPower(boolean high) {
        digitalWrite(DPIN_DRV_MODE, high ? HIGH : LOW);
    }

    void setPower(byte power) {
        pinMode(DPIN_PWR, OUTPUT);

        if(power == 0) {
            digitalWrite(DPIN_PWR, LOW);
            digitalWrite(DPIN_DRV_EN, LOW);
            return;
        }

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

    GreenLED gled;
    class RedLED {
    public:
        typedef LED<DPIN_RLED_SW>::BlinkContext BlinkContext;
        typedef LED<DPIN_RLED_SW>::OnContext OnContext;

        OnContext on() {
            return _led.on();
        }

        BlinkContext blink() {
            return _led.blink();
        }

        void handle(long time) {
            pinMode(DPIN_RLED_SW, OUTPUT);
            _led.handle(time);
        }

    private:
        LED<DPIN_RLED_SW> _led;
    } rled;

private:
    byte _maxPower;
} hb;

class Handler {
public:
    virtual void init() {}

    virtual void handle(long time) {}

    // Called when the button is first pressed
    virtual void onButtonDown() {}

    // Called when the button is released after a momentary press
    virtual void onButtonUp() {}

    // Called when the button has been held down for a period of time
    virtual void onButtonHold(long holdTime) {}

    // Called when the button has been released after being held down
    virtual void onButtonHoldRelease(long holdTime) {}

    virtual void onHighTemperature() {}

    // Useful for debugging
    virtual const char* getName() {return "Base";}

    void setHandler(Handler* newHandler);
};

class OffHandler: public Handler {
public:
    virtual void init() {
        Serial.println("Mode = Off");
        hb.setPower(0);
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
            pinMode(DPIN_PWR, OUTPUT);

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

Handler* eventHandler = &offHandler;

void Handler::setHandler(Handler* newHandler) {
    eventHandler = newHandler;
    eventHandler->init();
}

void OffHandler::onButtonHold(long time) {
    Serial.println("Off: hold");
    if(time > 500) setHandler(&strobeHandler);
}

void OffHandler::onButtonUp() {
    setHandler(&toggleHandler);
}

void ToggleHandler::onButtonHold(long time) {
    setIntensity(INTENSITY_OFF);
}

void ToggleHandler::onButtonHoldRelease(long holdTime) {
    _intensity = INTENSITY_LOW;
    setHandler(&offHandler);
}

void ToggleHandler::onButtonUp() {
    if(_intensity == INTENSITY_HIGH) {
        setIntensity(INTENSITY_LOW);
        setHandler(&offHandler);
    }
    else if(_intensity == INTENSITY_MED) {
        setIntensity(INTENSITY_HIGH);
    }
    else if(_intensity == INTENSITY_LOW) {
        setIntensity(INTENSITY_MED);
    }
}

void StrobeHandler::onButtonHoldRelease(long time) {
    setHandler(&offHandler);
}

void StrobeHandler::onHighTemperature() {
    setHandler(&offHandler);
}

#define MIN_INTERVAL 20
#define HOLD_INTERVAL 250
class Dispatcher {
public:
    Dispatcher(): _btnDown(false) {}

    void init() {
        _btnTime = millis();
        _btnDown = digitalRead(DPIN_RLED_SW);
    }

    void dispatchTemperatureEvents() {
        eventHandler->onHighTemperature();
    }

    void dispatchButtonEvents(long time) {
        byte newBtnDown = digitalRead(DPIN_RLED_SW);
        long buttonDownTime = (time - _btnTime);

        if(!_btnDown && newBtnDown) {
            eventHandler->onButtonDown();
        }
        else if(_btnDown && !newBtnDown && (buttonDownTime > MIN_INTERVAL)) {
            if(buttonDownTime > HOLD_INTERVAL)
                eventHandler->onButtonHoldRelease(buttonDownTime);
            else
                eventHandler->onButtonUp();
        }
        else if(_btnDown && newBtnDown && (buttonDownTime > HOLD_INTERVAL)) {
            eventHandler->onButtonHold(time-btnTime);
        }

        eventHandler->handle(time);

        // Remember button state so we can detect transitions
        if (newBtnDown != _btnDown) {
            _btnTime = time;
            _btnDown = newBtnDown;
            delay(50);
        }
    }

private:
    byte _btnDown;
    long _btnTime;
} dispatcher;

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

    btnTime = millis();

    eventHandler = &offHandler;
    eventHandler->init();
    dispatcher.init();

    Serial.println("Powered up!");
}

GreenLED::BlinkContext chargingLEDContext = hb.gled.blink();
GreenLED::OnContext chargedLEDContext = hb.gled.on();

HexBright::RedLED::BlinkContext temperatureLEDContext = hb.rled.blink();
HexBright::RedLED::BlinkContext batteryLEDContext = hb.rled.blink();

void loop()
{
    static unsigned long lastTempTime;

    unsigned long time = millis();
    Serial.println(time-oldTime);

    // Check the state of the charge controller
    int chargeState = getChargeState();
    read_avr_voltage(chargeState);

    if(chargeState == CHARGING) {
        chargingLEDContext.blink();
        chargedLEDContext.off();
    }
    else if(chargeState == CHARGED) {
        chargingLEDContext.stopBlink();
        chargedLEDContext.on();
    }
    else if(chargeState == BATTERY) {
        chargingLEDContext.stopBlink();
        chargedLEDContext.off();

        if(low_voltage_state()) {
            batteryLEDContext.blink();
        }
        else {
            batteryLEDContext.stopBlink();
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
            temperatureLEDContext.stopBlink();
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
