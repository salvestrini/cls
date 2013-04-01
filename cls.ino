// -*- c++ -*-

//
// Deliberately taken from:
//   http://dptnt.com/2010/03/smart-optical-slave-flash-trigger/
//

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define PIN_PHOTODIODE  0
#define PIN_XSYNC       7

#define PIN_LCD_SCLK    7 // Serial clock out    (SCLK)
#define PIN_LCD_DIN     6 // Serial data out     (DIN)
#define PIN_LCD_DC      5 // Data/command select (D/C)
#define PIN_LCD_CS      4 // LCD chip select     (CS)
#define PIN_LCD_RST     3 // LCD reset           (RST)

#define DEBUG 1

#if DEBUG
#define LDBG(FMT, ARG...) do { Serial.println(FMT, ##ARG); } while (0)
#else
#define LDBG(FMT, ARG...)
#endif

Adafruit_PCD8544 lcd = Adafruit_PCD8544(PIN_LCD_SCLK,
                                        PIN_LCD_DIN,
                                        PIN_LCD_DC,
                                        PIN_LCD_CS,
                                        PIN_LCD_RST);

// FIXME: virtual methods enlarge the binary size ...

class bitmap {
public:
        bitmap(const uint8_t * buffer,
               uint16_t        width,
               uint16_t        height,
               uint16_t        color) :
                buffer_(buffer),
                width_(width),
                height_(height),
                color_(color)
        { }

        virtual ~bitmap()
        { }

        const uint8_t * buffer()  { return buffer_; }
        uint16_t        width()   { return width_;  }
        uint16_t        height()  { return height_; }
        uint16_t        color()   { return color_;  }

private:
        const uint8_t * buffer_;
        uint16_t        width_;
        uint16_t        height_;
        uint16_t        color_;
};

class image : public bitmap {
public:
        image(const uint8_t * buffer,
              uint16_t        w,
              uint16_t        h) :
                x_(0), y_(0), bitmap(buffer, w, h, 1)
        { }
#if 0
        image(uint16_t        x,
              uint16_t        y, 
              const uint8_t * buffer,
              uint16_t        w,
              uint16_t        h) :
                x_(x), y_(y), bitmap(buffer, w, h, 1)
        { }
#endif
        virtual ~image()
        { }
        
        virtual void move(uint16_t x,
                          uint16_t y)
        { x_ = x; y_ = y; }

        virtual void draw()
        {
                lcd.drawBitmap(x_, y_,
                               buffer(), width(), height(),
                               color());
                lcd.display();
        }

private:
        uint16_t x_;
        uint16_t y_;
};

class icon : public image {
public:
        icon(const uint8_t * buffer,
             uint16_t        w,
             uint16_t        h) :
                image(buffer, w, h),
                visible_(true)
        { }
#if 0
        icon(uint16_t        x,
             uint16_t        y, 
             const uint8_t * buffer,
             uint16_t        w,
             uint16_t        h) :
                image(x, y, buffer, w, h),
                visible_(true)
        { }
#endif
        virtual ~icon()
        { }

        virtual bool changed() { return true;      }
        virtual bool visible() { return visible_;  }
        virtual void hide()    { visible_ = false; }
        virtual void show()    { visible_ = true;  }

        virtual void draw() {
                if (!visible_)
                        return;
                image::draw();
        }

private:
        bool visible_;
};

typedef enum {
        GROUP_A,
        GROUP_B
} group_t;

typedef enum {
        CHANNEL_1,
        CHANNEL_2,
        CHANNEL_3,
        CHANNEL_4
} channel_t;

typedef enum {
        MODE_DUMB,
        MODE_SMART
} mode_t;

// Fire the flash with a 15 microsecond pulse on the xsync terminal
void fire_flash()
{
        digitalWrite(PIN_XSYNC, HIGH);
        delayMicroseconds(15);
        digitalWrite(PIN_XSYNC, LOW);
}

// 4 microsecond precision timing using 16MHz Arduino
extern volatile unsigned long timer0_overflow_count;

unsigned long hpticks()
{ return (timer0_overflow_count << 8) + TCNT0; }

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// For timeing measurements
volatile unsigned long prev_pulse, time_passed;

int       ar0;              // Photodiode input
int       pulse_count;      // Pulse count
int       prev_pulse_count; // Pulses in previous pulse groups

mode_t    mode;
group_t   group;
channel_t channel;
int       threshold;

// UI items
icon ui_battery (NULL, 0, 0);
icon ui_mode    (NULL, 0, 0);
icon ui_power   (NULL, 0, 0);
icon ui_channel (NULL, 0, 0);

void setup()
{
#if DEBUG
        Serial.begin(57600);
#endif

        lcd.begin();
        lcd.setContrast(50);
        lcd.clearDisplay();

        lcd.setTextSize(1);
        lcd.setTextColor(BLACK);
        lcd.setTextWrap(false);

        lcd.setCursor(0, 0);
        lcd.println("Hello world");

        // Use internal reference voltage of 1.2v for ADC
        analogReference(INTERNAL);

        // Set prescale to 8 for much faster analog read
        // 16MHz/8 = 2MHz ADC clock speed. Each ADC conversion takes 13 cycles
        // So 2MHz/13 = 154KHz sampling rate
        // With other overhead considered, each analogRead takes ~10 us
        cbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);

        pinMode(PIN_PHOTODIODE, INPUT);
        pinMode(PIN_XSYNC,      OUTPUT);

        // I don't why. The first analog read always return 1023 so a dummy
        // read is added
        analogRead(PIN_PHOTODIODE);
        ar0 = analogRead(PIN_PHOTODIODE);

        // Values to be stored on flash memory (later on)
        threshold = 50;
        mode      = MODE_SMART;
        group     = GROUP_A;
        channel   = CHANNEL_1;

        ui_battery.move(0, 0);
        ui_mode.move(0, 0);
        ui_power.move(0, 0);
        ui_channel.move(0, 0);

#if DEBUG
        if (lcd.width() <
            (ui_battery.width() +
             ui_mode.width()    +
             ui_power.width()   +
             ui_channel.width()))
                LDBG("Display is too short ...");
#endif

        LDBG("Ready");
}

void loop_ui()
{
        // (Re-)Draw changed items
        if (ui_battery.changed())
                ui_battery.draw();
        if (ui_mode.changed())
                ui_mode.draw();
        if (ui_power.changed())
                ui_power.draw();
        if (ui_channel.changed())
                ui_channel.draw();
}

void loop_slave()
{
        int           new_ar, delta;
        unsigned long now;

        new_ar      = analogRead(PIN_PHOTODIODE);
        delta       = new_ar - ar0;
        now         = hpticks();
        time_passed = now - prev_pulse;

        // Pulses come in bundles. The minimal seperation seems to be around
        // 4000 us
        if (pulse_count > 0 && time_passed > 1000) {
                prev_pulse_count = pulse_count;
                pulse_count      = 0;
        }

        // Reset if > 125000 * 4 = 500ms
        if (time_passed > 125000) {
                pulse_count = 0;
                prev_pulse_count = 0;
                prev_pulse = now;
        }

        if (delta > threshold) {
                if (mode == MODE_DUMB) {
                        // Fires upon seeing any light pulse
                        fire_flash();
                } else {
                        // Smart mode

                        // If the delay since the previous pulse is longer than
                        // 10000*4 microseconds (40ms)...

                        // Checking the previous pulse group is needed to avoid
                        // mis-fire.
                        if ((time_passed > 10000) &&
                            (prev_pulse_count == 1 || prev_pulse_count > 5)) {
                                // Fire the flash now!
                                fire_flash();
                                
                                // Reset the pulse counter
                                pulse_count      = 0;
                                prev_pulse_count = 0;
                        } else {
                                // Get to the end of the pulse. Some pulses are
                                // specially wide.
                                while (analogRead(PIN_PHOTODIODE) > threshold);
                                
                                // Increase the pulse count
                                pulse_count++;
                                
                                // Keep track of the time a pulse is detected
                                prev_pulse = now;
                        }
                }
        } else {
                // Background. It should be zero most of the time
                ar0 = new_ar;
        }
}

void loop()
{
        loop_slave();
        loop_ui();
}
