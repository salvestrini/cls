// -*- c++ -*-

//
// Deliberately taken from:
//   http://dptnt.com/2010/03/smart-optical-slave-flash-trigger/
//

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define PIN_PHOTODIODE  0
#define PIN_XSYNC       7

#define PIN_DISPLAY_SCLK 7 // Serial clock out    (SCLK)
#define PIN_DISPLAY_DIN  6 // Serial data out     (DIN)
#define PIN_DISPLAY_DC   5 // Data/command select (D/C)
#define PIN_DISPLAY_CS   4 // LCD chip select     (CS)
#define PIN_DISPLAY_RST  3 // LCD reset           (RST)

Adafruit_PCD8544 display = Adafruit_PCD8544(PIN_DISPLAY_SCLK,
                                            PIN_DISPLAY_DIN,
                                            PIN_DISPLAY_DC,
                                            PIN_DISPLAY_CS,
                                            PIN_DISPLAY_RST);

// NOTE: virtual methods enlarge the binary size

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
        image(uint16_t        x,
              uint16_t        y, 
              const uint8_t * buffer,
              uint16_t        w,
              uint16_t        h) :
                x_(x), y_(y), bitmap(buffer, w, h, 1)
        { }

        virtual ~image()
        { }
        
        virtual void move(uint16_t x,
                          uint16_t y)
        { x_ = x; y_ = y; }

        virtual void draw()
        { display.drawBitmap(x_, y_, buffer(), width(), height(), color()); }

private:
        uint16_t x_;
        uint16_t y_;
};

class icon : public image {
public:
        icon(uint16_t        x,
             uint16_t        y, 
             const uint8_t * buffer,
             uint16_t        w,
             uint16_t        h) :
                image(x, y, buffer, w, h),
                visible_(true)
        { }

        virtual ~icon()
        { }

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

icon battery (0,               0, NULL, 0, 0);
//icon mode    (battery.width(), 0, NULL, 0, 0);

typedef enum {
        GROUP_A,
        GROUP_B
} group_t;

group_t group;

typedef enum {
        CHANNEL_1,
        CHANNEL_2,
        CHANNEL_3,
        CHANNEL_4
} channel_t;

channel_t channel;

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// 4 microsecond precision timing using 16MHz Arduino
extern volatile unsigned long timer0_overflow_count;

// For timeing measurements
volatile unsigned long prev_pulse, time_passed;

int ar0;              // Photodiode input
int pulse_count;      // Pulse count
int prev_pulse_count; // Pulses in previous pulse groups

int threshold = 50;   // Pulse height threshold

typedef enum {
        MODE_DUMB,
        MODE_SMART
} mode_t;
mode_t mode = MODE_SMART;

// Each tick is 4 microsecond
unsigned long hpticks()
{ return (timer0_overflow_count << 8) + TCNT0; }

// Fire the flash with a 15 microsecond pulse on the xsync terminal
void fire_flash()
{
        digitalWrite(PIN_XSYNC, HIGH);
        delayMicroseconds(15);
        digitalWrite(PIN_XSYNC, LOW);
}

void setup()
{
        Serial.begin(57600);

        display.begin();
        display.setContrast(50);
        display.clearDisplay();

        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(0, 0);
        display.println("Hello world");

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

        Serial.println("Ready");
}

void loop()
{
        int           new_ar, delta;
        unsigned long now;

        new_ar = analogRead(PIN_PHOTODIODE);
        delta  = new_ar - ar0;

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
