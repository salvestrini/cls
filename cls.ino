// -*- c++ -*-

//
// Deliberately taken from:
//   http://dptnt.com/2010/03/smart-optical-slave-flash-trigger/
//

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define DISPLAY_SCLK 7 // Serial clock out    (SCLK)
#define DISPLAY_DIN  6 // Serial data out     (DIN)
#define DISPLAY_DC   5 // Data/command select (D/C)
#define DISPLAY_CS   4 // LCD chip select     (CS)
#define DISPLAY_RST  3 // LCD reset           (RST)

Adafruit_PCD8544 display = Adafruit_PCD8544(DISPLAY_SCLK,
                                            DISPLAY_DIN,
                                            DISPLAY_DC,
                                            DISPLAY_CS,
                                            DISPLAY_RST);

// NOTE: virtual methods enlarge the binary size

class bitmap {
public:
        bitmap(const uint8_t * buffer,
               uint16_t        width,
               uint16_t        height) :
                buffer_(buffer),
                width_(width),
                height_(height)
        { }

        virtual ~bitmap()
        { }

        const uint8_t * buffer()  { return buffer_; }
        uint16_t        width()   { return width_;  }
        uint16_t        height()  { return height_; }

private:
        const uint8_t * buffer_;
        uint16_t        width_;
        uint16_t        height_;
};

class image : public bitmap {
public:
        image(uint16_t        x,
              uint16_t        y, 
              const uint8_t * buffer,
              uint16_t        w,
              uint16_t        h) :
                x_(x), y_(y), bitmap(buffer, w, h)
        { }

        virtual ~image()
        { }
        
        virtual void move(uint16_t x,
                          uint16_t y)
        { x_ = x; y_ = y; }

        virtual void draw(uint16_t color)
        { display.drawBitmap(x_, y_, buffer(), width(), height(), color); }

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
                image::draw(1);
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

// Arduino pins
#define PHOTODIODE  0
#define MODE        3
#define SENSITIVITY 4
#define XSYNC       7
#define BUTTON      8

// 4 microsecond precision timing using 16MHz Arduino
extern volatile unsigned long timer0_overflow_count;

// For timeing measurements
volatile unsigned long prev_pulse, time_passed;

int ar0;              // Photodiode input
int threshold;        // Pulse height threshold. DIP #2 off: 10, on: 5
int mode;             // DIP #1 off: 1st pulse, on: first pulse
int pulse_count;      // Pulse count
int prev_pulse_count; // Pulses in previous pulse groups

// Each tick is 4 microsecond
unsigned long hpticks()
{ return (timer0_overflow_count << 8) + TCNT0; }

// Fire the flash with a 15 microsecond pulse on the xsync terminal
void fire_flash()
{
        digitalWrite(XSYNC, HIGH);
        delayMicroseconds(15);
        digitalWrite(XSYNC, LOW);
}

void dip_config(int info)
{
        // DIP switch #1
        if (digitalRead(MODE) == HIGH)
                mode = 1; // Switch Off: smart mode
        else
                mode = 0; // Switch On: dumb mode

        // DIP switch #2
        if (digitalRead(SENSITIVITY) == HIGH)
                threshold = 50; // Switch off: low sensitivity
        else
                threshold = 15; // Switch on: high sensitivity

        if (info == 1) {
                Serial.print("Mode: ");
                Serial.print(mode);
                Serial.print(", Sensitivity: ");
                Serial.println(threshold);
        }
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

        pinMode(PHOTODIODE, INPUT);
        pinMode(MODE,       INPUT);
        digitalWrite(MODE, HIGH);        // Use the internal pull-up register
        pinMode(SENSITIVITY, INPUT);
        digitalWrite(SENSITIVITY, HIGH); // Use the internal pull-up register
        pinMode(XSYNC,  OUTPUT);
        pinMode(BUTTON, INPUT);

        // I don't why. The first analog read always return 1023 so a dummy
        // read is added
        analogRead(PHOTODIODE);
        ar0 = analogRead(PHOTODIODE);

        dip_config(1);

        Serial.println("Ready");
}

void loop()
{
        int           new_ar, delta;
        unsigned long now;

        new_ar = analogRead(PHOTODIODE);
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
                // Check the DIP switches. No serial xfer to save time.
                dip_config(0);
        }

        if (delta > threshold) {
                if (mode == 0) {
                        // Dumb mode. Fires upon seeing any light pulse
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
                                while (analogRead(PHOTODIODE) > threshold);
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

        // Read the button status
        if (digitalRead(BUTTON) == HIGH) {
                fire_flash();
                pulse_count      = 0;
                prev_pulse_count = 0;
                dip_config(1);

                // Avoid button contact problem so delay 300ms
                delay(300);
        }
}
