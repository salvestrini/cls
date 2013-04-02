// -*- c++ -*-

//
// Deliberately taken from:
//   http://dptnt.com/2010/03/smart-optical-slave-flash-trigger/
//

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//
// Configurable values (to some extent ...)
//

// Pins allocation
#define PIN_PHOTODIODE      0
#define PIN_XSYNC           1

#define PIN_LED            13 // Internal LED

#define PIN_BUTTON_GROUP    8
#define PIN_BUTTON_CHANNEL  9

#define PIN_LCD_SCLK        7 // Serial clock out    (SCLK)
#define PIN_LCD_DIN         6 // Serial data out     (DIN)
#define PIN_LCD_DC          5 // Data/command select (D/C)
#define PIN_LCD_CS          4 // LCD chip select     (CS)
#define PIN_LCD_RST         3 // LCD reset           (RST)

// Miscellaneous
#define XSYNC_DELAY        15 // us

#define DEBUG               1

#if DEBUG
#define LDBG(FMT, ARG...) do { Serial.println(FMT, ##ARG); } while (0)
#define CDBG(FMT, ARG...) do { Serial.print(FMT, ##ARG);   } while (0)
#else
#define LDBG(FMT, ARG...)
#define CDBG(FMT, ARG...)
#endif

Adafruit_PCD8544 lcd = Adafruit_PCD8544(PIN_LCD_SCLK,
                                        PIN_LCD_DIN,
                                        PIN_LCD_DC,
                                        PIN_LCD_CS,
                                        PIN_LCD_RST);

// FIXME: virtual methods enlarge the binary size ...

typedef uint8_t pin_t;

class pin {
public:
        pin(pin_t p) :
                pin_(p)
        { }

        //virtual ~pin() { }

        virtual void setup() = 0;

protected:
        pin_t pin_;
};

class analog_pin_input : public pin {
public:
        analog_pin_input(pin_t p) :
                pin(p)
        { }

        //virtual ~digital_pin_input() { }

        virtual void setup()
        { pinMode(pin_, INPUT); }

        virtual int get()
        { return analogRead(pin_); }
};

class analog_pin_output : public pin {
public:
        analog_pin_output(pin_t p) :
                pin(p)
        { }

        //virtual ~digital_pin_output() { }

        virtual void setup()
        { pinMode(pin_, OUTPUT); }

        virtual void set(int value)
        { analogWrite(pin_, value); }
};

class digital_pin_input : public pin {
public:
        digital_pin_input(pin_t p) :
                pin(p)
        { }

        //virtual ~digital_pin_input() { }

        virtual void setup()
        { pinMode(pin_, INPUT); }

        virtual bool get()
        { return digitalRead(pin_) == HIGH ? true : false; }
};

class digital_pin_output : public pin {
public:
        digital_pin_output(pin_t p) :
                pin(p)
        { }

        //virtual ~digital_pin_output() { }

        virtual void setup()
        { pinMode(pin_, OUTPUT); }

        virtual void set(bool value)
        { digitalWrite(pin_, value ? HIGH : LOW); }
};

class button : private digital_pin_input {
public:
        button(pin_t  p,
               size_t presses_max      = 3,
               long   time_debounce    = 20,
               long   time_multi_click = 250,
               long   time_held_down   = 1000) :
                digital_pin_input(p),
                presses_max_      (presses_max),
                time_debounce_    (time_debounce),
                time_multi_click_ (time_multi_click),
                time_held_down_   (time_held_down)
        {
                clicks_           = 0;
                depressed_        = false;
                state_last_       = false;
                presses_current_  = 0;
                time_last_bounce_ = 0;
        }

        //virtual ~button() { }

        virtual void setup()
        { digital_pin_input::setup(); }

        virtual void update()
        {
                // Get current time && state
                long now   = (long) millis();
                bool state = get(); //digitalRead(pin_);

                // If the switch has changed, reset the debounce timer
                if (state != state_last_)
                        time_last_bounce_ = now;

                // Debounce the button
                if (now - time_last_bounce_ > time_debounce_ &&
                    state != depressed_) {
                        depressed_ = state;
                        
                        if (depressed_)
                                presses_current_++;

                        // Limit number of clicks / button presses
                        if (presses_current_ > presses_max_)
                                presses_current_ = presses_max_;
                }

                // If the button released state is stable, update the clicks
                // count and start a new cycle
                if (!depressed_ &&
                    (now - time_last_bounce_) > time_multi_click_) {
                        // positive count for released buttons
                        clicks_          = presses_current_;
                        presses_current_ = 0;
                }

                // Check button held-down time
                if (depressed_ &&
                    (now - time_last_bounce_ > time_held_down_) &&
                    clicks_ <= presses_max_) {
                        // negative count for held-down buttons
                        clicks_          = 0 - presses_current_;
                        presses_current_ = 0;
                }

                state_last_ = state;
        }

        virtual int clicks()
        { return clicks_; }

private:
        pin_t   pin_;

        int     clicks_;

        boolean depressed_;

        size_t  presses_current_;
        size_t  presses_max_;

        long    time_debounce_;
        long    time_multi_click_;
        long    time_held_down_;
        long    time_last_bounce_;

        boolean state_last_;
};

class led_static : private digital_pin_output {
public:
        led_static(pin_t p) :
                digital_pin_output(p),
                state_current_(false),
                state_previous_(false)
        { }

        //virtual ~led_static() { }

        virtual void setup()
        { digital_pin_output::setup(); }

        virtual void update()
        {
                if (state_current_ != state_previous_) {
                        state_previous_ = state_current_;
                        set(state_current_);
                }
        }

        virtual void on()
        {
                state_previous_ = state_current_;
                state_current_  = true;
        }

        virtual void off()
        {
                state_previous_ = state_current_;
                state_current_  = false;
        }
        
        virtual void flip()
        {
                state_previous_ = state_current_;
                state_current_  = !state_current_;
        }

private:
        bool state_current_;
        bool state_previous_;
};

class led_blinking : private led_static {
public:
        led_blinking(pin_t         p,
                     unsigned long time_on,
                     unsigned long time_off) :
                led_static(p),
                time_on_(time_on),
                time_off_(time_off)
        { }

        //virtual ~led_blinking() { }

        virtual void setup()
        { led_static::setup(); }

        virtual void change(unsigned long time_on,
                            unsigned long time_off)
        {
                time_on_  = time_on;
                time_off_ = time_off;
        }

        virtual void update()
        {
                unsigned long now = millis();
                if (now % (time_on_ + time_off_) < time_on_)
                        on();
                else
                        off();
                led_static::update();
        }

private:
        unsigned long time_on_;
        unsigned long time_off_;
};

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
        
        //virtual ~bitmap() { }

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
        //virtual ~image() { }

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
        //virtual ~icon() { }

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

class xsync : private digital_pin_output {
public:
        xsync(pin_t p, unsigned int delay_us) :
                digital_pin_output(p),
                delay_(delay_us)
        { }

        virtual void setup()
        { digital_pin_output::setup(); }

        virtual void fire()
        {
                set(true);
                delayMicroseconds(delay_);
                set(false);
        }

private:
        unsigned int delay_;
};

typedef enum {
        GROUP_A,
        GROUP_B,
        GROUP_NONE,
} group_t;

typedef enum {
        CHANNEL_1,
        CHANNEL_2,
        CHANNEL_3,
        CHANNEL_4
} channel_t;

typedef struct {
        group_t   group;
        channel_t channel;
} status_t;

status_t status_current;

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

icon             ui_battery (NULL, 0, 0);
icon             ui_mode    (NULL, 0, 0);
icon             ui_power   (NULL, 0, 0);
icon             ui_channel (NULL, 0, 0);

button           button_group(PIN_BUTTON_GROUP);
button           button_channel(PIN_BUTTON_CHANNEL);
led_blinking     led_status(PIN_LED, 10, 990);

xsync            flash(PIN_XSYNC, XSYNC_DELAY);

analog_pin_input photodiode(PIN_PHOTODIODE);

void lcd_update()
{

#if DEBUG
        CDBG("Current status -> ");

        CDBG("G=");
        switch (status_current.group) {
        case GROUP_A:    CDBG("A"); break;
        case GROUP_B:    CDBG("B"); break;
        case GROUP_NONE: CDBG("-"); break;
        default:                   break;
        }

        CDBG(" ");

        CDBG("C=");
        switch (status_current.channel) {
        case CHANNEL_1: CDBG("1"); break;
        case CHANNEL_2: CDBG("2"); break;
        case CHANNEL_3: CDBG("3"); break;
        case CHANNEL_4: CDBG("4"); break;
        default:                   break;
        }

        LDBG("");
#endif
}

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
        
        button_group.setup();
        button_channel.setup();
        flash.setup();
        led_status.setup();
        photodiode.setup();

        // The first analog read always return 1023 so a dummy read is added
        (void) photodiode.get();
        ar0 = photodiode.get();
        
        // Values to be stored on flash memory (later on)
        status_current.group   = GROUP_A;
        status_current.channel = CHANNEL_1;

        ui_battery.move (0, 0);
        ui_mode.move    (0, 0);
        ui_power.move   (0, 0);
        ui_channel.move (0, 0);
        
#if DEBUG
        if (lcd.width() <
            (ui_battery.width() +
             ui_mode.width()    +
             ui_power.width()   +
             ui_channel.width()))
                LDBG("Display is too short ...");
#endif

        LDBG("Ready");
        lcd_update();
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

        new_ar      = photodiode.get();
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

#if 0
        if (delta > threshold) {
                // If the delay since the previous pulse is longer than
                // 10000*4 microseconds (40ms)...
                        
                // Checking the previous pulse group is needed to avoid
                // mis-fire.
                if ((time_passed > 10000) &&
                    (prev_pulse_count == 1 || prev_pulse_count > 5)) {
                        // Fire the flash now!
                        flash.fire();
                        
                        // Reset the pulse counter
                        pulse_count      = 0;
                        prev_pulse_count = 0;
                } else {
                        // Get to the end of the pulse. Some pulses are
                        // specially wide.
                        while (photodiode.get() > threshold);
                        
                        // Increase the pulse count
                        pulse_count++;
                        
                        // Keep track of the time a pulse is detected
                        prev_pulse = now;
                }
        } else {
                // Background. It should be zero most of the time
                ar0 = new_ar;
        }
#endif
}

void spin_group(size_t count)
{
        for (size_t i = 0; i < count; i++) {
                switch (status_current.group) {
                case GROUP_A:    status_current.group = GROUP_B;    break;
                case GROUP_B:    status_current.group = GROUP_NONE; break;
                case GROUP_NONE: status_current.group = GROUP_A;    break;
                default:                                            break;
                }
        }
}

void spin_channel(size_t count)
{
        for (size_t i = 0; i < count; i++) {
                switch (status_current.channel) {
                case CHANNEL_1: status_current.channel = CHANNEL_2; break;
                case CHANNEL_2: status_current.channel = CHANNEL_3; break;
                case CHANNEL_3: status_current.channel = CHANNEL_4; break;
                case CHANNEL_4: status_current.channel = CHANNEL_1; break;
                default:                                            break;
                }
        }
}

void loop()
{
        delay(10);

        //loop_slave();
        //loop_ui();
        led_status.update();
        button_group.update();
        button_channel.update();

        spin_group(button_group.clicks());
        spin_channel(button_channel.clicks());

        bool something_changed =
                button_group.clicks() || button_channel.clicks();

        if (something_changed) {
                lcd_update();
        }
}
