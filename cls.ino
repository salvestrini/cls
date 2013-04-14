// -*- c++ -*-

//
// Deliberately taken from:
//   http://dptnt.com/2010/03/smart-optical-slave-flash-trigger/
//

//#include <EEPROM.h>
#include <avr/eeprom.h>

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

typedef struct configuration {
        group_t   group;
        channel_t channel;

        void reset()
        {
                group   = GROUP_A;
                channel = CHANNEL_1;
        }

        configuration()
        { reset(); }

        bool is_ok()
        {
                if ((group != GROUP_A)   &&
                    (group != GROUP_B)   &&
                    (group != GROUP_NONE))
                        return false;

                if ((channel != CHANNEL_1) &&
                    (channel != CHANNEL_2) &&
                    (channel != CHANNEL_3) &&
                    (channel != CHANNEL_4))
                        return false;

                return true;
        }
} configuration_t;

configuration_t configuration_current;

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
        switch (configuration_current.group) {
        case GROUP_A:    CDBG("A"); break;
        case GROUP_B:    CDBG("B"); break;
        case GROUP_NONE: CDBG("-"); break;
        default:                   break;
        }

        CDBG(" ");

        CDBG("C=");
        switch (configuration_current.channel) {
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

        //EEPROM_read<configuration_t>(0, configuration_current);
        eeprom_read_block((void *) &configuration_current,
                          (void *) 0,
                          sizeof(configuration_current));
        if (!configuration_current.is_ok()) {
                LDBG("Wrong EEPROM values, restoring defaults");
                configuration_current.reset();
        }

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

#define CODE_GROUP_A            0x9 // 1001
#define CODE_GROUP_B            0xA // 1010
#define CODE_GROUP_C            0xB // 1011
                               
#define CODE_CHANNEL_1         0xA4 // 10100100
#define CODE_CHANNEL_2         0x94 // 10010100
#define CODE_CHANNEL_3         0x92 // 10010010
#define CODE_CHANNEL_4         0x91 // 10010001
                               
#define CODE_COMMAND_OFF        0x8 // 1000
#define CODE_COMMAND_TTL        0x9 // 1001
#define CODE_COMMAND_AA         0xA // 1010
#define CODE_COMMAND_M          0xB // 1011

#define CODE_FLASH_POWER_1_1   0x01 // 00000001
#define CODE_FLASH_POWER_1_2   0x0D // 00001101
#define CODE_FLASH_POWER_1_4   0x19 // 00011001
#define CODE_FLASH_POWER_1_8   0x25 // 00100101
#define CODE_FLASH_POWER_1_16  0x31 // 00110001
#define CODE_FLASH_POWER_1_32  0x3D // 00111101
#define CODE_FLASH_POWER_1_64  0x49 // 01001001
#define CODE_FLASH_POWER_1_128 0x55 // 01010101

// Flash power #3 (FP sync)
//   CMD = <CHN>1101<mmmm><bbbbbbbb>[<bbbbbbbb>[<bbbbbbbb>]]
//
//     mmmm = 1011 if shutter > 1/500s
//     mmmm = 1100 if shutter < 1/500s
void decode_command_flash_power_3()
{ }

// Flash power #2 (Rear-curtain sync)
//   CMD = <CHN>0111<bbbbbbbb>[<bbbbbbbb>[<bbbbbbbb>]] + <CHN>0111
void decode_command_flash_power_2()
{ }

// Flash power #1
//   CMD = <CHN>0101<bbbbbbbb>[<bbbbbbbb>[<bbbbbbbb>]]
void decode_command_flash_power_1()
{ }

// Pre-flash
//   CMD = <CHN><gggg> + double-pulse
void decode_command_pre_flash()
{ }

// Remote configuration
//   CMD = <CHN>0110<gggg>[<gggg>[<gggg>]]
void decode_command_remote_group_setting()
{ }

void parse_command()
{
        // Parse group
}

void loop_slave()
{
}

void spin_group(size_t count)
{
        for (size_t i = 0; i < count; i++) {
                switch (configuration_current.group) {
                case GROUP_A:
                        configuration_current.group = GROUP_B;
                        break;
                case GROUP_B:
                        configuration_current.group = GROUP_NONE;
                        break;
                case GROUP_NONE:
                        configuration_current.group = GROUP_A;
                        break;
                default:
                        break;
                }
        }
}

void spin_channel(size_t count)
{
        for (size_t i = 0; i < count; i++) {
                switch (configuration_current.channel) {
                case CHANNEL_1: configuration_current.channel = CHANNEL_2;
                        break;
                case CHANNEL_2: configuration_current.channel = CHANNEL_3;
                        break;
                case CHANNEL_3: configuration_current.channel = CHANNEL_4;
                        break;
                case CHANNEL_4: configuration_current.channel = CHANNEL_1;
                        break;
                default:
                        break;
                }
        }
}

void loop()
{
        delay(10);

        //loop_ui();
        led_status.update();
        button_group.update();
        button_channel.update();

        int bg_clicks = button_group.clicks();
        int bc_clicks = button_channel.clicks();

        spin_group(bg_clicks);
        spin_channel(bc_clicks);

        if (bg_clicks || bc_clicks) {
                eeprom_write_block((void *) &configuration_current,
                                   (void *) 0,
                                   sizeof(configuration_current));
                lcd_update();
        }
}
