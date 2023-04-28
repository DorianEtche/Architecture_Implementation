#include <Arduino.h>
#include "pwmwrite_arduino.h"
#include "tc_lib.h"
#include "pwm_lib.h"
using namespace arduino_due::pwm_lib;
//#define DEBUG_WITH_SERIAL2
// USE WHITE WIRE OF USB_TTL KEY ON PIN 16 OF ARDUINO
// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
#include "pwm_lib.h"
#include "tc_lib.h"
//#define PWM_DEFAULT_DUTY 500                      // hundredths of usecs (1e-8 secs)
//#define PWM_DEFAULT_PERIOD (2 * PWM_DEFAULT_DUTY) // hundredths of usecs  (1e-8 secs)=> 100 KHZ

#define TEST_WITH_VISUAL_STUDIO_CODE

#ifdef TEST_WITH_VISUAL_STUDIO_CODE
//#define USE_LOOP_SETUP
#define DIRECT_PIN_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)
struct s_pin
{
    // from library
    uint8_t arduino_pin; // arduino pin
    arduino_due::pwm_lib::pwm_pin pwm_pin;
    Pio *pio_p;          //= pio;
    uint32_t pin;        // = pio_pin;
    uint32_t id;         // = pio_id;
    EPioType type;       //= pio_type;
    uint32_t conf;       // = pio_conf;
    EPWMChannel channel; // = pwm_channel;
    bool inverted;       // = pwm_inverted;
    bool started;
    uint32_t clock;
    uint32_t period;

    uint32_t duty;
};
bool my_set_duty(struct s_pin &s, uint32_t duty /* 1e-8 secs. */)
{
#ifdef DEBUG_WITH_SERIAL2
    Serial2.print("pin :");
    Serial2.print(s.arduino_pin);
#endif

    if ((!s.started) || (duty > s.period))
    {
#ifdef DEBUG_WITH_SERIAL2
        Serial2.print(" , aborting  :duty =");
        Serial2.print(s.duty);
        Serial2.print(", period =");
        Serial2.println(s.period);
#endif

        return false;
    }

    s.duty = duty;
#ifdef DEBUG_WITH_SERIAL2
    Serial2.print(" , applying duty= ");
    Serial2.println(s.duty);
#endif

    //PWMC_SetDutyCycle(
    pwm_core::pwmc_setdutycycle(
        PWM_INTERFACE,
        s.channel,
        static_cast<uint32_t>(
            (static_cast<double>(duty) / 100000000) /
            pwm_core::tick_times[s.clock]));

    return true;
}

void my_pin_init(struct s_pin &s, arduino_due::pwm_lib::pwm_pin pwmpin, Pio *pio_p, uint32_t pin, uint32_t id, EPioType type, uint32_t conf, EPWMChannel channel, bool inverted)
{
    s.channel = channel;
    s.conf = conf;
    s.id = id;
    s.inverted = inverted;
    s.pin = pin;
    s.pio_p = pio_p;
    s.pwm_pin = pwmpin;
    s.type = type;
    s.started = false;
}
bool my_pwm_start(
    struct s_pin &s,
    uint32_t period, // hundredths of usecs (1e-8 secs.)
    uint32_t duty    // hundredths of usecs (1e-8 secs.)

)
{
    uint32_t clock;
    bool ok1 = !s.started;
    bool ok2 = pwm_core::find_clock(period, clock);
    s.duty = duty;
    s.period = period;
    s.clock = clock;
#ifdef DEBUG_WITH_SERIAL2
    Serial2.print("Starting Pwm for pin:");
    Serial2.print(s.arduino_pin);
    Serial2.print(", period:");
    Serial2.print(s.period);
    Serial2.print(", duty:");
    Serial2.print(s.duty);

#endif
    if (!ok1 || !ok2)
    {
#ifdef DEBUG_WITH_SERIAL2

        Serial2.println(", Aborted : problem");
#endif
        return false;
    }
#ifdef DEBUG_WITH_SERIAL2

    Serial2.print(", Success");
#endif
    pmc_enable_periph_clk(PWM_INTERFACE_ID);

    // we are not using clkA and clkB
    //PWMC_ConfigureClocks(0,0,VARIANT_MCK);

    // configuring the pwm pin
    PIO_Configure(
        s.pio_p,
        s.type,
        s.pin,
        s.conf);

    PWMC_ConfigureChannelExt(
        PWM_INTERFACE,
        s.channel,
        pwm_core::clock_masks[clock],
        0,                           // left aligned
        (s.inverted) ? 0 : (1 << 9), // polarity
        0,                           // interrupt on counter event at end's period
        0,                           // dead-time disabled
        0,                           // non inverted dead-time high output
        0                            // non inverted dead-time low output
    );

    PWMC_SetPeriod(
        PWM_INTERFACE,
        s.channel,
        static_cast<uint32_t>(
            (static_cast<double>(period) / 100000000) /
            pwm_core::tick_times[clock]));

    PWMC_EnableChannel(PWM_INTERFACE, s.channel);

    //PWMC_SetDutyCycle(
    pwm_core::pwmc_setdutycycle(
        PWM_INTERFACE,
        s.channel,
        static_cast<uint32_t>(
            (static_cast<double>(s.duty) / 100000000) /
            pwm_core::tick_times[clock]));
    s.started = true;
    return true;
} //my_pwm_start
void my_pwm_stop(
    struct s_pin &s)
{
    PWMC_DisableChannel(
        PWM_INTERFACE,
        s.channel);

    while (
        (PWM->PWM_SR & (1 << s.channel)) != 0)
    { /* nothing */
    }
    digitalWrite(s.arduino_pin, 0); // idem : not sure this is a good idea
    s.started = false;
} // my_pwm_stop
typedef struct
{

    //uint8_T pinA, pinB;
    struct s_pin pin_on, pin_off;
    uint8_T useComplementaryPin;

} Pwm_state_t;
#define MAX_NB_PWM 2
uint8_t pwm_is_active[MAX_NB_PWM] = {0, 0};
Pwm_state_t pwms[MAX_NB_PWM];
// Create an pwm object
extern "C" void pwmSetup(uint8_T pinA, uint8_T useComplementaryPin, float period_us, float duty_us, uint8_T id)
{
#ifdef DEBUG_WITH_SERIAL2
    Serial2.begin(115200);
    delay(100); // wait 100ms
#endif
    uint32_t period = (uint32_t)(period_us * 100);
    uint32_t duty = (uint32_t)(duty_us * 100);
    uint32_t id32 = id;
    // better to correctly initialize state
    pwms[id].pin_on.arduino_pin = pinA;
    switch (pinA)
    {
        // CH0 prefered pins
    case 34:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH0-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |

        my_pin_init(pwms[id].pin_on, pwm_pin::PWML0_PC2, PIOC, PIO_PC2B_PWML0, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH0, true);
        pwms[id].pin_off.arduino_pin = 35;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWMH0_PC3, PIOC, PIO_PC3B_PWMH0, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH0, false);
        }
        pwm_is_active[id] = 1;
        break;

    case 35:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWMH0_PC3, PIOC, PIO_PC3B_PWMH0, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH0, false);
        pwms[id].pin_off.arduino_pin = 34;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWML0_PC2, PIOC, PIO_PC2B_PWML0, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH0, true);
        }
        pwm_is_active[id] = 1;
        break;
        // CH1 prefered pins
    case 36:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML1_PC4, PIOC, PIO_PC4B_PWML1, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH1, true);
        pwms[id].pin_off.arduino_pin = 37;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWMH1_PC5, PIOC, PIO_PC5B_PWMH1, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH1, false);
        }
        pwm_is_active[id] = 1;
        break;

    case 37:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH0-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWMH1_PC5, PIOC, PIO_PC5B_PWMH1, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH1, false);
        pwms[id].pin_off.arduino_pin = 36;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWML1_PC4, PIOC, PIO_PC4B_PWML1, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH1, true);
        }
        pwm_is_active[id] = 1;
        break;
        // CH2 prefered pins
    case 38:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML2_PC6, PIOC, PIO_PC6B_PWML2, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH2, true);
        pwms[id].pin_off.arduino_pin = 39;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWMH2_PC7, PIOC, PIO_PC7B_PWMH2, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH2, false);
        }
        pwm_is_active[id] = 1;
        break;

    case 39:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH0-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWMH2_PC7, PIOC, PIO_PC7B_PWMH2, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH2, false);
        pwms[id].pin_off.arduino_pin = 38;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWML2_PC6, PIOC, PIO_PC6B_PWML2, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH2, true);
        }
        pwm_is_active[id] = 1;
        break;
        // CH3 prefered pins
    case 40:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML3_PC8, PIOC, PIO_PC8B_PWML3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH3, true);
        pwms[id].pin_off.arduino_pin = 41;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWMH3_PC9, PIOC, PIO_PC9B_PWMH3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH3, false);
        }
        pwm_is_active[id] = 1;
        break;

    case 41:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH4-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWMH3_PC9, PIOC, PIO_PC9B_PWMH3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH3, false);
        pwms[id].pin_off.arduino_pin = 40;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWML3_PC8, PIOC, PIO_PC8B_PWML3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH3, true);
        }
        pwm_is_active[id] = 1;
        break;
        // CH5 PREFERRED PINS
    case 8:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML5_PC22, PIOC, PIO_PC22B_PWML5, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH5, true);

        my_pin_init(pwms[id].pin_on, pwm_pin::PWML3_PC8, PIOC, PIO_PC8B_PWML3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH3, true);
        pwms[id].pin_off.arduino_pin = 44;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWMH5_PC19, PIOC, PIO_PC19B_PWMH5, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH5, false);
        }
        pwm_is_active[id] = 1;
        break;

    case 44:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWMH5_PC19, PIOC, PIO_PC19B_PWMH5, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH5, false);
        pwms[id].pin_off.arduino_pin = 8;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWML3_PC8, PIOC, PIO_PC8B_PWML3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH3, true);
        }
        pwm_is_active[id] = 1;
        break;
        // CH5 PREFERRED PINS
    case 7:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML6_PC23, PIOC, PIO_PC23B_PWML6, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH6, true);
        pwms[id].pin_off.arduino_pin = 45;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWMH6_PC18, PIOC, PIO_PC18B_PWMH6, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH6, false);
        }
        pwm_is_active[id] = 1;
        break;

    case 45:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWMH6_PC18, PIOC, PIO_PC18B_PWMH6, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH6, false);
        pwms[id].pin_off.arduino_pin = 7;
        pwms[id].useComplementaryPin = useComplementaryPin;
        if (useComplementaryPin)
        {
            my_pin_init(pwms[id].pin_off, pwm_pin::PWML6_PC23, PIOC, PIO_PC23B_PWML6, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH6, true);
        }
        pwm_is_active[id] = 1;
        break;
    // CH7 => ONLY ONE PIN
    case 6:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML7_PC24, PIOC, PIO_PC24B_PWML7, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH7, true);
        pwms[id].pin_off.arduino_pin = 0;
        pwms[id].useComplementaryPin = false; // no complementary pin for this pin
        pwm_is_active[id] = 1;
        break;
        // CH4 => ONLY ONE PIN
    case 9:
        // ---------------------------------------------------------------------------------------------------------------------------
        // --------PWM_CH-------|    the_pwm_pin    | pio |     pio_pin    | per_id |  pio_type   |    conf    | channel | inverted |
        my_pin_init(pwms[id].pin_on, pwm_pin::PWML4_PC21, PIOC, PIO_PC21B_PWML4, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PWM_CH4, true);
        pwms[id].pin_off.arduino_pin = 0;
        pwms[id].useComplementaryPin = false; // no complementary pin for this pin
        pwm_is_active[id] = 1;
        break;
    default:
        pwm_is_active[id] = 0;
    }
    //---------------------------------
    // common post activation tasks
    //---------------------------------
    if (pwm_is_active[id] == 1)
    {
        pinMode(pwms[id].pin_on.arduino_pin, OUTPUT);
        pwms[id].pin_on.started = false;
        bool ok = my_pwm_start(pwms[id].pin_on, period, duty);
        if (pwms[id].useComplementaryPin == 1)
        {
            pwms[id].pin_off.started = false;
            pinMode(pwms[id].pin_off.arduino_pin, OUTPUT);
            ok = my_pwm_start(pwms[id].pin_off, period, duty);
        }
    }
}
// Read the position relative to the old position
// and reset the position to zero

extern "C" void pwmWrite(float value, uint8_T id)
{
    uint32_T duty;
    uint32_T demi_period;
    if (!pwm_is_active[id])
    {
        return;
    }
    value = (value > 1) ? 1 : (value < -1) ? -1 : value;
    demi_period = pwms[id].pin_on.period / 2;
    value = (demi_period - 1) * value + demi_period;
    duty = (uint32_T)value;
    bool ok = my_set_duty(pwms[id].pin_on, duty);
    if (pwms[id].useComplementaryPin == 1)
    {
        ok = my_set_duty((pwms[id].pin_off), duty);
    }
}

extern "C" void pwmWriteDutyMicros(float duty_us, uint8_T id)
{
#ifdef DEBUG_WITH_SERIAL2
    Serial2.print(" Writing duty(us) =");
    Serial2.print((uint32_t)duty_us);
    Serial2.print(" for id = ");
    Serial2.print(id);

#endif
    if (!pwm_is_active[id])
    {
#ifdef DEBUG_WITH_SERIAL2
        Serial2.println(",inactive => abort ");
#endif
        return;
    }
    uint32_t duty = (uint32_t)(duty_us * 100);
#ifdef DEBUG_WITH_SERIAL2
    Serial2.println(", ok:call my_set_duty ");
#endif
    bool ok = my_set_duty(pwms[id].pin_on, duty);
    if (pwms[id].useComplementaryPin == 1)
    {
        ok = my_set_duty((pwms[id].pin_off), duty);
    }
}

extern "C" void pwmStop(uint8_T id)
{
    if (!pwm_is_active[id])
    {
        return;
    }
    my_pwm_stop(pwms[id].pin_on);
    if (pwms[id].useComplementaryPin)
    {
        my_pwm_stop(pwms[id].pin_off);
    }
    pwm_is_active[id] = 0;
}
#ifdef USE_LOOP_SETUP
// unused by matlab
void setup()
{
    Serial.begin(115200);
    pwmSetup(6, 0, 20000.0, 100.0, 0);
    // pwmSetup(34, 0, 100.0,10.0,  1);
    // unused1
}

void loop()
{
    float value;
    while (1)
    {
        value = 100;

        for (int id = 0; id < MAX_NB_PWM; id++)
        {
            pwmWriteDutyMicros(value, id);
        }
        Serial.print(" period=10ns. ");
        Serial.print(pwms[0].pin_on.period);
        Serial.print(",duty:10ns . ");
        Serial.print(pwms[0].pin_on.duty);
        Serial.print(",value:");
        Serial.println(value);
        delay(1000);
    }
    // unused
}
#endif
#endif // #ifndef TEST_WITH_VISUAL_STUDIO_CODE
