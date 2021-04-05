#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include "api.h"

MODULE_AUTHOR("MX_Master");
MODULE_DESCRIPTION("ARISC driver for the Allwinner ARISC firmware");
MODULE_LICENSE("GPL");




static int32_t comp_id;
static const uint8_t * comp_name = "arisc";

static int8_t *in = "";
RTAPI_MP_STRING(in, "input pins, comma separated");

static int8_t *out = "";
RTAPI_MP_STRING(out, "output pins, comma separated");

static char *ctrl_type = "";
RTAPI_MP_STRING(ctrl_type, "channels control type, comma separated");

static const char *gpio_name[GPIO_PORTS_MAX_CNT] =
    {"PA","PB","PC","PD","PE","PF","PG","PL"};

static hal_bit_t **gpio_hal_0[GPIO_PORTS_MAX_CNT];
static hal_bit_t **gpio_hal_1[GPIO_PORTS_MAX_CNT];
static hal_bit_t gpio_hal_0_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];
static hal_bit_t gpio_hal_1_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static hal_s32_t **gpio_hal_pull[GPIO_PORTS_MAX_CNT];
static hal_s32_t gpio_hal_pull_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static hal_u32_t **gpio_hal_drive[GPIO_PORTS_MAX_CNT];
static hal_u32_t gpio_hal_drive_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static uint32_t gpio_out_mask[GPIO_PORTS_MAX_CNT] = {0};
static uint32_t gpio_in_mask[GPIO_PORTS_MAX_CNT] = {0};

static uint32_t gpio_in_cnt = 0;
static uint32_t gpio_out_cnt = 0;
static uint32_t gpio_ports_cnt = 0;
static uint32_t gpio_pins_cnt[GPIO_PINS_MAX_CNT] = {0};

static uint32_t pin_msk[GPIO_PINS_MAX_CNT] = {0};

typedef struct
{
    hal_bit_t *enable; // in

    hal_u32_t *pwm_port; // in
    hal_u32_t *pwm_pin; // in
    hal_bit_t *pwm_inv; // in
    hal_u32_t *pwm_duty; // in

    hal_u32_t *dir_port; // in
    hal_u32_t *dir_pin; // in
    hal_bit_t *dir_inv; // in
    hal_u32_t *dir_hold; // in
    hal_u32_t *dir_setup; // in

    hal_float_t *pos_scale; // in
    hal_float_t *pos_cmd; // in
    hal_float_t *vel_cmd; // in

    hal_s32_t *counts; // out
    hal_float_t *pos_fb; // out
    hal_float_t *freq; // out
} pwm_ch_shmem_t;

typedef struct
{
    hal_bit_t ctrl_type;

    hal_s32_t pwm_pos_cmd;
    hal_u32_t pwm_port_old;
    hal_u32_t pwm_pin_old;
    hal_bit_t pwm_inv_old;

    hal_u32_t dir;
    hal_u32_t dir_port_old;
    hal_u32_t dir_pin_old;
    hal_bit_t dir_inv_old;
} pwm_ch_priv_t;

static pwm_ch_shmem_t *pwmh;
static pwm_ch_priv_t pwmp[PWM_CH_MAX_CNT] = {0};
static uint8_t pwm_cnt = 0;

#define gh *pwmh[ch]
#define ggh pwmh[ch]
#define gp pwmp[ch]




// TOOLS

static void comp_write(void *arg, long period);
static void comp_read(void *arg, long period);

static inline
int32_t malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int8_t* arg_str[2] = {in, out};
    int8_t n;
    uint8_t port;
    int32_t r, ch;
    int8_t *data = ctrl_type, *token, type[PWM_CH_MAX_CNT] = {0};
    char name[HAL_NAME_LEN + 1];

    // init some GPIO vars
    for ( n = GPIO_PINS_MAX_CNT; n--; ) pin_msk[n] = 1UL << n;

    // shared memory allocation for GPIO
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        gpio_hal_0[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_1[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_pull[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_s32_t *));
        gpio_hal_drive[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_u32_t *));

        if ( !gpio_hal_0[port] || !gpio_hal_1[port] ||
             !gpio_hal_pull[port] || !gpio_hal_drive[port] )
        {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "%s.gpio: port %s hal_malloc() failed \n",
                comp_name, gpio_name[port]);
            return -1;
        }
    }

    // export GPIO HAL pins
    for ( n = 2; n--; )
    {
        if ( !arg_str[n] ) continue;

        int8_t *data = arg_str[n], *token;
        uint8_t pin, found;
        int32_t retval;
        int8_t* type_str = n ? "out" : "in";

        while ( (token = strtok(data, ",")) != NULL )
        {
            if ( data != NULL ) data = NULL;
            if ( strlen(token) < 3 ) continue;

            // trying to find a correct port name
            for ( found = 0, port = GPIO_PORTS_MAX_CNT; port--; )
            {
                if ( 0 == memcmp(token, gpio_name[port], 2) )
                {
                    found = 1;
                    break;
                }
            }

            if ( !found ) continue;

            // trying to find a correct pin number
            pin = (uint8_t) strtoul(&token[2], NULL, 10);

            if ( (pin == 0 && token[2] != '0') || pin >= GPIO_PINS_MAX_CNT ) continue;

            // export pin function
            retval = hal_pin_bit_newf( (n ? HAL_IN : HAL_OUT),
                &gpio_hal_0[port][pin], comp_id,
                "%s.gpio.%s-%s", comp_name, token, type_str);

            // export pin inverted function
            retval += hal_pin_bit_newf( (n ? HAL_IN : HAL_OUT),
                &gpio_hal_1[port][pin], comp_id,
                "%s.gpio.%s-%s-not", comp_name, token, type_str);

            // export pin pull up/down function
            retval += hal_pin_s32_newf( HAL_IN,
                &gpio_hal_pull[port][pin], comp_id,
                "%s.gpio.%s-pull", comp_name, token);

            // export pin multi-drive (open drain) function
            retval += hal_pin_u32_newf( HAL_IN,
                &gpio_hal_drive[port][pin], comp_id,
                "%s.gpio.%s-multi-drive-level", comp_name, token);

            if (retval < 0)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s.gpio: pin %s export failed \n",
                    comp_name, token);
                return -1;
            }

            // configure GPIO pin
            if ( n )
            {
                gpio_out_cnt++;
                gpio_out_mask[port] |= pin_msk[pin];
                gpio_pin_func_set(port, pin, GPIO_FUNC_OUT, 0);
            }
            else
            {
                gpio_in_cnt++;
                gpio_in_mask[port] |= pin_msk[pin];
                gpio_pin_func_set(port, pin, GPIO_FUNC_IN, 0);
            }

            // disable pull up/down
            gpio_pin_pull_set(port, pin, GPIO_PULL_DISABLE, 0);

            // get/set pin init state
            *gpio_hal_0[port][pin] = gpio_pin_get(port, pin, 0);
            *gpio_hal_1[port][pin] = *gpio_hal_0[port][pin] ? 0 : 1;
            gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
            gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];

            // get pin pull up/down state
            switch ( gpio_pin_pull_get(port, pin, 0) )
            {
                case GPIO_PULL_UP:      *gpio_hal_pull[port][pin] = 1;
                case GPIO_PULL_DOWN:    *gpio_hal_pull[port][pin] = -1;
                default:                *gpio_hal_pull[port][pin] = 0;
            }
            gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];

            // get pin multi-drive (open drain) state
            *gpio_hal_drive[port][pin] = gpio_pin_multi_drive_get(port, pin, 0);
            gpio_hal_drive_prev[port][pin] = *gpio_hal_drive[port][pin];

            // used ports count update
            if ( port >= gpio_ports_cnt ) gpio_ports_cnt = port + 1;
            // used port pins count update
            if ( pin >= gpio_pins_cnt[port] ) gpio_pins_cnt[port] = pin + 1;
        }

    }

    // get PWM channels count and type
    while ( (token = strtok(data, ",")) != NULL )
    {
        if ( data != NULL ) data = NULL;

        if      ( token[0] == 'P' || token[0] == 'p' ) type[pwm_cnt++] = 0;
        else if ( token[0] == 'V' || token[0] == 'v' ) type[pwm_cnt++] = 1;
    }
    if ( !pwm_cnt ) return 0;
    if ( pwm_cnt > PWM_CH_MAX_CNT ) pwm_cnt = PWM_CH_MAX_CNT;

    // shared memory allocation for PWM
    pwmh = hal_malloc(pwm_cnt * sizeof(pwm_ch_shmem_t));
    if ( !pwmh ) PRINT_ERROR_AND_RETURN("hal_malloc() failed", -1);

    // export PWM HAL pins and set default values
    #define EXPORT_PIN(IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
        r += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(ggh.VAL), comp_id,\
        "%s.%d." NAME, comp_name, ch);\
        gh.VAL = DEFAULT;

    for ( r = 0, ch = pwm_cnt; ch--; )
    {
        EXPORT_PIN(HAL_IN,bit,enable,"enable", 0);
        EXPORT_PIN(HAL_IN,u32,pwm_port,"pwm-port", UINT32_MAX);
        EXPORT_PIN(HAL_IN,u32,pwm_pin,"pwm-pin", UINT32_MAX);
        EXPORT_PIN(HAL_IN,bit,pwm_inv,"pwm-invert", 0);
        EXPORT_PIN(HAL_IN,u32,pwm_duty,"pwm-duty", UINT32_MAX/2);
        EXPORT_PIN(HAL_IN,u32,dir_port,"dir-port", UINT32_MAX);
        EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-pin", UINT32_MAX);
        EXPORT_PIN(HAL_IN,bit,dir_inv,"dir-invert", 0);
        EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-hold", 50000);
        EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-setup", 50000);
        EXPORT_PIN(HAL_IN,float,pos_scale,"position-scale", 1.0);
        EXPORT_PIN(HAL_OUT,float,pos_fb,"position-fb", 0.0);
        EXPORT_PIN(HAL_OUT,float,freq,"frequency", 0.0);
        EXPORT_PIN(HAL_OUT,s32,counts,"counts", 0);

        if ( type[ch] ) { EXPORT_PIN(HAL_IN,float,vel_cmd,"velocity-cmd", 0.0); }
        else            { EXPORT_PIN(HAL_IN,float,pos_cmd,"position-cmd", 0.0); }

        gp.ctrl_type = type[ch];

        gp.pwm_pos_cmd = 0;
        gp.pwm_inv_old = 0;
        gp.pwm_pin_old = UINT32_MAX;
        gp.pwm_port_old = UINT32_MAX;

        gp.dir_inv_old = 0;
        gp.dir_pin_old = UINT32_MAX;
        gp.dir_port_old = UINT32_MAX;
    }
    if ( r )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s.pwm: HAL pins export failed\n", comp_name);
        return -1;
    }

    #undef EXPORT_PIN

    // export HAL functions
    r = 0;
    rtapi_snprintf(name, sizeof(name), "%s.write", comp_name);
    r += hal_export_funct(name, gpio_write, 0, 0, 0, comp_id);
    rtapi_snprintf(name, sizeof(name), "%s.read", comp_name);
    r += hal_export_funct(name, gpio_read, 0, 0, 0, comp_id);
    if ( r )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: HAL functions export failed\n", comp_name);
        return -1;
    }

    return 0;
}




// HAL functions

static inline
void gpio_read(void *arg, long period)
{
    static uint32_t port, pin, port_state;

    if ( !gpio_in_cnt ) return;

    for ( port = gpio_ports_cnt; port--; )
    {
        if ( !gpio_in_mask[port] ) continue;

        port_state = gpio_port_get(port, 0);

        for ( pin = gpio_pins_cnt[port]; pin--; )
        {
            if ( !(gpio_in_mask[port] & pin_msk[pin]) ) continue;

            if ( port_state & pin_msk[pin] )
            {
                *gpio_hal_0[port][pin] = 1;
                *gpio_hal_1[port][pin] = 0;
            }
            else
            {
                *gpio_hal_0[port][pin] = 0;
                *gpio_hal_1[port][pin] = 1;
            }
        }
    }
}

static inline
void gpio_write(void *arg, long period)
{
    static uint32_t port, pin, mask_0, mask_1;

    if ( !gpio_in_cnt && !gpio_out_cnt ) return;

    for ( port = gpio_ports_cnt; port--; )
    {
        if ( !gpio_in_mask[port] && !gpio_out_mask[port] ) continue;

        mask_0 = 0;
        mask_1 = 0;

        for ( pin = gpio_pins_cnt[port]; pin--; )
        {
            if ( !(gpio_in_mask[port] & pin_msk[pin]) &&
                 !(gpio_out_mask[port] & pin_msk[pin]) ) continue;

            // set pin pull up/down state
            if ( gpio_hal_pull_prev[port][pin] != *gpio_hal_pull[port][pin] )
            {
                if ( *gpio_hal_pull[port][pin] > 0 )
                {
                    *gpio_hal_pull[port][pin] = 1;
                    gpio_pin_pull_set(port, pin, GPIO_PULL_UP, 0);
                }
                else if ( *gpio_hal_pull[port][pin] < 0 )
                {
                    *gpio_hal_pull[port][pin] = -1;
                    gpio_pin_pull_set(port, pin, GPIO_PULL_DOWN, 0);
                }
                else gpio_pin_pull_set(port, pin, GPIO_PULL_DISABLE, 0);
                gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];
            }

            // set pin multi-drive (open drain) state
            if ( gpio_hal_drive_prev[port][pin] != *gpio_hal_drive[port][pin] )
            {
                *gpio_hal_drive[port][pin] &= (GPIO_PULL_CNT - 1);
                gpio_pin_multi_drive_set(port, pin, *gpio_hal_drive[port][pin], 0);
                gpio_hal_drive_prev[port][pin] = *gpio_hal_drive[port][pin];
            }

            if ( !(gpio_out_mask[port] & pin_msk[pin]) ) continue;

            if ( *gpio_hal_0[port][pin] != gpio_hal_0_prev[port][pin] )
            {
                if ( *gpio_hal_0[port][pin] )
                {
                    *gpio_hal_1[port][pin] = 0;
                    mask_1 |= pin_msk[pin];
                }
                else
                {
                    *gpio_hal_1[port][pin] = 1;
                    mask_0 |= pin_msk[pin];
                }
                gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
                gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];
            }

            if ( *gpio_hal_1[port][pin] != gpio_hal_1_prev[port][pin] )
            {
                if ( *gpio_hal_1[port][pin] )
                {
                    *gpio_hal_0[port][pin] = 0;
                    mask_0 |= pin_msk[pin];
                }
                else
                {
                    *gpio_hal_0[port][pin] = 1;
                    mask_1 |= pin_msk[pin];
                }
                gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];
                gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
            }
        }

        if ( mask_0 ) gpio_port_clr(port, mask_0, 0);
        if ( mask_1 ) gpio_port_set(port, mask_1, 0);
    }
}

static inline
void pwm_update_pos_scale(uint8_t ch)
{
    if ( gh.pos_scale < 0 || (gh.pos_scale < 1e-20 && gh.pos_scale > -1e-20) )
    {
        gh.pos_scale = 1.0;
    }
}

static inline
void pwm_update_pins(uint8_t ch)
{
    if ( gh.pwm_port != gp.pwm_port_old ||
         gh.pwm_pin  != gp.pwm_pin_old ||
         gh.pwm_inv  != gp.pwm_inv_old ||
         gh.dir_port != gp.dir_port_old ||
         gh.dir_pin  != gp.dir_pin_old ||
         gh.dir_inv  != gp.dir_inv_old ) return 0;

    if ( gh.pwm_port < GPIO_PORTS_MAX_CNT && gh.pwm_pin < GPIO_PINS_MAX_CNT )
    {
        gp.pwm_port_old = gh.pwm_port;
        gp.pwm_pin_old  = gh.pwm_pin;
        gp.pwm_inv_old  = gh.pwm_inv;
    }
    else
    {
        gh.pwm_port = gp.pwm_port_old;
        gh.pwm_pin = gp.pwm_pin_old;
        gh.pwm_inv = gp.pwm_inv_old;
    }

    if ( gh.dir_port < GPIO_PORTS_MAX_CNT && gh.dir_pin < GPIO_PINS_MAX_CNT )
    {
        gp.dir_port_old = gh.dir_port;
        gp.dir_pin_old  = gh.dir_pin;
        gp.dir_inv_old  = gh.dir_inv;
    }
    else
    {
        gh.dir_port = gp.dir_port_old;
        gh.dir_pin = gp.dir_pin_old;
        gh.dir_inv = gp.dir_inv_old;
    }

    pwm_ch_pins_setup(ch,
        gh.pwm_port, gh.pwm_pin, gh.pwm_inv,
        gh.dir_port, gh.dir_pin, gh.dir_inv,
        0);
}

static
void pwm_read(void *arg, long period)
{
    static uint8_t ch;
    static hal_s32_t pos_cmd_counts;

    for ( ch = pwm_cnt; ch--; )
    {
        if ( !gh.enable ) continue;

        pwm_update_pos_scale(ch);
        gh.counts = pwm_ch_pos_get(ch, 0);
        gh.pos_fb = ((hal_float_t)gh.counts) / gh.pos_scale;

        if ( !gp.ctrl_type ) // position mode
        {
            pos_cmd_counts = (hal_s32_t) round(gh.pos_scale * gh.pos_cmd);
            if ( pos_cmd_counts == gh.counts ) gh.pos_fb = gh.pos_cmd;
        }
    }
}

static
void pwm_write(void *arg, long period)
{
    static uint32_t ch;
    static int32_t freq;

    for ( ch = pwm_cnt; ch--; )
    {
        if ( !gh.enable ) {
            gh.freq = 0;
            pwm_ch_times_setup(ch,0,0,0,0,0);
            continue;
        }

        pwm_update_pins(ch);
        pwm_update_pos_scale(ch);

        if ( gp.ctrl_type ) // velocity mode
        {
            // stop any movement if velocity value is too small
            if ( gh.vel_cmd < 1e-20 && gh.vel_cmd > -1e-20 ) {
                gh.freq = 0;
                pwm_ch_times_setup(ch,0,0,0,0,0);
                continue;
            } else {
                gh.freq = gh.pos_scale * gh.vel_cmd;
                freq = (int32_t) round(gh.freq);
            }
        }
        else // position mode
        {
            gp.pwm_pos_cmd = (hal_s32_t) round(gh.pos_scale * gh.pos_cmd);
            freq = (int32_t) (
                ((int64_t)(gp.pwm_pos_cmd - gh.counts)) *
                ((int64_t)period) /
                ((int64_t)1000000)
            );

            // stop any movement if velocity value is too small
            if ( !freq ) {
                gh.freq = 0;
                pwm_ch_times_setup(ch,0,0,0,0,0);
                continue;
            } else {
                gh.freq = (hal_float_t) freq;
            }
        }

        if ( freq ) pwm_ch_times_setup(ch, freq, gh.pwm_duty, gh.dir_hold, gh.dir_setup, 1);
    }
}

static
void comp_read(void *arg, long period)
{
    gpio_read(arg, period);
    pwm_read(arg, period);
}

static
void comp_write(void *arg, long period)
{
    gpio_write(arg, period);
    pwm_write(arg, period);
}




// INIT

int32_t rtapi_app_main(void)
{
    if ( (comp_id = hal_init(comp_name)) < 0 )
        PRINT_ERROR_AND_RETURN("ERROR: hal_init() failed\n",-1);

    if ( shmem_init(comp_name) || malloc_and_export(comp_name, comp_id) )
    {
        hal_exit(comp_id);
        return -1;
    }

    pwm_cleanup(0);
    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    pwm_cleanup(0);
    shmem_deinit();
    hal_exit(comp_id);
}
