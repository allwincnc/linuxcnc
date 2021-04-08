#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
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

    hal_u32_t *dir_port; // in
    hal_u32_t *dir_pin; // in
    hal_bit_t *dir_inv; // in
    hal_u32_t *dir_hold; // io
    hal_u32_t *dir_setup; // io

    hal_float_t *dc_cmd; // in
    hal_float_t *dc_scale; // io
    hal_float_t *dc_min; // io
    hal_float_t *dc_max; // io
    hal_float_t *dc_offset; // io

    hal_float_t *pos_cmd; // in
    hal_float_t *pos_scale; // io

    hal_float_t *vel_cmd; // in
    hal_float_t *vel_scale; // io

    hal_float_t *freq_cmd; // io

    hal_float_t *dc_fb; // out
    hal_float_t *pos_fb; // out
    hal_float_t *vel_fb; // out
    hal_float_t *freq_fb; // out
    hal_s32_t *counts; // out
}
pwm_ch_shmem_t;

typedef struct
{
    hal_bit_t enable;

    hal_u32_t pwm_port;
    hal_u32_t pwm_pin;
    hal_bit_t pwm_inv;

    hal_u32_t dir_port;
    hal_u32_t dir_pin;
    hal_bit_t dir_inv;
    hal_u32_t dir_hold;
    hal_u32_t dir_setup;

    hal_float_t dc_cmd;
    hal_float_t dc_scale;
    hal_float_t dc_min;
    hal_float_t dc_max;
    hal_float_t dc_offset;

    hal_float_t vel_cmd;
    hal_float_t vel_scale;

    hal_float_t freq_cmd;

    hal_float_t pos_scale;
    hal_float_t pos_fb;

    hal_u32_t ctrl_type;
    hal_s32_t freq_mHz;
    hal_s32_t dc_s32;
}
pwm_ch_priv_t;

static pwm_ch_shmem_t *pwmh;
static pwm_ch_priv_t pwmp[PWM_CH_MAX_CNT] = {0};
static uint8_t pwm_ch_cnt = 0;

#define ph *pwmh[ch] // `ph` means `PWM HAL`
#define pp pwmp[ch] // `pp` means `PWM Private`

enum
{
    PWM_CTRL_BY_POS,
    PWM_CTRL_BY_VEL,
    PWM_CTRL_BY_FREQ
};




// TOOLS

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

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

    // export GPIO HAL functions
    if ( gpio_out_cnt || gpio_in_cnt ) {
        r = 0;
        rtapi_snprintf(name, sizeof(name), "%s.gpio.write", comp_name);
        r += hal_export_funct(name, gpio_write, 0, 0, 0, comp_id);
        rtapi_snprintf(name, sizeof(name), "%s.gpio.read", comp_name);
        r += hal_export_funct(name, gpio_read, 0, 0, 0, comp_id);
        if ( r ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s.gpio: HAL functions export failed\n", comp_name);
            return -1;
        }
    }

    // get PWM channels count and type
    while ( (token = strtok(data, ",")) != NULL )
    {
        if ( data != NULL ) data = NULL;

        if      ( token[0] == 'P' || token[0] == 'p' ) type[pwm_ch_cnt++] = PWM_CTRL_BY_POS;
        else if ( token[0] == 'V' || token[0] == 'v' ) type[pwm_ch_cnt++] = PWM_CTRL_BY_VEL;
        else if ( token[0] == 'F' || token[0] == 'f' ) type[pwm_ch_cnt++] = PWM_CTRL_BY_FREQ;
    }

    if ( pwm_ch_cnt )
    {
        // export PWM HAL functions
        r = 0;
        rtapi_snprintf(name, sizeof(name), "%s.pwm.write", comp_name);
        r += hal_export_funct(name, pwm_write, 0, 0, 0, comp_id);
        rtapi_snprintf(name, sizeof(name), "%s.pwm.read", comp_name);
        r += hal_export_funct(name, pwm_read, 0, 0, 0, comp_id);
        if ( r ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s.pwm: HAL functions export failed\n", comp_name);
            return -1;
        }

        if ( pwm_ch_cnt > PWM_CH_MAX_CNT ) pwm_ch_cnt = PWM_CH_MAX_CNT;

        // shared memory allocation for PWM
        pwmh = hal_malloc(pwm_ch_cnt * sizeof(pwm_ch_shmem_t));
        if ( !pwmh ) PRINT_ERROR_AND_RETURN("hal_malloc() failed", -1);

        // export PWM HAL pins and set default values
        #define EXPORT_PIN(IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
            r += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(pwmh[ch].VAL), comp_id,\
            "%s.pwm.%d." NAME, comp_name, ch);\
            ph.VAL = DEFAULT;

        for ( r = 0, ch = pwm_ch_cnt; ch--; )
        {
            // public HAL data
            EXPORT_PIN(HAL_IN,bit,enable,"enable", 0);

            EXPORT_PIN(HAL_IN,u32,pwm_port,"pwm-port", UINT32_MAX);
            EXPORT_PIN(HAL_IN,u32,pwm_pin,"pwm-pin", UINT32_MAX);
            EXPORT_PIN(HAL_IN,bit,pwm_inv,"pwm-invert", 0);

            EXPORT_PIN(HAL_IN,u32,dir_port,"dir-port", UINT32_MAX);
            EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-pin", UINT32_MAX);
            EXPORT_PIN(HAL_IN,bit,dir_inv,"dir-invert", 0);
            EXPORT_PIN(HAL_IO,u32,dir_hold,"dir-hold", 50000);
            EXPORT_PIN(HAL_IO,u32,dir_setup,"dir-setup", 50000);

            EXPORT_PIN(HAL_IN,float,dc_cmd,"dc-cmd", 0.0);
            EXPORT_PIN(HAL_IO,float,dc_min,"dc-min", -1.0);
            EXPORT_PIN(HAL_IO,float,dc_max,"dc-max", 1.0);
            EXPORT_PIN(HAL_IO,float,dc_offset,"dc-offset", 0.0);
            EXPORT_PIN(HAL_IO,float,dc_scale,"dc-scale", 1.0);

            EXPORT_PIN(HAL_IO,float,pos_scale,"pos-scale", 1.0);
            if ( type[n] == PWM_CTRL_BY_POS ) {
                EXPORT_PIN(HAL_IN,float,pos_cmd,"pos-cmd", 0.0);
            }

            if ( type[n] == PWM_CTRL_BY_VEL ) {
                EXPORT_PIN(HAL_IO,float,vel_scale,"vel-scale", 1.0);
                EXPORT_PIN(HAL_IN,float,vel_cmd,"vel-cmd", 0.0);
            }

            if ( type[n] == PWM_CTRL_BY_FREQ ) {
                EXPORT_PIN(HAL_IO,float,freq_cmd,"freq-cmd", 0.0);
            }

            EXPORT_PIN(HAL_OUT,float,dc_fb,"dc-fb", 0.0);
            EXPORT_PIN(HAL_OUT,float,pos_fb,"pos-fb", 0.0);
            EXPORT_PIN(HAL_OUT,float,freq_fb,"freq-fb", 0.0);
            if ( type[n] == PWM_CTRL_BY_VEL ) {
                EXPORT_PIN(HAL_OUT,float,vel_fb,"vel-fb", 0.0);
            }
            EXPORT_PIN(HAL_OUT,s32,counts,"counts", 0);

            // private data
            pp.enable = 0;

            pp.pwm_port = UINT32_MAX;
            pp.pwm_pin = UINT32_MAX;
            pp.pwm_inv = 0;

            pp.dir_port = UINT32_MAX;
            pp.dir_pin = UINT32_MAX;
            pp.dir_inv = 0;
            pp.dir_hold = 50000;
            pp.dir_setup = 50000;

            pp.dc_cmd = 0.0;
            pp.dc_scale = 1.0;
            pp.dc_min = -1.0;
            pp.dc_max = 1.0;
            pp.dc_offset = 0.0;

            pp.vel_cmd = 0.0;
            pp.vel_scale = 1.0;

            pp.freq_cmd = 0.0;

            pp.pos_fb = 0.0;
            pp.pos_scale = 1.0;

            pp.ctrl_type = type[ch];
            pp.freq_mHz = 0;
            pp.dc_s32 = 0;
        }
        if ( r )
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s.pwm: HAL pins export failed\n", comp_name);
            return -1;
        }

        #undef EXPORT_PIN
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
int32_t pwm_get_new_dc(uint8_t ch)
{
    if ( ph.dc_cmd    == pp.dc_cmd &&
         ph.dc_scale  == pp.dc_scale &&
         ph.dc_offset == pp.dc_offset &&
         ph.dc_min    == pp.dc_min &&
         ph.dc_max    == pp.dc_max ) return pp.dc_s32;

    if ( ph.dc_min < -1.0 ) ph.dc_min = -1.0;
    if ( ph.dc_max > 1.0 ) ph.dc_max = 1.0;
    if ( ph.dc_scale < 1e-20 && ph.dc_scale > -1e-20 ) ph.dc_scale = 1.0;

    ph.dc_fb = ph.dc_cmd / ph.dc_scale + ph.dc_offset;

    if ( ph.dc_fb < ph.dc_min ) ph.dc_fb = ph.dc_min;
    if ( ph.dc_fb > ph.dc_max ) ph.dc_fb = ph.dc_max;

    pp.dc_cmd = ph.dc_cmd;
    pp.dc_min = ph.dc_min;
    pp.dc_max = ph.dc_max;
    pp.dc_offset = ph.dc_offset;
    pp.dc_scale = ph.dc_scale;

    return (int32_t) (ph.dc_fb * INT32_MAX);
}

static inline
int32_t pwm_get_new_freq(uint8_t ch, long period)
{
    int32_t freq = 0;

    switch ( pp.ctrl_type )
    {
        case PWM_CTRL_BY_POS: {
            if ( ph.pos_fb == ph.pos_cmd && pp.pos_scale == ph.pos_scale ) break;
            pp.pos_scale = ph.pos_scale;
            freq = (int32_t) round( ph.pos_scale * (ph.pos_cmd - pp.pos_fb) *
                                    ((hal_float_t)period) / 1000000 );
            break;
        }
        case PWM_CTRL_BY_VEL: {
            if ( pp.vel_cmd == ph.vel_cmd && pp.vel_scale == ph.vel_scale ) {
                freq = pp.freq_mHz;
                break;
            }
            pp.vel_cmd = ph.vel_cmd;
            pp.vel_scale = ph.vel_scale;
            if ( ph.vel_cmd < 1e-20 && ph.vel_cmd > -1e-20 ) {
                ph.vel_fb = 0;
                return 0;
            }
            if ( ph.vel_scale < 1e-20 && ph.vel_scale > -1e-20 ) ph.vel_scale = 1.0;
            freq = (int32_t) round(ph.vel_scale * ph.vel_cmd * 1000);
            ph.vel_fb = ((hal_float_t) freq) / ph.vel_scale / 1000;
            return freq;
        }
        case PWM_CTRL_BY_FREQ: {
            if ( pp.freq_cmd == ph.freq_cmd ) {
                freq = pp.freq_mHz;
                break;
            }
            pp.freq_cmd = ph.freq_cmd;
            if ( ph.freq_cmd < 1e-20 && ph.freq_cmd > -1e-20 ) break;
            freq = (int32_t) round(ph.freq_cmd * 1000);
        }
    }

    ph.freq_fb = freq ? ((hal_float_t) freq) / 1000 : 0.0;
    return freq;
}

static inline
void pwm_pins_update(uint8_t ch)
{
    uint32_t upd = 0;

    if ( pp.pwm_port != ph.pwm_port ) { pp.pwm_port = ph.pwm_port; upd++; }
    if ( pp.pwm_pin  != ph.pwm_pin )  { pp.pwm_pin  = ph.pwm_pin;  upd++; }
    if ( pp.pwm_inv  != ph.pwm_inv )  { pp.pwm_inv  = ph.pwm_inv;  upd++; }

    if ( pp.dir_port != ph.dir_port ) { pp.dir_port = ph.dir_port; upd++; }
    if ( pp.dir_pin  != ph.dir_pin )  { pp.dir_pin  = ph.dir_pin;  upd++; }
    if ( pp.dir_inv  != ph.dir_inv )  { pp.dir_inv  = ph.dir_inv;  upd++; }

    if ( upd ) pwm_ch_pins_setup(ch, ph.pwm_port, ph.pwm_pin, ph.pwm_inv,
                                     ph.dir_port, ph.dir_pin, ph.dir_inv, 0);
}

static inline
uint32_t pwm_pins_ok(uint8_t ch)
{
    if ( ph.pwm_port > GPIO_PORTS_MAX_CNT || ph.pwm_pin > GPIO_PINS_MAX_CNT ||
         ph.dir_port > GPIO_PORTS_MAX_CNT || ph.dir_pin > GPIO_PINS_MAX_CNT ) return 0;

    return 1;
}

static
void pwm_read(void *arg, long period)
{
    static int32_t counts, ch;

    for ( ch = pwm_ch_cnt; ch--; )
    {
        if ( !ph.enable ) continue;

        ph.counts = pwm_ch_pos_get(ch, 0);

        if ( ph.pos_scale < 1e-20 && ph.pos_scale > -1e-20 ) ph.pos_scale = 1.0;
        ph.pos_fb = ((hal_float_t)ph.counts) / ph.pos_scale;
        pp.pos_fb = ph.pos_fb;
        counts = (hal_s32_t) round(ph.pos_scale * ph.pos_cmd);
        if ( counts == ph.counts ) ph.pos_fb = ph.pos_cmd;
    }
}

static
void pwm_write(void *arg, long period)
{
    static int32_t freq, dc, ch, update;

    for ( ch = pwm_ch_cnt; ch--; )
    {
        if ( pp.enable != ph.enable ) {
            pp.enable = ph.enable;
            if ( !ph.enable ) goto pwm_ch_stop;
        } else if ( !ph.enable ) continue;

        pwm_pins_update(ch);

        if ( !pwm_pins_ok(ch) ) goto pwm_ch_stop;

        dc = pwm_get_new_dc(ch);
        freq = pwm_get_new_freq(ch, period);
        update = 0;

        if ( pp.freq_mHz != freq ) { pp.freq_mHz = freq; update++; }
        if ( pp.dc_s32 != dc ) { pp.dc_s32 = dc; update++; }
        if ( pp.dir_hold != ph.dir_hold ) { pp.dir_hold = ph.dir_hold; update++; }
        if ( pp.dir_setup != ph.dir_setup ) { pp.dir_setup = ph.dir_setup; update++; }

        if ( update ) pwm_ch_times_setup(ch, pp.freq_mHz, pp.dc_s32, pp.dir_hold, pp.dir_setup, 1);

        continue;

        pwm_ch_stop:
        pwm_ch_times_setup(ch,0,0,0,0,0);
    }
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

    // TODO - gpio cleanup too
    pwm_cleanup(0);
    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    // TODO - gpio cleanup too
    pwm_cleanup(0);
    shmem_deinit();
    hal_exit(comp_id);
}
