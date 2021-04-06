#ifndef _ARISC_API_H
#define _ARISC_API_H

#include "rtapi.h"
#include "rtapi_app.h"
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/fsuid.h>
#include <time.h>




#define ARISC_CPU_FREQ          450000000 // Hz
#define ARISC_WASTED_TICKS      (160/2) // number of ARISC ticks wasted for calculations
#define ARISC_FW_BASE           (0x00040000) // for ARM CPU it's 0x00040000
#define ARISC_FW_SIZE           ((8+8+32)*1024)
#define ARISC_SHM_SIZE          (4096)
#define ARISC_SHM_BASE          (ARISC_FW_BASE + ARISC_FW_SIZE - ARISC_SHM_SIZE)




#define GPIO_BASE               0x01c20800
#define GPIO_R_BASE             0x01f02c00
#define GPIO_BANK_SIZE          0x24

#define GPIO_PORTS_MAX_CNT      8
#define GPIO_PINS_MAX_CNT       24

enum
{
    GPIO_FUNC_IN,
    GPIO_FUNC_OUT,
    GPIO_FUNC_2,
    GPIO_FUNC_3,
    GPIO_FUNC_RESERVED4,
    GPIO_FUNC_RESERVED5,
    GPIO_FUNC_EINT,
    GPIO_FUNC_DISABLE,
    GPIO_FUNC_CNT
};

enum
{
    GPIO_MULTI_DRIVE_LEVEL0,
    GPIO_MULTI_DRIVE_LEVEL1,
    GPIO_MULTI_DRIVE_LEVEL2,
    GPIO_MULTI_DRIVE_LEVEL3,
    GPIO_MULTI_DRIVE_LEVEL_CNT
};

enum
{
    GPIO_PULL_DISABLE,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
    GPIO_PULL_RESERVED3,
    GPIO_PULL_CNT
};

enum { PA, PB, PC, PD, PE, PF, PG, PL };
enum { LOW, HIGH };

#define GPIO_PIN_SET(PORT,PIN_MASK) \
    _GPIO[PORT]->data |= PIN_MASK

#define GPIO_PIN_CLR(PORT,PIN_MASK_NOT) \
    _GPIO[PORT]->data &= PIN_MASK_NOT

#define GPIO_PIN_GET(PORT,PIN_MASK) \
    (_GPIO[PORT]->data & PIN_MASK)

typedef struct
{
    uint32_t config[4];
    uint32_t data;
    uint32_t drive[2];
    uint32_t pull[2];
} _GPIO_PORT_REG_t;




#define PWM_CH_MAX_CNT 16

enum
{
    PWM_CH_POS,
    PWM_CH_TICK,
    PWM_CH_TIMEOUT,

    PWM_CH_P_BUSY,
    PWM_CH_P_PORT,
    PWM_CH_P_PIN_MSK,
    PWM_CH_P_PIN_MSKN,
    PWM_CH_P_T0,
    PWM_CH_P_T1,
    PWM_CH_P_STOP,

    PWM_CH_D_BUSY,
    PWM_CH_D_PORT,
    PWM_CH_D_PIN_MSK,
    PWM_CH_D_PIN_MSKN,
    PWM_CH_D_T0,
    PWM_CH_D_T1,
    PWM_CH_D,
    PWM_CH_D_CHANGE,

    PWM_CH_DATA_CNT
};

enum
{
    PWM_TIMER_TICK,
    PWM_ARM_LOCK,
    PWM_ARISC_LOCK,
    PWM_CH_CNT,
    PWM_DATA_CNT
};

#define PWM_SHM_BASE         (ARISC_SHM_BASE)
#define PWM_SHM_DATA_BASE    (PWM_SHM_BASE)
#define PWM_SHM_CH_DATA_BASE (PWM_SHM_DATA_BASE + PWM_DATA_CNT*4)
#define PWM_SHM_SIZE         (PWM_SHM_CH_DATA_BASE + PWM_CH_MAX_CNT*PWM_CH_DATA_CNT*4)




#define PRINT_ERROR(MSG) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: "MSG"\n", comp_name)

#define PRINT_ERROR_AND_RETURN(MSG,RETVAL) \
    { PRINT_ERROR(MSG); return RETVAL; }




static uint32_t *_shm_vrt_addr, *_gpio_vrt_addr, *_r_gpio_vrt_addr;

volatile _GPIO_PORT_REG_t *_GPIO[GPIO_PORTS_MAX_CNT] = {0};
static uint32_t _gpio_buf[GPIO_PORTS_MAX_CNT] = {0};

volatile uint32_t * _pwmc[PWM_CH_MAX_CNT][PWM_CH_DATA_CNT] = {0};
volatile uint32_t * _pwmd[PWM_DATA_CNT] = {0};




static inline
void _spin_lock()
{
    *_pwmd[PWM_ARM_LOCK] = 1;
    while ( *_pwmd[PWM_ARISC_LOCK] );
}

static inline
void _spin_unlock()
{
    *_pwmd[PWM_ARM_LOCK] = 0;
}

static inline
int32_t spin_lock_test(uint32_t usec)
{
    _spin_lock();
    usleep(usec);
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_pull_set(uint32_t port, uint32_t pin, uint32_t level, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
        if ( level >= GPIO_PULL_CNT ) return -3;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    _spin_lock();
    _GPIO[port]->pull[slot] &= ~(0b11 << pos);
    _GPIO[port]->pull[slot] |= (level << pos);
    _spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_pull_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    return (_GPIO[port]->pull[slot] >> pos) & 0b11;
}

static inline
int32_t gpio_pin_multi_drive_set(uint32_t port, uint32_t pin, uint32_t level, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
        if ( level >= GPIO_MULTI_DRIVE_LEVEL_CNT ) return -3;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    _spin_lock();
    _GPIO[port]->drive[slot] &= ~(0b11 << pos);
    _GPIO[port]->drive[slot] |= (level << pos);
    _spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_multi_drive_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    return (_GPIO[port]->drive[slot] >> pos) & 0b11;
}

static inline
int32_t gpio_pin_func_set(uint32_t port, uint32_t pin, uint32_t func, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
        if ( func >= GPIO_FUNC_CNT ) return -3;
    }
    uint32_t slot = pin/8, pos = pin%8*4;
    _spin_lock();
    _GPIO[port]->config[slot] &= ~(0b0111 << pos);
    _GPIO[port]->config[slot] |=    (func << pos);
    _spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_func_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/8, pos = pin%8*4;
    return (_GPIO[port]->config[slot] >> pos) & 0b0111;
}

static inline
uint32_t gpio_pin_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
        if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    }
    return _GPIO[port]->data & (1UL << pin) ? HIGH : LOW;
}

static inline
int32_t gpio_pin_set(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
        if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    }
    _spin_lock();
    _GPIO[port]->data |= (1UL << pin);
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_clr(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
        if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    }
    _spin_lock();
    _GPIO[port]->data &= ~(1UL << pin);
    _spin_unlock();
    return 0;
}

static inline
uint32_t gpio_port_get(uint32_t port, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    return _GPIO[port]->data;
}

static inline
int32_t gpio_port_set(uint32_t port, uint32_t mask, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    _spin_lock();
    _GPIO[port]->data |= mask;
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_port_clr(uint32_t port, uint32_t mask, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    _spin_lock();
    _GPIO[port]->data &= ~mask;
    _spin_unlock();
    return 0;
}

static inline
uint32_t* gpio_all_get(uint32_t safe)
{
    uint32_t port;
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
        _gpio_buf[port] = _GPIO[port]->data;
    return (uint32_t*) &_gpio_buf[0];
}

static inline
int32_t gpio_all_set(uint32_t* mask, uint32_t safe)
{
    uint32_t port;
    _spin_lock();
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        _GPIO[port]->data |= mask[port];
    }
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_all_clr(uint32_t* mask, uint32_t safe)
{
    uint32_t port;
    _spin_lock();
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        _GPIO[port]->data &= ~mask[port];
    }
    _spin_unlock();
    return 0;
}




static inline
int32_t pwm_cleanup(uint32_t safe)
{
    uint32_t c, d;

    if ( safe )
    {
    }

//    _spin_lock();
    for ( d = PWM_DATA_CNT; d--; ) *_pwmd[d] = 0;
    for ( c = PWM_CH_MAX_CNT; c--; ) {
        for ( d = PWM_CH_DATA_CNT; d--; ) *_pwmc[c][d] = 0;
    }
//    _spin_unlock();

    return 0;
}

static inline
int32_t pwm_data_set(uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= PWM_DATA_CNT ) return -1;
        if ( name == PWM_CH_CNT && value >= PWM_CH_MAX_CNT ) return -2;
    }
    _spin_lock();
    *_pwmd[name] = value;
    _spin_unlock();
    return 0;
}

static inline
uint32_t pwm_data_get(uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= PWM_DATA_CNT ) return 0;
    }
    _spin_lock();
    uint32_t value = *_pwmd[name];
    _spin_unlock();
    return value;
}

static inline
int32_t pwm_ch_data_set(uint32_t c, uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
        if ( name >= PWM_CH_DATA_CNT ) return -2;
    }
    _spin_lock();
    *_pwmc[c][name] = (name == PWM_CH_POS) ? (int32_t)value : value;
    _spin_unlock();
    return 0;
}

static inline
uint32_t pwm_ch_data_get(uint32_t c, uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return 0;
        if ( name >= PWM_CH_DATA_CNT ) return 0;
    }
    _spin_lock();
    uint32_t value = *_pwmc[c][name];
    _spin_unlock();
    return value;
}

static inline
int32_t pwm_ch_pins_setup (
    uint32_t c,
    uint32_t p_port, uint32_t p_pin, uint32_t p_inv,
    uint32_t d_port, uint32_t d_pin, uint32_t d_inv,
    uint32_t safe )
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
        if ( p_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( p_pin >= GPIO_PINS_MAX_CNT ) return -1;
        if ( d_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( d_pin >= GPIO_PINS_MAX_CNT ) return -1;
    }

    gpio_pin_func_set(p_port, p_pin, GPIO_FUNC_OUT, safe);
    gpio_pin_pull_set(p_port, p_pin, GPIO_PULL_DISABLE, safe);
    if ( p_inv ) gpio_pin_set(p_port, p_pin, safe);
    else         gpio_pin_clr(p_port, p_pin, safe);

    gpio_pin_func_set(d_port, d_pin, GPIO_FUNC_OUT, safe);
    gpio_pin_pull_set(d_port, d_pin, GPIO_PULL_DISABLE, safe);
    if ( d_inv ) gpio_pin_set(d_port, d_pin, safe);
    else         gpio_pin_clr(d_port, d_pin, safe);

    _spin_lock();
    *_pwmc[c][PWM_CH_P_PORT] = p_port;
    *_pwmc[c][PWM_CH_P_PIN_MSK] = 1UL << p_pin;
    *_pwmc[c][PWM_CH_P_PIN_MSKN] = ~(1UL << p_pin);
    *_pwmc[c][PWM_CH_D_PORT] = d_port;
    *_pwmc[c][PWM_CH_D_PIN_MSK] = 1UL << d_pin;
    *_pwmc[c][PWM_CH_D_PIN_MSKN] = ~(1UL << d_pin);
    _spin_unlock();

    return 0;
}

static inline
int32_t pwm_ch_times_setup (
    uint32_t c,
    int32_t p_freq_mHz, int32_t p_duty_s32,
    uint32_t d_hold_ns, uint32_t d_setup_ns,
    uint32_t safe )
{
    uint32_t p_t0, p_t1, p_period, d_t0, d_t1, d_change, ch_cnt, ch;

    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
        if ( !d_hold_ns ) d_hold_ns = 50000;
        if ( !d_setup_ns ) d_setup_ns = 50000;
    }

    ch_cnt = *_pwmd[PWM_CH_CNT];

    if ( !p_freq_mHz || !p_duty_s32 )
    {
        if ( !(*_pwmc[c][PWM_CH_P_BUSY]) || *_pwmc[c][PWM_CH_P_STOP] ) return 0;
        if ( (c+1) == ch_cnt )
        {
            for ( ch = c; ch < PWM_CH_MAX_CNT && *_pwmc[ch][PWM_CH_P_BUSY]; ch-- );
            if ( ch >= PWM_CH_MAX_CNT ) ch = 0;
            ch_cnt = ch + 1;
        }
        _spin_lock();
        *_pwmc[c][PWM_CH_P_STOP] = 1;
        *_pwmd[PWM_CH_CNT] = ch_cnt;
        _spin_unlock();
        return 0;
    }

    if ( c >= ch_cnt ) ch_cnt = c + 1;

    d_change = (p_freq_mHz > 0 && (*_pwmc[c][PWM_CH_D])) ||
               (p_freq_mHz < 0 && !(*_pwmc[c][PWM_CH_D])) ? 1 : 0;
    d_change = (p_duty_s32 > 0 && (*_pwmc[c][PWM_CH_D])) ||
               (p_duty_s32 < 0 && !(*_pwmc[c][PWM_CH_D])) ?
                   (d_change ? 0 : 1) :
                   (d_change ? 1 : 0) ;

    p_duty_s32 = p_duty_s32 < 0 ? -p_duty_s32 : p_duty_s32;

    p_period = ARISC_CPU_FREQ / (p_freq_mHz < 0 ? -p_freq_mHz : p_freq_mHz) / 1000;
    p_period = p_period < (2*ARISC_WASTED_TICKS) ? 0 : p_period - (2*ARISC_WASTED_TICKS);
    p_t1 = (uint32_t) ( ((uint64_t)p_period) * ((uint64_t)p_duty_s32) / ((uint64_t)INT32_MAX) );
    p_t0 = p_period - p_t1;

    d_t0 = ARISC_CPU_FREQ / (1000000000 / d_hold_ns);
    d_t0 = d_t0 < ARISC_WASTED_TICKS ? 0 : d_t0 - ARISC_WASTED_TICKS;
    d_t1 = ARISC_CPU_FREQ / (1000000000 / d_setup_ns);
    d_t1 = d_t1 < ARISC_WASTED_TICKS ? 0 : d_t1 - ARISC_WASTED_TICKS;

    _spin_lock();
    *_pwmc[c][PWM_CH_P_BUSY] = 1;
    *_pwmc[c][PWM_CH_P_T0] = p_t0;
    *_pwmc[c][PWM_CH_P_T1] = p_t1;
    *_pwmc[c][PWM_CH_D_T0] = d_t0;
    *_pwmc[c][PWM_CH_D_T1] = d_t1;
    *_pwmc[c][PWM_CH_D_CHANGE] = d_change;
    *_pwmd[PWM_CH_CNT] = ch_cnt;
    _spin_unlock();

    return 0;
}

static inline
int32_t pwm_ch_pos_get(uint32_t c, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return 0;
    }
    _spin_lock();
    int32_t value = (int32_t) *_pwmc[c][PWM_CH_POS];
    _spin_unlock();
    return value;
}

static inline
int32_t pwm_ch_pos_set(uint32_t c, int32_t pos, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
    }
    _spin_lock();
    *_pwmc[c][PWM_CH_POS] = (uint32_t) pos;
    _spin_unlock();
    return 0;
}




static inline
int32_t shmem_init(const char *comp_name)
{
    int32_t mem_fd;
    uint32_t addr, off, port, ch, name, *p;

    // open physical memory file
    seteuid(0);
    setfsuid( geteuid() );
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if ( mem_fd  < 0 ) PRINT_ERROR_AND_RETURN("ERROR: can't open /dev/mem file\n",-1);
    setfsuid( getuid() );

    // mmap shmem
    addr = PWM_SHM_BASE & ~(4096 - 1);
    off = PWM_SHM_BASE & (4096 - 1);
    _shm_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_shm_vrt_addr == MAP_FAILED) { printf("ERROR: shm mmap() failed\n"); return; }
    p = _shm_vrt_addr + off/4;
    for ( name = 0; name < PWM_DATA_CNT; name++, p++ ) _pwmd[name] = p;
    for ( ch = 0; ch < PWM_CH_MAX_CNT; ch++ ) {
        for ( name = 0; name < PWM_CH_DATA_CNT; name++, p++ ) _pwmc[ch][name] = p;
    }

    // mmap gpio
    addr = GPIO_BASE & ~(4096 - 1);
    off = GPIO_BASE & (4096 - 1);
    _gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_gpio_vrt_addr == MAP_FAILED) PRINT_ERROR_AND_RETURN("ERROR: gpio mmap() failed\n",-3);
    for ( port = PA; port <= PG; ++port )
    {
        _GPIO[port] = (_GPIO_PORT_REG_t *)(_gpio_vrt_addr + (off + port*0x24)/4);
    }

    // mmap r_gpio (PL)
    addr = GPIO_R_BASE & ~(4096 - 1);
    off = GPIO_R_BASE & (4096 - 1);
    _r_gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_r_gpio_vrt_addr == MAP_FAILED) PRINT_ERROR_AND_RETURN("ERROR: r_gpio mmap() failed\n",-4);
    _GPIO[PL] = (_GPIO_PORT_REG_t *)(_r_gpio_vrt_addr + off/4);

    // no need to keep phy memory file open after mmap
    close(mem_fd);

    return 0;
}

static inline
void shmem_deinit(void)
{
    munmap(_shm_vrt_addr, 4096);
    munmap(_gpio_vrt_addr, 4096);
    munmap(_r_gpio_vrt_addr, 4096);
}




#endif
