/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2018-2019 MCCI Corporation
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
// include all the lmic header files, including ../lmic/hal.h
#include "../lmic.h"
#include "esp_timer.h"
// include the C++ hal.h
#include "hal.h"
#include "mgos.h"  // TODO(nwiles): Remove this in favor of using the os layer.
#include "mgos_gpio.h"  // TODO(nwiles): Remove this in favor of using the os layer.
// we may need some things from stdio.
#include <stdio.h>

// -----------------------------------------------------------------------------
// I/O

static const Arduino_LMIC::HalPinmap_t *plmic_pins;
static Arduino_LMIC::HalConfiguration_t *pHalConfig;
static Arduino_LMIC::HalConfiguration_t nullHalConig;
static hal_failure_handler_t* custom_hal_failure_handler = NULL;
static int64_t hal_timer_offset_micros = 0;

static void hal_interrupt_init(); // Fwd declaration

static void hal_io_init () {
    // NSS required.
    // SX127x: DIO0 required, DIO1 is required for LoRa, DIO2 for FSK
    // SX126x: DIO0, DIO1 required.
    ASSERT(plmic_pins->nss != LMIC_UNUSED_PIN);
#if defined(CFG_sx1261_radio) || defined(CFG_sx1262_radio)
    ASSERT(plmic_pins->dio[0] != LMIC_UNUSED_PIN);
    ASSERT(plmic_pins->dio[1] != LMIC_UNUSED_PIN);
#elif defined(CFG_sx1272_radio) || defined(CFG_sx1276_radio)
    ASSERT(plmic_pins->dio[0] != LMIC_UNUSED_PIN);
    ASSERT(plmic_pins->dio[1] != LMIC_UNUSED_PIN || plmic_pins->dio[2] != LMIC_UNUSED_PIN);
#endif
//    Serial.print("nss: "); Serial.println(plmic_pins->nss);
//    Serial.print("rst: "); Serial.println(plmic_pins->rst);
//    Serial.print("dio[0]: "); Serial.println(plmic_pins->dio[0]);
//    Serial.print("dio[1]: "); Serial.println(plmic_pins->dio[1]);
//    Serial.print("dio[2]: "); Serial.println(plmic_pins->dio[2]);

    // initialize SPI chip select to high (it's active low)
    digitalWrite(plmic_pins->nss, HIGH);
    pinMode(plmic_pins->nss, OUTPUT);

    if (plmic_pins->rxtx != LMIC_UNUSED_PIN) {
        // initialize to RX
        digitalWrite(plmic_pins->rxtx, LOW != plmic_pins->rxtx_rx_active);
        pinMode(plmic_pins->rxtx, OUTPUT);
    }
    if (plmic_pins->rst != LMIC_UNUSED_PIN) {
        // initialize RST to floating
        pinMode(plmic_pins->rst, INPUT);
    }
    if(plmic_pins->busy != LMIC_UNUSED_PIN) {
        pinMode(plmic_pins->busy, INPUT);
    }

    hal_interrupt_init();
}

// val == 1  => tx
void hal_pin_rxtx (u1_t val) {
    if (plmic_pins->rxtx != LMIC_UNUSED_PIN)
        digitalWrite(plmic_pins->rxtx, val != plmic_pins->rxtx_rx_active);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if (plmic_pins->rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        digitalWrite(plmic_pins->rst, val);
        pinMode(plmic_pins->rst, OUTPUT);
    } else { // keep pin floating
        pinMode(plmic_pins->rst, INPUT);
    }
}

s1_t hal_getRssiCal (void) {
    return plmic_pins->rssi_cal;
}

void hal_irqmask_set (int mask) {
    if(plmic_pins->dio[0] != LMIC_UNUSED_PIN){
        if(mask & HAL_IRQMASK_DIO0)
            mgos_gpio_enable_int(plmic_pins->dio[0]);
        else
            mgos_gpio_disable_int(plmic_pins->dio[0]);
    }
    if(plmic_pins->dio[1] != LMIC_UNUSED_PIN){
        if(mask & HAL_IRQMASK_DIO1)
            mgos_gpio_enable_int(plmic_pins->dio[1]);
        else
            mgos_gpio_disable_int(plmic_pins->dio[1]);
    }
    if(plmic_pins->dio[2] != LMIC_UNUSED_PIN){
        if(mask & HAL_IRQMASK_DIO2)
            mgos_gpio_enable_int(plmic_pins->dio[2]);
        else
            mgos_gpio_disable_int(plmic_pins->dio[2]);
    }
}

#if defined(CFG_sx1261_radio) || defined(CFG_sx1262_radio)
// Datasheet defines typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static int MAX_BUSY_TIME = 5000;

void hal_pin_busy_wait (void) {
    if (plmic_pins->busy == LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
        delayMicroseconds(MAX_BUSY_TIME);
    } else {
        unsigned long start = micros();
        while((micros() - start) < MAX_BUSY_TIME && digitalRead(plmic_pins->busy)) /* wait */;
    }
}

// TODO(nwiles): Make this configurable if needed.
bool hal_dio3_controls_tcxo (void) {
    return false;
}
// TODO(nwiles): Make this configurable if needed.
bool hal_dio2_controls_rxtx (void) {
    return true;
}
#endif // defined(CFG_sx1261_radio) || defined(CFG_sx1262_radio)


//--------------------
// Interrupt handling
//--------------------
static constexpr unsigned NUM_DIO_INTERRUPT = 3;
static_assert(NUM_DIO_INTERRUPT <= NUM_DIO, "Number of interrupt-sensitive lines must be less than number of GPIOs");
static volatile int64_t interrupt_time[NUM_DIO_INTERRUPT] = {0};  // In micros using esp_timer_get_time().

#if !defined(LMIC_USE_INTERRUPTS)
static void hal_interrupt_init() {
    pinMode(plmic_pins->dio[0], INPUT);
    if (plmic_pins->dio[1] != LMIC_UNUSED_PIN)
        pinMode(plmic_pins->dio[1], INPUT);
    if (plmic_pins->dio[2] != LMIC_UNUSED_PIN)
        pinMode(plmic_pins->dio[2], INPUT);
    static_assert(NUM_DIO_INTERRUPT == 3, "Number of interrupt lines must be set to 3");
}

static bool dio_states[NUM_DIO_INTERRUPT] = {0};
void hal_pollPendingIRQs_helper() {
    uint8_t i;
    for (i = 0; i < NUM_DIO_INTERRUPT; ++i) {
        if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
            continue;

        if (dio_states[i] != digitalRead(plmic_pins->dio[i])) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i] && interrupt_time[i] == 0) {
                ostime_t const now = os_getTime();
                interrupt_time[i] = now ? now : 1;
            }
        }
    }
}

#else

static void hal_isr_userspace_handler(void *arg){
    const int isr_idx = reinterpret_cast<int>(arg);
    const ostime_t iTime = interrupt_time[isr_idx];
    interrupt_time[isr_idx] = 0;
    radio_irq_handler_v2(isr_idx, iTime);
}

// Interrupt handlers
static IRAM void hal_isr_handler(int pin, void *arg) {
    const int isr_idx = reinterpret_cast<int>(arg);
    if (interrupt_time[isr_idx] != 0) return;
    const ostime_t now = os_getTime();
    interrupt_time[isr_idx] = (now != 0) ? now : 1;
    mgos_invoke_cb(hal_isr_userspace_handler, arg, /*from_isr=*/true);
}

static_assert(NUM_DIO_INTERRUPT == 3, "number of interrupts must be 3 for initializing interrupt_fns[]");

static void hal_interrupt_init() {
  for (uint8_t i = 0; i < NUM_DIO_INTERRUPT; ++i) {
      if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
          continue;

      pinMode(plmic_pins->dio[i], INPUT);
      mgos_gpio_set_int_handler(plmic_pins->dio[i], MGOS_GPIO_INT_EDGE_POS,
                                hal_isr_handler, reinterpret_cast<void *>(i));
      // Intentionally not enabled yet.
      // TODO(nwiles): Detach interrupt on LMIC reset?
  }
}
#endif // LMIC_USE_INTERRUPTS

void hal_processPendingIRQs() {
    uint8_t i;
    for (i = 0; i < NUM_DIO_INTERRUPT; ++i) {
        ostime_t iTime;
        if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
            continue;

        // NOTE(tmm@mcci.com): if using interrupts, this next step
        // assumes uniprocessor and fairly strict memory ordering
        // semantics relative to ISRs. It would be better to use
        // interlocked-exchange, but that's really far beyond
        // Arduino semantics. Because our ISRs use "first time
        // stamp" semantics, we don't have a value-race. But if
        // we were to disable ints here, we might observe a second
        // edge that we'll otherwise miss. Not a problem in this
        // use case, as the radio won't release IRQs until we
        // explicitly clear them.
        iTime = interrupt_time[i];
        if (iTime != 0) {
            interrupt_time[i] = 0;
            radio_irq_handler_v2(i, iTime);
        }
    }
}

// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init () {
    SPI.begin();
}

void hal_spi_select (int on) {
    if (on)
        SPI.beginTransaction(SPISettings((plmic_pins->spi_freq == 0? LMIC_SPI_FREQ : plmic_pins->spi_freq), MSBFIRST, SPI_MODE0));
    else
        SPI.endTransaction();

    digitalWrite(plmic_pins->nss, !on);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    return SPI.transfer(out);
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // We need to fetch absolute time in ISR context where gettimeofday() doesn't work.
    // Instead use esp_timer_get_time() and calculate the offset between these timers here.
    hal_disableIRQs();
    const int64_t uptime_micros = esp_timer_get_time();
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const int64_t wall_micros = (tv.tv_sec * 1000000LL + tv.tv_usec);
    hal_timer_offset_micros = wall_micros - uptime_micros;
    hal_enableIRQs();
}

u4_t hal_ticks () {
    return (esp_timer_get_time() + hal_timer_offset_micros) >> US_PER_OSTICK_EXPONENT;
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

// deal with boards that are stressed by no-interrupt delays #529, etc.
#if defined(ARDUINO_DISCO_L072CZ_LRWAN1)
# define HAL_WAITUNTIL_DOWNCOUNT_MS 16      // on this board, 16 ms works better
# define HAL_WAITUNTIL_DOWNCOUNT_THRESH ms2osticks(16)  // as does this threashold.
#else
# define HAL_WAITUNTIL_DOWNCOUNT_MS 8       // on most boards, delay for 8 ms
# define HAL_WAITUNTIL_DOWNCOUNT_THRESH ms2osticks(9) // but try to leave a little slack for final timing.
#endif

u4_t hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // check for already too late.
    if (delta < 0)
        return -delta;

    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383. Also, STM32 does a better
    // job with delay is less than 10,000 us; so reduce in steps.
    // It's nice to use delay() for the longer times.
    while (delta > HAL_WAITUNTIL_DOWNCOUNT_THRESH) {
        // deliberately delay 8ms rather than 9ms, so we
        // will exit loop with delta typically positive.
        // Depends on BSP keeping time accurately even if interrupts
        // are disabled.
        delay(HAL_WAITUNTIL_DOWNCOUNT_MS);
        // re-synchronize.
        delta = delta_time(time);
    }

    // The radio driver runs with interrupt disabled, and this can
    // mess up timing APIs on some platforms. If we know the BSP feature
    // set, we can decide whether to use delta_time() [more exact, 
    // but not always possible with interrupts off], or fall back to
    // delay_microseconds() [less exact, but more universal]

#if defined(_mcci_arduino_version)
    // unluckily, delayMicroseconds() isn't very accurate.
    // but delta_time() works with interrupts disabled.
    // so spin using delta_time().
    while (delta_time(time) > 0)
        /* loop */;
#else // ! defined(_mcci_arduino_version)
    // on other BSPs, we need to stick with the older way,
    // until we fix the radio driver to run with interrupts
    // enabled.
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
#endif // ! defined(_mcci_arduino_version)

    // we aren't "late". Callers are interested in gross delays, not
    // necessarily delays due to poor timekeeping here.
    return 0;
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        interrupts();

#if !defined(LMIC_USE_INTERRUPTS)
        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs.
        // We merely collect the edges and timestamps here; we wait for
        // a call to hal_processPendingIRQs() before dispatching.
        hal_pollPendingIRQs_helper();
#endif /* !defined(LMIC_USE_INTERRUPTS) */
    }
}

uint8_t hal_getIrqLevel(void) {
    return irqlevel;
}

void hal_sleep () {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
#if !defined(__AVR)
static ssize_t uart_putchar (void *, const char *buf, size_t len) {
    return LMIC_PRINTF_TO.write((const uint8_t *)buf, len);
}

static cookie_io_functions_t functions =
 {
     .read = NULL,
     .write = uart_putchar,
     .seek = NULL,
     .close = NULL
 };

void hal_printf_init() {
    stdout = fopencookie(NULL, "w", functions);
    if (stdout != nullptr) {
        setvbuf(stdout, NULL, _IONBF, 0);
    }
}
#else // defined(__AVR)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}

#endif // !defined(ESP8266) || defined(ESP31B) || defined(ESP32)
#endif // defined(LMIC_PRINTF_TO)

void hal_init (void) {
    // use the global constant
    Arduino_LMIC::hal_init_with_pinmap(&lmic_pins);
}

// hal_init_ex is a C API routine, written in C++, and it's called
// with a pointer to an lmic_pinmap.
void hal_init_ex (const void *pContext) {
    const lmic_pinmap * const pHalPinmap = (const lmic_pinmap *) pContext;
    if (! Arduino_LMIC::hal_init_with_pinmap(pHalPinmap)) {
        hal_failed(__FILE__, __LINE__);
    }
}

// C++ API: initialize the HAL properly with a configuration object
namespace Arduino_LMIC {
bool hal_init_with_pinmap(const HalPinmap_t *pPinmap)
    {
    if (pPinmap == nullptr)
        return false;

    // set the static pinmap pointer.
    plmic_pins = pPinmap;

    // set the static HalConfiguration pointer.
    HalConfiguration_t * const pThisHalConfig = pPinmap->pConfig;

    if (pThisHalConfig != nullptr)
        pHalConfig = pThisHalConfig;
    else
        pHalConfig = &nullHalConig;

    pHalConfig->begin();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
    // declare success
    return true;
    }
}; // namespace Arduino_LMIC


void hal_failed (const char *file, u2_t line) {
    if (custom_hal_failure_handler != NULL) {
        (*custom_hal_failure_handler)(file, line);
    }

#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif

    hal_disableIRQs();

    // Infinite loop
    while (1) {
        ;
    }
}

void hal_set_failure_handler(const hal_failure_handler_t* const handler) {
    custom_hal_failure_handler = handler;
}

ostime_t hal_setModuleActive (bit_t val) {
    // setModuleActive() takes a c++ bool, so
    // it effectively says "val != 0". We
    // don't have to.
    return pHalConfig->setModuleActive(val);
}

bit_t hal_pin_tcxo(bit_t enable) {
    // TODO(nwiles): This doesn't control TCXO, only returns if we use it.
    return pHalConfig->queryUsingTcxo();
}

uint8_t hal_getTxPowerPolicy(
    u1_t inputPolicy,
    s1_t requestedPower,
    u4_t frequency
    ) {
    return (uint8_t) pHalConfig->getTxPowerPolicy(
                        Arduino_LMIC::HalConfiguration_t::TxPowerPolicy_t(inputPolicy),
                        requestedPower,
                        frequency
                        );
}
