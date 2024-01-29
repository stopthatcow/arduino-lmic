/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * Copyright (c) 2016-2017, 2019 MCCI Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LMIC_DR_LEGACY 0

#include "lmic.h"
#include "mgos.h"

extern const struct lmic_pinmap lmic_pins;


int os_init_ex (const void *pintable) {
    hal_init_ex(pintable);
    if (! radio_init(false))
        return 0;
    radio_rand_init();
    LMIC_init();
    return 1;
}

void os_init() {
    if (os_init_ex((const void *)&lmic_pins))
        return;
    ASSERT(0);
}

ostime_t os_getTime () {
    return hal_ticks();
}

// clear scheduled job
void os_clearCallback (osjob_t* job) {
    mgos_clear_timer((mgos_timer_id)(job->timer));
    job->timer = MGOS_INVALID_TIMER_ID;
}

static void os_run_job(void *arg){
    osjob_t* job = (osjob_t*)arg; 
    job->timer = MGOS_INVALID_TIMER_ID;
    job->func(job);
}

// schedule immediately runnable job
void os_setCallback (osjob_t* job, osjobcb_t cb) {
    mgos_clear_timer((mgos_timer_id)(job->timer));
    job->func = cb;
    job->timer = (void *)mgos_set_timer(0, /*flags=*/0, os_run_job, job);
}

// schedule timed job
void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb) {
    mgos_clear_timer((mgos_timer_id)(job->timer));
    hal_disableIRQs(); // Disable IRQ to ensure our time snapshots are coherent.
    const ostime_t relative_ticks = time - os_getTime();
    const int relative_ms = osticks2ms(relative_ticks > 0 ? relative_ticks : 0);
    job->func = cb;
    job->timer = (void *)mgos_set_timer(relative_ms, /*flags=*/0, os_run_job, job);
    hal_enableIRQs();
}
