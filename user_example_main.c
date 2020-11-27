/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)           // 1ms period
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING
#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/
static unsigned int counter = FREQUENCY;
static unsigned int blink = 0;
// process data
static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;
static uint8_t *domain1_pd = NULL;
static uint8_t *domain1_pd2 = NULL;
#define LAB_2_SlavePos  0, 0
#define LAB_1_SlavePos  0, 1

#define LAB_1 0x0000079a, 0xababa001
#define LAB_2 0x0000079a, 0xababa002

// offsets for PDO entries
static uint8_t alarmStatus;
static uint32_t temperatureStatus;
static uint8_t segments;
static uint16_t potentiometer;
static uint8_t swithces;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};


static ec_pdo_entry_reg_t domain1_regs[] = {
    {LAB_2_SlavePos,  LAB_2, 0X0005, 0X01, &segments},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X01, &potentiometer},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X02, &swithces},
    {LAB_1_SlavePos,  LAB_1, 0X0005, 0X01, &alarmStatus},
    {LAB_1_SlavePos,  LAB_1, 0X0006, 0X01, &temperatureStatus},
    {}
};

/* Master 0, Slave 0, "LAB_2"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa002
 * Revision number: 0x00000001
 */

static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Segments */
    {0x0006, 0x01, 16}, /* Potentiometer */
    {0x0006, 0x02, 8}, /* Switches */
};

static ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 1, slave_0_pdo_entries + 0}, /* Outputs */
    {0x1a00, 2, slave_0_pdo_entries + 1}, /* Inputs */
};

static ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 1, "LAB_1"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa001
 * Revision number: 0x00000001
 */

static ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Alarm */
    {0x0006, 0x01, 32}, /* Temperature */
};

static ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 1, slave_1_pdo_entries + 0}, /* Outputs */
    {0x1a00, 1, slave_1_pdo_entries + 1}, /* Inputs */
};

static ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_ana_in, &s);

    if (s.al_state != sc_ana_in_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_ana_in_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_ana_in_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_ana_in_state = s;
}

/*****************************************************************************/

void cyclic_task()
{
    float tempData=0;
    unsigned short potVal=0;
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state
    check_domain1_state();

    if (counter) 
    {
        counter--;
    } 
    else
    { 
         // do this at 1 Hz
        counter = FREQUENCY;

        // calculate new process data
        blink = !blink;

        // check for master state (optional)
        //check_master_state();

        // check for slave configuration state(s) (optional)
        //check_slave_config_states();
    }

#if 1
    // read process data
    tempData = EC_READ_REAL(domain1_pd2 + temperatureStatus);
            potVal = EC_READ_U16(domain1_pd + potentiometer);
    printf("Potentiometer Val = %d\n Temp Value = %f \n",potVal,tempData);

#endif

#if 1
    // write process data
    EC_WRITE_U8(domain1_pd + segments, blink ? 0x0c : 0x03);
#endif

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    ec_slave_config_t *slave_config2;
    struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, LAB_2_SlavePos, LAB_2);
    if (!sc) {
        return -1;
    }
    slave_config2 = ecrt_master_slave_config(master,LAB_1_SlavePos,LAB_1);
    if(!slave_config2)
        return -1;

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }
    if(!(domain1_pd2 = ecrt_domain_data(domain1))) 
    return -1;

    segments = ecrt_slave_config_reg_pdo_entry(sc,
            0x0005, 1, domain1, NULL);
    if (segments < 0)
        return -1;

    potentiometer = ecrt_slave_config_reg_pdo_entry(sc,
            0x0006, 0x01, domain1, NULL);
    if ( potentiometer < 0)
        return -1;

    alarmStatus = ecrt_slave_config_reg_pdo_entry(slave_config2,
            0x005, 0x01, domain1, NULL);
    if (alarmStatus < 0)
        return -1;
    
    temperatureStatus = ecrt_slave_config_reg_pdo_entry(slave_config2,
            0x006, 0x01, domain1, NULL);
    if (temperatureStatus < 0)
        return -1;


    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = 80;

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("\nStarting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 0; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}

/****************************************************************************/
/*.libs/ec_user_example
# Generated by libtool (GNU libtool) 2.4.2 Debian-2.4.2-1.7ubuntu1
#
# The ec_user_example program cannot be directly executed until all the libtool
# libraries that it depends on are installed.
#
# This wrapper script should never be moved out of the build directory.
# If it is, it will not operate correctly.

# Sed substitution that helps us do robust quoting.  It backslashifie
*/
