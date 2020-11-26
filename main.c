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
#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING
#define MAX_SAFE_STACK (2048 * 1024)
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

// process data
static unsigned int counter = 0;
static unsigned int blink = 0;
// process data
static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;
static uint8_t *domain1_pd = NULL;
#define LAB_2_SlavePos  0, 0
#define LAB_1_SlavePos  0, 1

#define LAB_1 0x0000079a, 0xababa001
#define LAB_2 0x0000079a, 0xababa002

// offsets for PDO entries
static uint8_t alarmStatus;
static float temperatureStatus;
static uint8_t switches;
static uint16_t potentiometer;
static uint8_t segmentWrite;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

static ec_pdo_entry_reg_t domain1_regs[] = {
    {LAB_2_SlavePos,  LAB_2, 0X0005, 0X01, &switches},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X01, &potentiometer},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X02, &segmentWrite},
    {}
};

/* Master 0, Slave 0, "LAB_2"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa002
 * Revision number: 0x00000001
 */

static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* switches */
    {0x0006, 0x01, 16}, /* Potentiometer */
    {0x0006, 0x02, 8}, /* segmentWrite */
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

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}


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
/****************************************************************************/

void *cyclic_task(void *arg)
{
    struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0,
             max_period=0, max_latency=0,max_exec=0,
             min_latency=0xffffffff,
             jitter=0;
             
#endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    int begin=1 ;
    while(1) {
        
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;
        if(!begin)
        {
           if(latency_ns > max_latency)        max_latency = latency_ns;
           if(period_ns > max_period)          max_period  = period_ns;
           if(exec_ns > max_exec)              max_exec    = exec_ns;
        }
        if (latency_ns > latency_max_ns)  {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // check process data state (optional)
        check_domain1_state();

        if (counter) {
            counter--;
        } else { 
            // do this at 1 Hz
            counter = 10;
            // check for master state (optional)
        check_master_state();

#ifdef MEASURE_TIMING
            // output timing stats
            printf("-----------------------------------------------\n");
            printf("Tperiod   min : %10u ns  | max :%10u ns\n",
                    period_min_ns, max_period);
            printf("Texec     min : %10u ns  | max : %10u ns\n",
                    exec_min_ns, max_exec);
            printf("Tlatency  min : %10u ns  | max : %10u ns\n",
                    latency_min_ns, max_latency);
            printf("Tjitter max: %10u ns  \n",
                    max_latency-min_latency, max_latency);
            printf("-----------------------------------------------\n");
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif

            // calculate new process data
            blink = !blink;
        }

        // write process data
        int readData = EC_READ_U8(domain1_pd + switches);
        EC_WRITE_U8(domain1_pd + segmentWrite, blink ? 0x0c : 0x03);

        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(master);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        if(begin) begin=0;
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
return NULL;
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;


    if (!(sc = ecrt_master_slave_config(master,
                    LAB_2_SlavePos, LAB_2))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    switches = ecrt_slave_config_reg_pdo_entry(sc,
            0x0005, 1, domain1, NULL);
    if (switches < 0)
        return -1;

    if (!(sc = ecrt_master_slave_config(master,
                    LAB_1_SlavePos, LAB_1))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    alarmStatus = ecrt_slave_config_reg_pdo_entry(sc,
            0x005, 0x01, domain1, NULL);
    if (switches < 0)
        return -1;

    potentiometer = ecrt_slave_config_reg_pdo_entry(sc,
            0x0006, 1, domain1, NULL);
    if ( potentiometer < 0)
        return -1;

    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc, 0x0006, PERIOD_NS, 4400000, 0, 0);

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

 

  /*  printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }
*/   
        /* Set priority */
    printf("\nStarting cyclic function.\n");

    struct sched_param param;
    pthread_t cyclicThread;
    pthread_attr_t attr;
    int err;
    
    err = pthread_attr_init(&attr);
        if (err) {
                printf("init pthread attributes failed\n");
                goto out;
        }
 
        /* Set a specific stack size  */
        err = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
        if (err) {
            printf("pthread setstacksize failed\n");
            goto out;
        }
 
        /* Set scheduler policy and priority of pthread */
        err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (err) {
                printf("pthread setschedpolicy failed\n");
                goto out;
        }
        param.sched_priority = 80;
        err = pthread_attr_setschedparam(&attr, &param);
        if (err) {
                printf("pthread setschedparam failed\n");
                goto out;
        }
        /* Use scheduling parameters of attr */
        err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (err) {
                printf("pthread setinheritsched failed\n");
                goto out;
        }
 
        /* Create a pthread with specified attributes */
        err = pthread_create(&cyclicThread, &attr, cyclic_task, NULL);
        if (err) {
                printf("create pthread failed\n");
                goto out;
        }
 
        /* Join the thread and wait until it is done */
        err = pthread_join(cyclicThread, NULL);
        if (err)
                printf("join pthread failed: %m\n");
    
/*
    pthread_attr_init(&attr);

    pthread_attr_setschedpolicy(&attr,SCHED_FIFO);

    param.sched_priority = 80;

    pthread_attr_setschedparam(&attr,&param);

    err = pthread_create(&cyclicThread,&attr,&cyclic_task,NULL);

    err = pthread_join(cyclicThread,NULL);
*/
    //cyclic_task();

out:
        return err;
        
    return 0;
}

/****************************************************************************/
