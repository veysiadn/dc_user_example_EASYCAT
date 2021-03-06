#include <QCoreApplication>


#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>
#include <iostream>
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
#define gettid() syscall(__NR_gettid)

#define SCHED_DEADLINE	6

/* XXX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr		314
#define __NR_sched_getattr		315
#endif

#ifdef __i386__
#define __NR_sched_setattr		351
#define __NR_sched_getattr		352
#endif

#ifdef __arm__
#define __NR_sched_setattr		380
#define __NR_sched_getattr		381
#endif

struct sched_attr {
   __u32 size;

   __u32 sched_policy;
   __u64 sched_flags;

   /* SCHED_NORMAL, SCHED_BATCH */
   __s32 sched_nice;

   /* SCHED_FIFO, SCHED_RR */
   __u32 sched_priority;

   /* SCHED_DEADLINE (nsec) */
   __u64 sched_runtime;
   __u64 sched_deadline;
   __u64 sched_period;
};

int sched_setattr(pid_t pid,
         const struct sched_attr *attr,
         unsigned int flags)
{
   return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid,
         struct sched_attr *attr,
         unsigned int size,
         unsigned int flags)
{
   return syscall(__NR_sched_getattr, pid, attr, size, flags);
}



/****************************************************************************/

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY  1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING 1
#define MAX_SAFE_STACK (4096 * 1024)
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
static unsigned int counter = 1;
static unsigned int blink = 0;
static uint8_t *domain1_pd = NULL;
#define LAB_2_SlavePos  0, 0

#define LAB_2 0x0000079a, 0xababa002

// offsets for PDO entries
static uint32_t segments;
static uint32_t potentiometer;
static uint32_t switches;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
/*
static ec_pdo_entry_reg_t domain1_regs[] = {
    {LAB_2_SlavePos,  LAB_2, 0X0005, 0X01, &segments},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X01, &potentiometer},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X02, &switches},
    {}
};
*/
/* Master 0, Slave 0, "LAB_2"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa002
 * Revision number: 0x00000001
 */
/*
static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, 	// Segments //
    {0x0006, 0x01, 16}, // Potentiometer //
    {0x0006, 0x02, 8}, 	// switches //
};

static ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 1, slave_0_pdo_entries + 0}, // Outputs //
    {0x1a00, 2, slave_0_pdo_entries + 1}, // Inputs //
};

static ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
*/
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
static volatile sig_atomic_t sig = 0;
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received" << std::endl;
    sig = 1 ;
}
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
    int print_MaxMin = 3e5 + 100 ;
    uint32_t switchInfo=0;
    unsigned short potVal=0;
    uint32_t old_switchInfo=0;
    #if MEASURE_TIMING
        int begin=5;
        struct timespec startTime={}, endTime={}, lastStartTime = {};
        uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
        latency_min_ns = 0, latency_max_ns = 0,
        period_min_ns = 0, period_max_ns = 0,
        exec_min_ns = 0, exec_max_ns = 0,
        max_period=0, max_latency=0, old_latency = 0,
        max_exec=0, min_latency=0xffffffff,
        min_period=0xffffffff, min_exec=0xffffffff;
        int32_t    min_jitter = 0, max_jitter = 0xffffffff, jitter_ns = 0;
    #endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while(1)
 {

    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // Write application time to master
    //
    // It is a good idea to use the target time (not the measured time) as
    // application time, because it is more stable.
    //
    ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

    #if MEASURE_TIMING
       clock_gettime(CLOCK_TO_USE, &startTime);
       latency_ns = DIFF_NS(wakeupTime, startTime);
       period_ns  = DIFF_NS(lastStartTime, startTime);
       exec_ns    = DIFF_NS(lastStartTime, endTime);
       lastStartTime = startTime;
       if(!begin){
           if(old_latency != latency_ns)       jitter_ns = old_latency-latency_ns ;
           if(jitter_ns < 0 )                  jitter_ns*=-1;
           if(jitter_ns > max_jitter)          max_jitter  = jitter_ns ;
           if(jitter_ns < min_jitter)          min_jitter  = jitter_ns ;
           if(latency_ns > max_latency)        max_latency = latency_ns;
           if(period_ns > max_period)          max_period  = period_ns;
           if(exec_ns > max_exec)              max_exec    = exec_ns;
           if(latency_ns < min_latency)        min_latency = latency_ns;
           if(period_ns < min_period )         min_period  = period_ns;
           if(exec_ns < min_exec)              min_exec    = exec_ns;
       }
       old_latency = latency_ns;
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
    //check_domain1_state();

    if (counter)
    {
        counter--;
    }
    else
    {
        // do this at 1 Hz
        counter = 1000;
        // check for master state (optional)
       // check_master_state();
       // check_slave_config_states();
    }
        #if MEASURE_TIMING
        // output timing stats
        //  printf("-----------------------------------------------\n");
        /* printf("%10u %10u %10u %10u\n",
                period_ns, exec_ns,latency_ns,jitter_ns); */
        if(!print_MaxMin | sig){
            printf("%10u\n",
                    max_period);
            printf("%10u\n",
                    max_exec);
            printf("%10u\n",
                    max_latency);
            printf("%10d\n",
                    max_jitter);

            printf("%10u\n",
                    min_period);
            printf("%10u\n",
                    min_exec);
            printf("%10u\n",
                    min_latency);
            printf("%10d\n",
                    min_jitter);
            return NULL;
        }
        else{
            print_MaxMin--;
        }

        // printf("-----------------------------------------------\n");
        period_max_ns = 0;
        period_min_ns = 0xffffffff;
        exec_max_ns = 0;
        exec_min_ns = 0xffffffff;
        latency_max_ns = 0;
        latency_min_ns = 0xffffffff;
        #endif
            // calculate new process data
            switchInfo = EC_READ_U8(domain1_pd + switches);
            potVal     = EC_READ_U16(domain1_pd + potentiometer);

            // write process data
            if(switchInfo != old_switchInfo){

                EC_WRITE_U8(domain1_pd + segments, blink ? 0x0c : 0x03);
                blink = !blink;
            }
            old_switchInfo = switchInfo;

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
            if(begin) begin--;
    #if MEASURE_TIMING
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

    signal(SIGINT,signalHandler);
    int	fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    if(fd){
        std::cout << "EtherCAT master is not active, activating...." << std::endl;
        std::system("cd ~; sudo ethercatctl start");
        usleep(1e6);
        fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
        if(fd)
            std::cout << "EtherCAT device not found...." << std::endl;
    }
    ec_slave_config_t *sc;
    QCoreApplication a(argc, argv);
    stack_prefault();
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

    segments = ecrt_slave_config_reg_pdo_entry(sc,0x0005, 1, domain1, NULL);
    if (segments < 0)
        return -1;

    potentiometer = ecrt_slave_config_reg_pdo_entry(sc,0x0006, 0x01, domain1, NULL);
    if ( potentiometer < 0)
        return -1;

    switches = ecrt_slave_config_reg_pdo_entry(sc,0x0006,0x02,domain1,NULL);
    if(switches < 0)
        return -1;

    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc, 0x0006, PERIOD_NS, 1000, 0, 0);


    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }



    cpu_set_t  mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
    CPU_SET(1, &mask);
    CPU_SET(2, &mask);
    CPU_SET(3, &mask);
    int result = sched_setaffinity(0, sizeof(mask), &mask);
  //  struct sched_param param = {};
    pthread_t cyclicThread;
    pthread_attr_t attr;
    struct sched_attr atr;
    int ret = 0;
    atr.size = sizeof(attr);
    atr.sched_flags = 0;
    atr.sched_nice = 0;
    atr.sched_priority = 0;

    /* This creates a 10ms/30ms reservation */
    unsigned int flags = 0;
    atr.sched_policy = SCHED_DEADLINE;
    atr.sched_runtime = 5e4;
   // atr.sched_priority = 98;
    atr.sched_period = atr.sched_deadline = PERIOD_NS;
    ret = sched_setattr(0, &atr, flags);
    int err;

   // sched_getparam(0, &param);
    printf("\nStarting cyclic function.\n");
   // printf("Using priority  %d ",param.sched_priority );


  /*  err = pthread_attr_init(&attr);
    if (err) {
        printf("init pthread attributes failed\n");
        return -1;
    }
*/
   /*cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(4,&cpus);
    pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
*/
    /* Set a specific stack size  */
    /*err = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (err) {
        printf("pthread setstacksize failed\n");
        return -1 ;
    }*/

    /* Set scheduler policy and priority of pthread */
 /*   err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (err) {
        printf("pthread setschedpolicy failed\n");
        return -1 ;
    }*/
/*    err = pthread_attr_setschedparam(&attr, &param);
    if (err) {
            printf("pthread setschedparam failed\n");
            return -1 ;
    }*/
    /* Use scheduling parameters of attr */
  /* err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (err)
    {
        printf("pthread setinheritsched failed\n");
        return -1 ;
    }
*/
    /* Create a pthread with specified attributes */
    err = pthread_create(&cyclicThread, NULL, &cyclic_task, NULL);
    if (err) {
        printf("create pthread failed\n");
        return -1 ;
    }
    err = pthread_join(cyclicThread,NULL);
    printf("Thread returned\n");
    if(err){
        printf("Thread Join failed.\n");
        return -1 ;
    }
    printf("Main returned\n");
    return 0;
    return a.exec();
}

