
#include "ethercat_common.h"

#include <stdio.h>          // printf, fprintf, perror
#include <stdlib.h>         // exit, malloc, free
#include <stdint.h>         // uint64_t, uint32_t, etc.
#include <stdbool.h>        // bool, true, false
#include <unistd.h>         // read, usleep, close
#include <string.h>         // memset, memcpy, etc.
#include <pthread.h>        // pthread_create, pthread_join, etc.
#include <sys/timerfd.h>    // timerfd_create, timerfd_settime, etc.
#include <time.h>           // struct timespec and itimerspec
#include <errno.h>          // errno
#include <net/if.h>
#include "cia402_def.h"

#define ESC_TIMER          (HPM_GPTMR3)
#define ESC_TIMER_CH       1
#define ESC_TIMER_IRQ      IRQn_GPTMR3
#define ESC_TIMER_CLK_NAME (clock_gptmr3)

#define EC_TIMEOUTMON 500

volatile bool esc_pdo_error = false;
volatile bool esc_enter_op = false;

static volatile bool worker_threads_running = false;
static bool pdo_thread_started = false;
static bool check_thread_started = false;
static pthread_t pdo_thread;
static pthread_t check_thread;
static int cycle_time_us = DC_SYNC_CYCLE_MS * 1000;
static int expected_wkc = 0;
static volatile int last_wkc = 0;
static bool ethercat_open = false;
static char ethercat_ifname[IFNAMSIZ] = DEFAULT_ECAT_IFNAME;
pthread_mutex_t pdo_mutex;
static bool pdo_mutex_initialized = false;

char IOmap[4096];
boolean inOP;

#define NSEC_PER_SEC 1000000000
#define stack64k (64 * 1024)
int dorun = 0;
uint8 *digout = 0;

static bool verbose_startup_enabled(void)
{
   const char *value = getenv("IPE_ECAT_VERBOSE");
   return value && value[0] != '\0' && strcmp(value, "0") != 0;
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec   ts, tleft;
   int ht;
   int64 cycletime;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC) {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */
   int64 toff = 0;
   int bad_wkc_cycles = 0;
   while(worker_threads_running)
   {
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {

        pthread_mutex_lock(&pdo_mutex);
         ec_send_processdata();
         int wkc = ec_receive_processdata(EC_TIMEOUTRET);
         last_wkc = wkc;
        pthread_mutex_unlock(&pdo_mutex);

         if ((expected_wkc > 0) && (wkc < expected_wkc))
         {
            if (++bad_wkc_cycles >= 10)
               esc_pdo_error = true;
         }
         else
         {
            bad_wkc_cycles = 0;
            esc_pdo_error = false;
         }


         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }

      }
   }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;

    (void) ptr;

    boolean needlf = FALSE;
    uint8 currentgroup = 0;

    while(worker_threads_running)
    {
        if( inOP && ec_group[currentgroup].docheckstate)
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = FIRST_SLAVE; slave <= ec_slavecount; slave++)
            {
               #if USE_BRANCHER == 1
                   if (slave == BRANCHER_ADDR)
                       continue;
               #endif
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

static void *ecatthread_pthread_entry(void *ptr)
{
    ecatthread(ptr);
    return NULL;
}

static void *ecatcheck_pthread_entry(void *ptr)
{
    ecatcheck(ptr);
    return NULL;
}

int ethercat_common_set_interface(const char *ifname)
{
    if (!ifname || !ifname[0] || strlen(ifname) >= sizeof(ethercat_ifname))
        return -1;
    if (ethercat_open)
        return -2;
    strcpy(ethercat_ifname, ifname);
    return 0;
}

const char *ethercat_common_get_interface(void)
{
    return ethercat_ifname;
}

int ethercat_common_get_bus_status(uint16_t slave, uint16_t *state,
                                   uint16_t *al_status, int *actual_wkc,
                                   int *required_wkc)
{
    if (!ethercat_open || slave == 0 || slave > ec_slavecount)
        return -1;

    pthread_mutex_lock(&pdo_mutex);
    ec_readstate();
    if (state)
        *state = ec_slave[slave].state;
    if (al_status)
        *al_status = ec_slave[slave].ALstatuscode;
    if (actual_wkc)
        *actual_wkc = last_wkc;
    if (required_wkc)
        *required_wkc = expected_wkc;
    pthread_mutex_unlock(&pdo_mutex);
    return 0;
}

static void stop_worker_threads(void)
{
    dorun = 0;
    worker_threads_running = false;
    if (pdo_thread_started) {
        pthread_join(pdo_thread, NULL);
        pdo_thread_started = false;
    }
    if (check_thread_started) {
        pthread_join(check_thread, NULL);
        check_thread_started = false;
    }
}

static int start_worker_thread(pthread_t *thread, size_t stack_size,
                               void *(*function)(void *), void *argument,
                               bool realtime)
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, stack_size);
    int ret = pthread_create(thread, &attr, function, argument);
    pthread_attr_destroy(&attr);
    if (ret != 0)
        return 0;

    if (realtime) {
        struct sched_param scheduling = { .sched_priority = 40 };
        if (pthread_setschedparam(*thread, SCHED_FIFO, &scheduling) != 0)
            fprintf(stderr, "Warning: PDO thread is not using SCHED_FIFO.\n");
    }
    return 1;
}

int ethercat_common_start(esc_sdo_config_t sdo_config,
                          esc_preop_config_t preop_config)
{
    if (ethercat_open) {
        fprintf(stderr, "EtherCAT is already open.\n");
        return -6;
    }

    if (!pdo_mutex_initialized) {
        pthread_mutexattr_t mutexattr;
        pthread_mutexattr_init(&mutexattr);
        pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
        pthread_mutex_init(&pdo_mutex, &mutexattr);
        pthread_mutexattr_destroy(&mutexattr);
        pdo_mutex_initialized = true;
    }

    int cnt, i, j, slc, try_count;

    dorun = 0;
    esc_pdo_error = false;
    esc_enter_op = false;

    /* initialise SOEM */
    if (ec_init(ethercat_ifname) > 0) {
        ethercat_open = true;
        printf("ec_init on %s succeeded.\n", ethercat_ifname);

        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (slc = FIRST_SLAVE; slc <= ec_slavecount; slc++) {
               #if USE_BRANCHER == 1
                   if (slc == BRANCHER_ADDR)
                       continue;
               #endif
                printf("Found %s at position %d\n", ec_slave[slc].name, slc);
            }

            if (sdo_config) {
                sdo_config();
            }

            ec_configdc();
            for (uint16_t dc_slave = FIRST_SLAVE;
                 dc_slave <= ec_slavecount; ++dc_slave) {
#if USE_BRANCHER == 1
                if (dc_slave == BRANCHER_ADDR)
                    continue;
#endif
                ec_dcsync0(dc_slave, TRUE, DC_SYNC_CYCLE_MS * 1000000, 20000);
            }

            uint32 dc_time;
            for (uint16_t i = 0; i < 16000; i++) {
                dc_time = 0;
                ec_FRMW(ec_slave[FIRST_SLAVE].configadr, ECT_REG_DCSYSTIME, 4, &dc_time, 5 * EC_TIMEOUTRET);
            }

            slc = 0;

            /* Run IO mapping */
            int iomap_size = ec_config_map(&IOmap);
            if (iomap_size <= 0 || iomap_size > (int)sizeof(IOmap)) {
                fprintf(stderr, "Invalid IO map size: %d bytes.\n", iomap_size);
                ec_close();
                ethercat_open = false;
                return -9;
            }
            expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(slc, EC_STATE_SAFE_OP, 5 * EC_TIMEOUTSTATE);
            ec_readstate();
            printf("Slave 0 State=0x%04x\r\n", ec_slave[0].state);
            printf("Slave 1 State=0x%04x\r\n", ec_slave[FIRST_SLAVE].state);

            /* Inputs are valid in SAFE-OP. Read them before preparing the
             * first output frame, so CSP can start from the measured position. */
            for (int safeop_cycle = 0; safeop_cycle < 10; ++safeop_cycle) {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                osal_usleep(1000);
            }
            if (preop_config && preop_config() < 0) {
                fprintf(stderr, "Failed to prepare safe PDO outputs in SAFE-OP.\n");
                ec_close();
                ethercat_open = false;
                return -10;
            }

            /* Keep normal learning output short. Full FMMU/address details
             * remain available with IPE_ECAT_VERBOSE=1. */
            const bool verbose_startup = verbose_startup_enabled();
            for (cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
               #if USE_BRANCHER == 1
                   if (cnt == BRANCHER_ADDR)
                       continue;
               #endif
                if( !digout)
                {
                    digout = ec_slave[cnt].outputs;
                }
                printf("Slave %d: %s, PDO out=%dbit in=%dbit, DC=%d, address=0x%x\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits,
                       ec_slave[cnt].Ibits, ec_slave[cnt].hasdc,
                       ec_slave[cnt].configadr);
                if (verbose_startup) {
                    printf("  state=%d delay=%d ns outputs=%p inputs=%p\n",
                           ec_slave[cnt].state, ec_slave[cnt].pdelay,
                           (void *)ec_slave[cnt].outputs,
                           (void *)ec_slave[cnt].inputs);
                    for (j = 0; j < ec_slave[cnt].FMMUunused; j++) {
                        printf("  FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
                               (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
                               ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
                               ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
                    }
                    printf("  FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                           ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func,
                           ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
                }
            }

            printf("Request operational state for all slaves\n");

            /* Enable process-data exchange before the worker is created.
             * The worker must never reset dorun: doing so races this startup
             * path and can leave the slave in SAFE-OP + watchdog error. */
            dorun = 1;
            worker_threads_running = true;
            if (!start_worker_thread(&pdo_thread, stack64k * 2,
                                     ecatthread_pthread_entry,
                                     (void *)&cycle_time_us, true)) {
                worker_threads_running = false;
                fprintf(stderr, "Failed to create EtherCAT PDO thread.\n");
                ec_close();
                ethercat_open = false;
                return -7;
            }
            pdo_thread_started = true;
            if (!start_worker_thread(&check_thread, stack64k * 4,
                                     ecatcheck_pthread_entry,
                                     NULL, false)) {
                fprintf(stderr, "Failed to create EtherCAT recovery thread.\n");
                stop_worker_threads();
                ec_close();
                ethercat_open = false;
                return -8;
            }
            check_thread_started = true;

            try_count = 0;
            ec_slave[slc].state = EC_STATE_OPERATIONAL;
            ec_writestate(slc);
            do {
                ec_statecheck(slc, EC_STATE_OPERATIONAL, 0);
                if (ec_slave[slc].state ==
                    (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                    fprintf(stderr,
                            "SAFE-OP + ERROR while entering OP; acknowledging and retrying.\n");
                    for (i = FIRST_SLAVE; i <= ec_slavecount; ++i) {
                        if (ec_slave[i].state ==
                            (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                            ec_slave[i].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
                            ec_writestate(i);
                            ec_statecheck(i, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
                        }
                    }
                    ec_slave[slc].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slc);
                }
                if (try_count++ >= 5000) {
                    fprintf(stderr,
                            "Timed out waiting for EtherCAT OP (wkc=%d, expected=%d, pdo_error=%d).\n",
                            last_wkc, expected_wkc, esc_pdo_error ? 1 : 0);
                    ec_readstate();
                    for (i = FIRST_SLAVE; i <= ec_slavecount; ++i) {
                        fprintf(stderr,
                                "Slave %d: state=0x%02x AL=0x%04x (%s), lost=%d\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                                ec_ALstatuscode2string(ec_slave[i].ALstatuscode),
                                ec_slave[i].islost);
                    }
                    stop_worker_threads();
                    ec_close();
                    ethercat_open = false;
                    return -2;
                }
                osal_usleep(1000);
            } while (ec_slave[slc].state != EC_STATE_OPERATIONAL);

            inOP = TRUE;

            printf("Slave 0 State=0x%04x\r\n", ec_slave[slc].state);

            if (ec_slave[slc].state == EC_STATE_OPERATIONAL) {
                esc_enter_op = true;
                printf("Operational state reached for all slaves.\n");
                return 0;
            } else {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = FIRST_SLAVE; i <= ec_slavecount; i++) {
                   #if USE_BRANCHER == 1
                       if (i == BRANCHER_ADDR)
                           continue;
                   #endif
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%04x StatusCode=0x%04x\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode);
                    }
                }
                return -3;
            }
        } else {
            printf("ec_config_init failed\n");
            ec_close();
            ethercat_open = false;
            return -4;
        }
    }
    printf("ec_init failed.\n");
    return -5;
}

void ethercat_common_stop(void)
{
    if (!ethercat_open)
        return;
    esc_enter_op = false;
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    inOP = FALSE;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    stop_worker_threads();
    ec_close();
    ethercat_open = false;
    expected_wkc = 0;
    last_wkc = 0;
    ec_slavecount = 0;
}
