/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int el7211_pos;


int EL7211_setup(uint16 slave) {
    /*
     *      EL7211 startup configuration function
     *      Reproduced by exporting the XML file on Startup Tab and converting the output with a Python script
     *      https://gist.github.com/81cd5b429b7080e9c8cc8780b2ea6f86
     */

    int retval = 0;
    uint8 u8val;
    uint16 u16val;
    uint32 u32val;

    int u32l = sizeof(u32l);

//    u32val = 1572874;
//    retval += ec_SDOwrite(slave, 0x8011, 0x1, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM); //
    u8val = 4;
    retval += ec_SDOwrite(slave, 0x8011, 0x13, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM); // Motor pole pairs (0x4)
    u32val = 4950;
    retval += ec_SDOwrite(slave, 0x8011, 0x12, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM); // Rated current (0x1356)
    u32val = 27800;
    retval += ec_SDOwrite(slave, 0x8011, 0x11, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM); // Max current (0x6c98)
    u32val = 270;
    retval += ec_SDOwrite(slave, 0x8011, 0x16, FALSE, sizeof(u32val), &u32val,
                          EC_TIMEOUTRXM); // Torque constant (0x10e)
    u16val = 53;
    retval += ec_SDOwrite(slave, 0x8011, 0x19, FALSE, sizeof(u16val), &u16val,
                          EC_TIMEOUTRXM); // Winding inductance (0x35)
    u32val = 467;
    retval += ec_SDOwrite(slave, 0x8011, 0x18, FALSE, sizeof(u32val), &u32val,
                          EC_TIMEOUTRXM); // Rotor moment of inertia (0x1d3)
    u16val = 721;
    retval += ec_SDOwrite(slave, 0x8011, 0x2d, FALSE, sizeof(u16val), &u16val,
                          EC_TIMEOUTRXM); // Motor thermal time constant (0x2d1)
    u16val = 270;
    retval += ec_SDOwrite(slave, 0x8011, 0x15, FALSE, sizeof(u16val), &u16val,
                          EC_TIMEOUTRXM); // Commutation offset (0x10e)
    u32val = 1108;
    retval += ec_SDOwrite(slave, 0x8011, 0x1b, FALSE, sizeof(u32val), &u32val,
                          EC_TIMEOUTRXM); // Motor speed limitation (0x454)
    u16val = 628;
    retval += ec_SDOwrite(slave, 0x8010, 0x13, FALSE, sizeof(u16val), &u16val,
                          EC_TIMEOUTRXM); // Current loop proportional gain (0x274)
    u16val = 5;
    retval += ec_SDOwrite(slave, 0x8010, 0x12, FALSE, sizeof(u16val), &u16val,
                          EC_TIMEOUTRXM); // Current loop integral time (0x5)
    u32val = 153;
    retval += ec_SDOwrite(slave, 0x8010, 0x15, FALSE, sizeof(u32val), &u32val,
                          EC_TIMEOUTRXM); // Velocity loop proportional gain (0x99)
    u32val = 150;
    retval += ec_SDOwrite(slave, 0x8010, 0x14, FALSE, sizeof(u32val), &u32val,
                          EC_TIMEOUTRXM); // Velocity loop integral time (0x96)

    while (EcatError) printf("%s", ec_elist2string());


    // Read parameters

    // TODO: velocity resolution should be saved to be converted as "real" velocity as on EL7211 manual, pg 118.
    ec_SDOread(slave, 0x9010, 0x14, FALSE, &u32l, &u32val, EC_TIMEOUTRXM); // Velocity encoder resolution
    printf("Velocity resolution: 0x%8.8x\n", u32val);


    if (retval != 14) {
        printf("EL7211 setup failed\t(retval = %i)\n", retval);
        return -1;
    }

    printf("EL7211 slave %d set, retval = %d\n", slave, retval);
    return 1;
}


// CanOPEN input/output rw helper functions
int32 get_input_int32(uint16 slave_no, uint8 module_index) {
    int32 return_value;
    uint8 *data_ptr;
    /* Get the IO map pointer from the ec_slave struct */
    data_ptr = ec_slave[slave_no].inputs;
    /* Move pointer to correct module index */
    data_ptr += module_index;
    /* Read value byte by byte since all targets can't handle misaligned
     * addresses
     */
    return_value = *data_ptr++;
    return_value += (*data_ptr++ << 8);
    return_value += (*data_ptr++ << 16);
    return_value += (*data_ptr++ << 24);

    return return_value;
}

int16 get_input_int16(uint16 slave_no, uint8 module_index) {
    int16 return_value;
    uint8 *data_ptr;
    /* Get the IO map pointer from the ec_slave struct */
    data_ptr = ec_slave[slave_no].inputs;
    /* Move pointer to correct module index */
    data_ptr += module_index;
    /* Read value byte by byte since all targets can't handle misaligned
     * addresses
     */
    return_value = *data_ptr++;
    return_value += (*data_ptr++ << 8);

    return return_value;
}

void set_output_int16(uint16 slave_no, uint8 module_index, int16 value) {
    uint8 *data_ptr;
    /* Get the IO map pointer from the ec_slave struct */
    data_ptr = ec_slave[slave_no].outputs;
    /* Move pointer to correct module index */
    data_ptr += module_index;
    /* Read value byte by byte since all targets can handle misaligned
     * addresses
     */
    *data_ptr++ = (value >> 0) & 0xFF;
    *data_ptr++ = (value >> 8) & 0xFF;
}


void redtest(char *ifname) {
    int cnt, i, j, oloop, iloop;

    printf("Starting Redundant test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)   // == ec_config_init + ec_config_map
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            // PO2SOconfig is for registering a hook function that will be called when the slave does the transition
            // between Pre-OP and Safe-OP.
            if ((ec_slavecount > 1)) {
                for (cnt = 1; cnt <= ec_slavecount; cnt++) {
                    if (ec_slave[cnt].eep_id == 0x1c2B3052) { // eep_id == ProductCode on ESI
                        el7211_pos = cnt;
                        printf("Found %s at position %d\n", ec_slave[cnt].name, el7211_pos);
                        ec_slave[cnt].PO2SOconfig = &EL7211_setup;
                    }
                }
            }

            /* configure DC options for every DC capable slave found in the list */
            // Special configuration parameters for EL7201 contoller (copied from TwinCat configuration)
            // This step should be done before config_map
            ec_dcsync0(el7211_pos, TRUE, 2000000U, 1000U);
            ec_dcsync01(el7211_pos, TRUE, 2000000U, 4000000U, 1000U);
            ec_configdc();

            ec_config_map(&IOmap);

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for (cnt = 1; cnt <= ec_slavecount; cnt++) {
                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, (int) ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                printf("         Out:%li,%4d In:%li,%4d\n",
                       (intptr_t) ec_slave[cnt].outputs, ec_slave[cnt].Obytes, (intptr_t) ec_slave[cnt].inputs,
                       ec_slave[cnt].Ibytes);
                /* check for EL2004 or EL2008 */
                if (!digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052))) {
                    digout = ec_slave[cnt].outputs;
                }
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* request OP state for all slaves */
            ec_writestate(0);
            /* activate cyclic process data */
            dorun = 1;
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;
                /* acyclic loop 5000 x 20ms = 10s */
                for (i = 1; i <= 500; i++) {
                    printf("Processdata cycle %5d , Wck %3d, DCtime %li, dt %li, O:",
                           dorun, wkc, ec_DCtime, gl_delta);
                    for (j = 0; j < oloop; j++) {
                        printf(" %2.2x", *(ec_slave[0].outputs + j));
                    }
                    printf(" I:");
                    for (j = 0; j < iloop; j++) {
                        printf(" %2.2x", *(ec_slave[0].inputs + j));
                    }
                    printf("\r");
                    fflush(stdout);
                    osal_usleep(20000);
                }
                dorun = 0;
                inOP = FALSE;
            } else {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
            // wait
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        } else {
            printf("No slaves found!\n");
        }
        printf("End redundant test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    } else {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2)) { delta = delta - cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr) {
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    int16 statusword;
    int16 control;
    int32 position;
    int32 velocity_actual;
    int n_steps = 0;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int *) ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    ec_send_processdata();
    while (1) {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0) {
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            dorun++;

            if (ec_slave[0].hasdc) {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {

/*          Output of slaveinfo -map:
            ...
              PDO mapping according to CoE :
              SM2 outputs
                 addr b   index: sub bitl data_type    name
              [0x0000.0] 0x7010:0x01 0x10 UNSIGNED16   Controlword
              [0x0002.0] 0x7010:0x06 0x20 INTEGER32    Target velocity
              SM3 inputs
                 addr b   index: sub bitl data_type    name
              [0x0006.0] 0x6000:0x11 0x20 UNSIGNED32   Position
              [0x000A.0] 0x6010:0x01 0x10 UNSIGNED16   Statusword
              [0x000C.0] 0x6010:0x07 0x20 INTEGER32    Velocity actual value */

                // TODO: read as struct
                position = get_input_int32(el7211_pos, 0);
                printf("pos: %8.8x\t", position);

                statusword = get_input_int16(el7211_pos, 4);
                printf("sts: %8.8x\t", statusword);

                velocity_actual = get_input_int32(el7211_pos, 6);
                printf("vel: %8.8x\n", velocity_actual);

                // DS402 / MDP State Machine implementation (Fig 127 of EL72x1-001x manual)
                // Adapted from https://github.com/sittner/linuxcnc-ethercat/blob/eaff89f7c9d3b7efad7b7f85eaa9d66308641dbd/src/lcec_el7211.c
                if ((statusword >> 3) & 0x01) { // status: fault
                    control = 0x80;
                    printf("Status: fault\t");
                } else if ((statusword >> 6) & 0x01) { // status: disabled
                    control = 0x06;
                    printf("Status: switch on disabled\t");
                } else if (((statusword >> 0) & 0x01) &&
                           !((statusword >> 1) & 0x01)) { // status: ready and no switched on
                    control = 0x07;
                    printf("Status: ready to switch on\t");
                } else if ((statusword >> 1) & 0x01) { // status: switched on
                    control = 0xf;
                }

                if ((statusword >> 7) & 0x01) { // status: warning
                    printf("Status: Warning\t");
                }

                set_output_int16(el7211_pos, 0, control);

                // Set velocity to 10% of max velocity during 25000 steps
                if (control == 0xf && n_steps < 2500) {
                    n_steps++;
                    set_output_int16(el7211_pos, 2, 26214); // 10 percent of max velocity
                } else {
                    set_output_int16(el7211_pos, 2, 0);
                }

            }


            ec_send_processdata();
        }
    }
}

OSAL_THREAD_FUNC ecatcheck() // void *ptr )
{
    int slave;

    while (1) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[]) {
    int ctime;

    printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");

    if (argc > 2) {
        dorun = 0;
        ctime = atoi(argv[2]);

        /* create RT thread */
        osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *) &ctime);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

        /* start acyclic part */
        redtest(argv[1]);
    } else {
        printf("Usage: red_test ifname1 cycletime\nifname = eth0 for example\ncycletime in us\n");
    }

    printf("End program\n");

    return (0);
}
