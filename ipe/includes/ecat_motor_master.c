#include "ecat_motor_master.h"
#include "ethercat_common.h"
#include "cia402_def.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

#define DEBUG_ENABLE        0

#define MOTOR_MODE_CSP      0
#define MOTOR_MODE_CSV      1
#define MOTOR_MODE_CST      2

#define IPE_VENDOR_ID       0x00041101u
#define IPE_IRGML_PRODUCT   0x00009253u
#define IPE_CSP_CONTROLWORD_RUN 0x001Fu

#define stack64k (64 * 1024)


volatile uint8_t motor_mode = MOTOR_MODE_CSV;
static volatile bool cia402_thread_running = false;
static bool cia402_thread_started = false;
static bool master_initialized = false;
static int configured_joints = 0;
static pthread_t cia402_thread;
static volatile uint16_t operation_enabled_controlword =
    CONTROLWORD_COMMAND_ENABLEOPERATION;

extern pthread_mutex_t pdo_mutex;

static bool config_sdo(uint16_t slave)
{
    if (ec_slave[slave].eep_man == IPE_VENDOR_ID &&
        ec_slave[slave].eep_id == IPE_IRGML_PRODUCT) {
        printf("Slave %u: IPE IRGML uses DC SYNC0; skipping optional 0x60C2 setup.\n",
               slave);
        return true;
    }

    uint32_t pid_data;
    int size;
    bool all_good = true;

    for (int attempt = 1; attempt <= 3; ++attempt) {
        all_good = true;

        // Set the cycle time.
        /* This IPE firmware exposes 0x60C2:01 as a 16-bit millisecond value. */
        uint16_t cycle_ms = DC_SYNC_CYCLE_MS;
        pid_data = cycle_ms;
        int write_wkc = ec_SDOwrite(slave, 0x60C2, 0x01, FALSE,
                                    sizeof(cycle_ms), &cycle_ms, EC_TIMEOUTRXM);

        // Validate PDO exchange.
        pid_data = 0;
        size = 4;
        int read_wkc = ec_SDOread(slave, 0x60C2, 0x01, FALSE, &size,
                                  &pid_data, EC_TIMEOUTRXM);
        if ((write_wkc <= 0) || (read_wkc <= 0) ||
            (pid_data != DC_SYNC_CYCLE_MS)) {
            printf("Slave %u: 0x60C2 cycle time setup failed (attempt %d/3).\n",
                   slave, attempt);
            all_good = false;
        }

        if (all_good)
            return true;

    }
    fprintf(stderr, "Slave %u: unable to verify 0x60C2 cycle time.\n", slave);
    return false;
}

uint32_t sdo_read(int slc, uint16 index) {
    uint32_t pid_data = 0;
    int size = 4;
    int wkc = ec_SDOread(slc, index, 0x00, FALSE, &size, &pid_data,
                         EC_TIMEOUTRXM);
    if (wkc <= 0)
        fprintf(stderr, "SDO read failed: slave=%d index=0x%04x.\n", slc, index);
    printf("Slave: %d, Reg: 0x%04x, Value: 0x%08x\r\n", slc, index, pid_data);
    return pid_data;
}

void sdo_config()
{
    for (int slc = FIRST_SLAVE; slc <= ec_slavecount; slc++) {
        #if USE_BRANCHER == 1
            if (slc == BRANCHER_ADDR)
                continue;
        #endif
        if (!config_sdo(slc)) {
            fprintf(stderr,
                    "Slave %d does not support optional 0x60C2:01 setup; "
                    "using EtherCAT DC SYNC0 cycle configuration instead.\n",
                    slc);
            while (EcatError)
                fprintf(stderr, "  %s\n", ec_elist2string());
        }
    }
}

static int pdo_preop_config(void) {
    for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        TCiA402PDO1600 *output = (TCiA402PDO1600 *)ec_slave[slave].outputs;
        TCiA402PDO1A00 *input = (TCiA402PDO1A00 *)ec_slave[slave].inputs;
        if (!output || !input)
            return -1;

        output->ObjControlWord = CONTROLWORD_COMMAND_DISABLEVOLTAGE;
        output->ObjTargetVelocity = 0;
        output->ObjTargetTorque = 0;
        if (motor_mode == MOTOR_MODE_CSP) {
            output->ObjModesOfOperation = CYCLIC_SYNC_POSITION_MODE;
            output->ObjTargetPosition = input->ObjPositionActualValue;
        } else if (motor_mode == MOTOR_MODE_CSV) {
            output->ObjModesOfOperation = CYCLIC_SYNC_VELOCITY_MODE;
        } else {
            output->ObjModesOfOperation = CYCLIC_SYNC_TORQUE_MODE;
        }
    }
    return 0;
}

int start_ethercat_master(void)
{
    int ret = ethercat_common_start(sdo_config, pdo_preop_config);
    if (ret < 0) {
        printf("EtherCAT master start failed with error code: %d\n", ret);
        return ret;
    }

    return ret;
}

int esc_start(const char *mode)
{

    if (!mode) {
        fprintf(stderr, "Mode is NULL\n");
        return -1;
    }

    if (!strcmp(mode, "csp")) {
        motor_mode = MOTOR_MODE_CSP;
    } else if (!strcmp(mode, "csv")) {
        motor_mode = MOTOR_MODE_CSV;
    } else if (!strcmp(mode, "cst")) {
        motor_mode = MOTOR_MODE_CST;
    } else {
        fprintf(stderr, "Unsupported mode '%s'. Use csp, csv, or cst.\n", mode);
        return -1;
    }

    printf("EtherCAT master started with mode: %s\n", mode);

    int ret = start_ethercat_master();
    if (ret < 0) {
        fprintf(stderr, "Failed to start EtherCAT master.\n");
        return ret;
    }

    printf("EtherCAT master started successfully.\n");
    return ret;

}

void esc_stop(void)
{
    printf("Stop ESC\n");
    

    TCiA402PDO1600 *output;

    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
        #endif
        pthread_mutex_lock(&pdo_mutex);
        output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;

        output->ObjControlWord = CONTROLWORD_COMMAND_SHUTDOWN;
        pthread_mutex_unlock(&pdo_mutex);
    }

    usleep(100 * 1000);  // 100 ms

    ethercat_common_stop();
}

void esc_control(int32_t *speed_pos_torque)
{
    // printf("Control = %d\n", speed_pos_torque[0]);
    if (!speed_pos_torque) {
        printf("Control input is NULL\n");
        return;
    }

    TCiA402PDO1600 *output;

    switch (motor_mode) {
        case MOTOR_MODE_CSP:
            for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
                uint16_t offset = 0;
                #if USE_BRANCHER == 1
                    if (cnt == BRANCHER_ADDR)
                        continue;
                    if (cnt > BRANCHER_ADDR)
                        offset = 1;
                #endif
                pthread_mutex_lock(&pdo_mutex);
                output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;
//                printf("modes of csp: %d\n", output->ObjModesOfOperation);

                output->ObjModesOfOperation = CYCLIC_SYNC_POSITION_MODE;
                output->ObjTargetPosition = speed_pos_torque[cnt-FIRST_SLAVE-offset];
                pthread_mutex_unlock(&pdo_mutex);
            }

            break;
        case MOTOR_MODE_CSV:
            for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
                uint16_t offset = 0;
                #if USE_BRANCHER == 1
                    if (cnt == BRANCHER_ADDR)
                        continue;
                    if (cnt > BRANCHER_ADDR)
                        offset = 1;
                #endif
                pthread_mutex_lock(&pdo_mutex);
                output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;
//                printf("modes of csv: %d\n", output->ObjModesOfOperation);

                output->ObjModesOfOperation = CYCLIC_SYNC_VELOCITY_MODE;
                output->ObjTargetVelocity = speed_pos_torque[cnt-FIRST_SLAVE-offset];
                pthread_mutex_unlock(&pdo_mutex);
            }
            break;
        case MOTOR_MODE_CST:
            for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
                uint16_t offset = 0;
                #if USE_BRANCHER == 1
                    if (cnt == BRANCHER_ADDR)
                        continue;
                    if (cnt > BRANCHER_ADDR)
                        offset = 1;
                #endif
                pthread_mutex_lock(&pdo_mutex);
                output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;
//                printf("modes of cst: %d\n", output->ObjModesOfOperation);
//                printf("force of cst: %d\n", output->ObjTargetTorque);

                output->ObjModesOfOperation = CYCLIC_SYNC_TORQUE_MODE;
                output->ObjTargetTorque = (int16_t)speed_pos_torque[cnt-FIRST_SLAVE-offset];
                pthread_mutex_unlock(&pdo_mutex);
            }
            break;

        default:
            break;
    }
}

void esc_switch_on(void)
{
    printf("Switch on\n");

    TCiA402PDO1600 *output;

    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
        #endif
        pthread_mutex_lock(&pdo_mutex);
        output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;

        output->ObjControlWord = CONTROLWORD_COMMAND_SWITCHON;
        pthread_mutex_unlock(&pdo_mutex);
    }

}

void esc_motor_run(void)
{
    printf("Run motor\n");

    TCiA402PDO1600 *output;

    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
        #endif
        pthread_mutex_lock(&pdo_mutex);
        output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;

        output->ObjControlWord = CONTROLWORD_COMMAND_ENABLEOPERATION;
        pthread_mutex_unlock(&pdo_mutex);
    }
}

void esc_motor_stop(void)
{
    printf("stop motor\n");

    TCiA402PDO1600 *output;

    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
        #endif
        pthread_mutex_lock(&pdo_mutex);
        output = (TCiA402PDO1600 *)ec_slave[cnt].outputs;

        output->ObjControlWord = CONTROLWORD_COMMAND_SHUTDOWN;
        pthread_mutex_unlock(&pdo_mutex);
    }

}

void esc_get_states(int32_t* positions, int32_t* velocities, int32_t* torques) {
    if (ec_slavecount == 0 || !positions || !velocities || !torques) {
        return;
    }

    TCiA402PDO1A00 *input;

    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        uint16_t offset = 0;
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
            if (cnt > BRANCHER_ADDR)
                offset = 1;
        #endif
        pthread_mutex_lock(&pdo_mutex);
        input = (TCiA402PDO1A00 *)ec_slave[cnt].inputs;
        positions[cnt-FIRST_SLAVE-offset] = input->ObjPositionActualValue;
        velocities[cnt-FIRST_SLAVE-offset] = input->ObjVelocityActualValue;
        torques[cnt-FIRST_SLAVE-offset] = (int32_t)input->ObjTorqueActualValue;
        // printf("moasdfasdfasdfasfdasdfasfasdfasdfasdfasfdasdfasefasefasdfaesfasdfawefasefasefasefade: %d/n", input->ObjModesOfOperationDisplay);
        pthread_mutex_unlock(&pdo_mutex);
    }

}

void esc_get_status_word(uint16_t* status) {
    if (ec_slavecount == 0 || !status) {
        return;
    }

    TCiA402PDO1A00 *input;

    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        uint16_t offset = 0;
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
            if (cnt > BRANCHER_ADDR)
                offset = 1;
        #endif
        pthread_mutex_lock(&pdo_mutex);
        input = (TCiA402PDO1A00 *)ec_slave[cnt].inputs;
        status[cnt-FIRST_SLAVE-offset] = input->ObjStatusWord;
        pthread_mutex_unlock(&pdo_mutex);
    }
}

uint16_t decodeStatusword(uint16_t statusword) {
    uint8_t bit6 = (statusword >> 6) & 0x01;
    uint8_t bit5 = (statusword >> 5) & 0x01;
    uint8_t bit3 = (statusword >> 3) & 0x01;
    uint8_t bit2 = (statusword >> 2) & 0x01;
    uint8_t bit1 = (statusword >> 1) & 0x01;
    uint8_t bit0 = statusword & 0x01;

    if (bit6 == 0 && bit3 == 0 && bit2 == 0 && bit1 == 0 && bit0 == 0) {
        return STATUSWORD_STATE_NOTREADYTOSWITCHON;
    } else if (bit6 == 1 && bit3 == 0 && bit2 == 0 && bit1 == 0 && bit0 == 0) {
        return STATUSWORD_STATE_SWITCHEDONDISABLED;
    } else if (bit6 == 0 && bit5 == 1 && bit3 == 0 && bit2 == 0 && bit1 == 0 && bit0 == 1) {
        return STATUSWORD_STATE_READYTOSWITCHON;
    } else if (bit6 == 0 && bit5 == 1 && bit3 == 0 && bit2 == 0 && bit1 == 1 && bit0 == 1) {
        return STATUSWORD_STATE_SWITCHEDON;
    } else if (bit6 == 0 && bit5 == 1 && bit3 == 0 && bit2 == 1 && bit1 == 1 && bit0 == 1) {
        return STATUSWORD_STATE_OPERATIONENABLED;
    } else if (bit6 == 0 && bit5 == 0 && bit3 == 0 && bit2 == 1 && bit1 == 1 && bit0 == 1) {
        return STATUSWORD_STATE_QUICKSTOPACTIVE;
    } else if (bit6 == 0 && bit3 == 1 && bit2 == 1 && bit1 == 1 && bit0 == 1) {
        return STATUSWORD_STATE_FAULTREACTIONACTIVE;
    } else if (bit6 == 0 && bit3 == 1 && bit2 == 0 && bit1 == 0 && bit0 == 0) {
        return STATUSWORD_STATE_FAULT;
    } else {
        return 0xFFFF; // Unknown state.
    }
}

void decode_status_word(uint16_t status) {
    // Print the binary status-word representation.
    printf("Status Word (Binary): ");
    for (int i = 15; i >= 0; i--) {
        printf("%d", (status >> i) & 0x01);
    }
    printf("\n");

    // Decode and print each relevant bit.
    printf("Ready to switch on: %s\n", (status & (1 << 0)) ? "Yes" : "No");
    printf("Switched on: %s\n", (status & (1 << 1)) ? "Yes" : "No");
    printf("Operation enabled: %s\n", (status & (1 << 2)) ? "Yes" : "No");
    printf("Fault: %s\n", (status & (1 << 3)) ? "Yes" : "No");
    printf("Voltage enabled: %s\n", (status & (1 << 4)) ? "Yes" : "No");
    printf("Quick stop: %s\n", (status & (1 << 5)) ? "Yes" : "No");
    printf("Switched on disabled: %s\n", (status & (1 << 6)) ? "Yes" : "No");
    printf("Warning: %s\n", (status & (1 << 7)) ? "Yes" : "No");
}

const char* cia402_state_to_string(uint16_t state) {
    switch (state) {
        case STATUSWORD_STATE_NOTREADYTOSWITCHON:    return "Not ready to switch on";
        case STATUSWORD_STATE_SWITCHEDONDISABLED:    return "Switch on disabled";
        case STATUSWORD_STATE_READYTOSWITCHON:       return "Ready to switch on";
        case STATUSWORD_STATE_SWITCHEDON:            return "Switched on";
        case STATUSWORD_STATE_OPERATIONENABLED:      return "Operation enabled";
        case STATUSWORD_STATE_QUICKSTOPACTIVE:       return "Quick stop active";
        case STATUSWORD_STATE_FAULTREACTIONACTIVE:   return "Fault reaction active";
        case STATUSWORD_STATE_FAULT:                 return "Fault";
        default:                                     return "Unknown";
    }
}

void print_status_word() {
    uint16_t status[EC_MAXSLAVE] = {0};
    esc_get_status_word(status);
    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
        uint16_t offset = 0;
        #if USE_BRANCHER == 1
            if (cnt == BRANCHER_ADDR)
                continue;
            if (cnt > BRANCHER_ADDR)
                offset = 1;
        #endif
        printf("Command: %s\n", cia402_state_to_string(decodeStatusword(status[cnt-FIRST_SLAVE-offset])));
        // decode_status_word(status[cnt-FIRST_SLAVE-offset]);
    }
}

void control_word_write(uint16_t slave_num, uint16_t contorl_word) {

    TCiA402PDO1600 *output;

    pthread_mutex_lock(&pdo_mutex);
    output = (TCiA402PDO1600 *)ec_slave[slave_num].outputs;

    output->ObjControlWord = contorl_word;
    pthread_mutex_unlock(&pdo_mutex);
    
}

OSAL_THREAD_FUNC cia402_FSM(void *ptr) {
    (void) ptr;

    uint16_t status[EC_MAXSLAVE] = {0};

    while(cia402_thread_running) {
        esc_get_status_word(status);

        for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; cnt++) {
            uint16_t offset = 0;
            #if USE_BRANCHER == 1
                if (cnt == BRANCHER_ADDR)
                    continue;
                if (cnt > BRANCHER_ADDR)
                    offset = 1;
            #endif
            switch(decodeStatusword(status[cnt-FIRST_SLAVE-offset])) {
                case STATUSWORD_STATE_SWITCHEDONDISABLED:
                    printf("Command: %s\n", cia402_state_to_string(STATUSWORD_STATE_SWITCHEDONDISABLED));
                    control_word_write(cnt, CONTROLWORD_COMMAND_SHUTDOWN);
                    break;
                case STATUSWORD_STATE_READYTOSWITCHON:
                    printf("Command: %s\n", cia402_state_to_string(STATUSWORD_STATE_READYTOSWITCHON));
                    control_word_write(cnt, CONTROLWORD_COMMAND_SWITCHON);
                    break;
                case STATUSWORD_STATE_SWITCHEDON:
                    printf("Command: %s\n", cia402_state_to_string(STATUSWORD_STATE_SWITCHEDON));
                    control_word_write(cnt, CONTROLWORD_COMMAND_ENABLEOPERATION);
                    break;
                case STATUSWORD_STATE_OPERATIONENABLED:
                    /* IPE cyclic modes use mode-specific bit 4 as a motion trigger.
                     * The requested run word is controlled by the motion
                     * arming API so every move receives a deliberate edge. */
                    if (ec_slave[cnt].eep_man == IPE_VENDOR_ID &&
                        ec_slave[cnt].eep_id == IPE_IRGML_PRODUCT)
                        control_word_write(cnt, operation_enabled_controlword);
                    else
                        control_word_write(cnt, CONTROLWORD_COMMAND_ENABLEOPERATION);
                    break;
                case STATUSWORD_STATE_FAULT:
                    fprintf(stderr,
                            "Slave %u is in FAULT. Automatic fault reset is disabled.\n",
                            cnt);
                    control_word_write(cnt, CONTROLWORD_COMMAND_DISABLEVOLTAGE);
                    break;
                default:
                    break;

            }
        }
        osal_usleep(10000);
    }
}

static void *cia402_pthread_entry(void *ptr) {
    cia402_FSM(ptr);
    return NULL;
}

static void stop_cia402_thread(void) {
    cia402_thread_running = false;
    if (cia402_thread_started) {
        pthread_join(cia402_thread, NULL);
        cia402_thread_started = false;
    }
}

int ecatm_set_interface(const char *ifname) {
    return ethercat_common_set_interface(ifname);
}

static int motor_slave_count(void) {
    int count = 0;
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        ++count;
    }
    return count;
}

static int ecatm_init_internal(const char *mode, int n_joints, bool auto_enable) {
    if (master_initialized) {
        fprintf(stderr, "EtherCAT motor master is already initialized.\n");
        return -10;
    }
    if (n_joints <= 0 || n_joints >= EC_MAXSLAVE) {
        fprintf(stderr, "Invalid joint count: %d.\n", n_joints);
        return -11;
    }

    int ret = esc_start(mode);
    if (ret < 0)
        return ret;

    int found_motors = motor_slave_count();
    if (found_motors != n_joints) {
        fprintf(stderr, "ERROR: expected %d motor(s), but found %d.\n",
                n_joints, found_motors);
        esc_stop();
        return -12;
    }

    for (int slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        if (ec_slave[slave].Obits != (int)(sizeof(TCiA402PDO1600) * 8) ||
            ec_slave[slave].Ibits != (int)(sizeof(TCiA402PDO1A00) * 8)) {
            fprintf(stderr,
                    "Slave %d PDO size mismatch: expected %zu/%zu bits, got %d/%d.\n",
                    slave, sizeof(TCiA402PDO1600) * 8,
                    sizeof(TCiA402PDO1A00) * 8,
                    ec_slave[slave].Obits, ec_slave[slave].Ibits);
            esc_stop();
            return -13;
        }
    }

    configured_joints = n_joints;
    master_initialized = true;

    if(motor_mode==MOTOR_MODE_CSP) {
        int32_t init_pos[EC_MAXSLAVE] = {0};
        int32_t init_vel[EC_MAXSLAVE] = {0};
        int32_t init_tor[EC_MAXSLAVE] = {0};
        esc_get_states(init_pos, init_vel, init_tor);
        esc_control(init_pos);
    }
    else if(motor_mode==MOTOR_MODE_CST) {
        int32_t init_torque[EC_MAXSLAVE] = {0};
        esc_control(init_torque);
    }

    if (auto_enable) {
        ret = ecatm_enable();
        if (ret < 0)
            ecatm_stop();
        return ret;
    }
    return 0;
}

int ecatm_init(const char *mode, int n_joints) {
    return ecatm_init_internal(mode, n_joints, true);
}

int ecatm_init_passive(const char *mode, int n_joints) {
    return ecatm_init_internal(mode, n_joints, false);
}

int ecatm_enable(void) {
    if (!master_initialized)
        return -20;
    if (cia402_thread_started)
        return 0;

    operation_enabled_controlword = CONTROLWORD_COMMAND_ENABLEOPERATION;

    int healthy_cycles = 0;
    for (int attempt = 0; attempt < 300; ++attempt) {
        if (esc_enter_op && !esc_pdo_error) {
            if (++healthy_cycles >= 20)
                break;
        } else {
            healthy_cycles = 0;
        }
        osal_usleep(10000);
    }
    if (healthy_cycles < 20) {
        fprintf(stderr,
                "Cannot enable drive: PDO did not remain healthy for 200 ms.\n");
        return -21;
    }

    cia402_thread_running = true;
    if (pthread_create(&cia402_thread, NULL, cia402_pthread_entry, NULL) != 0) {
        cia402_thread_running = false;
        return -22;
    }
    cia402_thread_started = true;

    for (int attempt = 0; attempt < 500; ++attempt) {
        uint16_t status[EC_MAXSLAVE] = {0};
        esc_get_status_word(status);
        bool all_enabled = true;
        for (int i = 0; i < configured_joints; ++i) {
            uint16_t state = decodeStatusword(status[i]);
            if (state == STATUSWORD_STATE_FAULT) {
                uint32_t errors[EC_MAXSLAVE] = {0};
                esc_get_error_codes(errors);
                fprintf(stderr,
                        "Drive %d reported FAULT while enabling: status=0x%04x "
                        "error=0x%08x.\n",
                        i, status[i], errors[i]);
                stop_cia402_thread();
                esc_motor_stop();
                return -23;
            }
            if (state != STATUSWORD_STATE_OPERATIONENABLED)
                all_enabled = false;
        }
        if (all_enabled)
            return 0;
        osal_usleep(10000);
    }
    fprintf(stderr, "Timed out waiting for CiA 402 Operation Enabled.\n");
    stop_cia402_thread();
    esc_motor_stop();
    return -24;

}

static int parse_cyclic_mode(const char *mode, uint8_t *internal_mode,
                             int8_t *cia402_mode) {
    if (!mode || !internal_mode || !cia402_mode)
        return -1;
    if (!strcmp(mode, "csp")) {
        *internal_mode = MOTOR_MODE_CSP;
        *cia402_mode = CYCLIC_SYNC_POSITION_MODE;
    } else if (!strcmp(mode, "csv")) {
        *internal_mode = MOTOR_MODE_CSV;
        *cia402_mode = CYCLIC_SYNC_VELOCITY_MODE;
    } else if (!strcmp(mode, "cst")) {
        *internal_mode = MOTOR_MODE_CST;
        *cia402_mode = CYCLIC_SYNC_TORQUE_MODE;
    } else {
        return -1;
    }
    return 0;
}

int ecatm_switch_mode(const char *mode) {
    uint8_t requested_internal = 0;
    int8_t requested_cia402 = 0;
    if (!master_initialized ||
        parse_cyclic_mode(mode, &requested_internal, &requested_cia402) < 0)
        return -25;
    if (!ecatm_is_pdo_healthy())
        return -26;

    ecatm_disable();
    int32_t positions[EC_MAXSLAVE] = {0};
    int32_t velocities[EC_MAXSLAVE] = {0};
    int32_t torques[EC_MAXSLAVE] = {0};
    esc_get_states(positions, velocities, torques);

    int joint = 0;
    pthread_mutex_lock(&pdo_mutex);
    motor_mode = requested_internal;
    operation_enabled_controlword = CONTROLWORD_COMMAND_ENABLEOPERATION;
    for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        TCiA402PDO1600 *output =
            (TCiA402PDO1600 *)ec_slave[slave].outputs;
        if (!output) {
            pthread_mutex_unlock(&pdo_mutex);
            return -27;
        }
        output->ObjControlWord = CONTROLWORD_COMMAND_DISABLEVOLTAGE;
        output->ObjTargetPosition = positions[joint++];
        output->ObjTargetVelocity = 0;
        output->ObjTargetTorque = 0;
        output->ObjModesOfOperation = requested_cia402;
    }
    pthread_mutex_unlock(&pdo_mutex);

    for (int attempt = 0; attempt < 100; ++attempt) {
        if (!ecatm_is_pdo_healthy())
            return -28;
        bool all_selected = true;
        pthread_mutex_lock(&pdo_mutex);
        for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
            if (slave == BRANCHER_ADDR)
                continue;
#endif
            TCiA402PDO1A00 *input =
                (TCiA402PDO1A00 *)ec_slave[slave].inputs;
            if (!input || input->ObjModesOfOperationDisplay != requested_cia402)
                all_selected = false;
        }
        pthread_mutex_unlock(&pdo_mutex);
        if (all_selected) {
            printf("Cyclic mode switched to %s (0x6061=%d); drive remains disabled.\n",
                   mode, requested_cia402);
            return 0;
        }
        osal_usleep(10000);
    }
    fprintf(stderr, "Timed out waiting for cyclic mode '%s' feedback.\n", mode);
    return -29;
}

int ecatm_arm_cyclic_motion(void) {
    if (!master_initialized || esc_pdo_error || !cia402_thread_started)
        return -30;

    uint16_t status[EC_MAXSLAVE] = {0};
    esc_get_status_word(status);
    for (int i = 0; i < configured_joints; ++i) {
        if (decodeStatusword(status[i]) != STATUSWORD_STATE_OPERATIONENABLED)
            return -31;
    }

    operation_enabled_controlword = CONTROLWORD_COMMAND_ENABLEOPERATION;
    for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        control_word_write(slave, CONTROLWORD_COMMAND_ENABLEOPERATION);
    }
    osal_usleep(30000);
    return 0;
}

int ecatm_commit_cyclic_motion(void) {
    if (!master_initialized || esc_pdo_error || !cia402_thread_started)
        return -32;

    operation_enabled_controlword = IPE_CSP_CONTROLWORD_RUN;
    for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        if (ec_slave[slave].eep_man == IPE_VENDOR_ID &&
            ec_slave[slave].eep_id == IPE_IRGML_PRODUCT)
            control_word_write(slave, IPE_CSP_CONTROLWORD_RUN);
        else
            control_word_write(slave, CONTROLWORD_COMMAND_ENABLEOPERATION);
    }
    return 0;
}

int ecatm_arm_csp_motion(void) {
    if (!master_initialized || motor_mode != MOTOR_MODE_CSP ||
        esc_pdo_error || !cia402_thread_started)
        return -30;
    return ecatm_arm_cyclic_motion();
}

int ecatm_commit_csp_motion(void) {
    if (!master_initialized || motor_mode != MOTOR_MODE_CSP ||
        esc_pdo_error || !cia402_thread_started)
        return -32;

    return ecatm_commit_cyclic_motion();
}

int ecatm_fault_reset(void) {
    if (!master_initialized || esc_pdo_error)
        return -40;

    uint16_t status[EC_MAXSLAVE] = {0};
    esc_get_status_word(status);
    bool has_fault = false;
    for (int i = 0; i < configured_joints; ++i) {
        if (decodeStatusword(status[i]) == STATUSWORD_STATE_FAULT)
            has_fault = true;
    }
    if (!has_fault)
        return 0;

    for (int attempt = 1; attempt <= 3; ++attempt) {
        /* Guarantee a fresh rising edge on CiA 402 fault-reset bit 7. */
        for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
            if (slave == BRANCHER_ADDR)
                continue;
#endif
            control_word_write(slave, CONTROLWORD_COMMAND_DISABLEVOLTAGE);
        }
        osal_usleep(100000);
        for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
            if (slave == BRANCHER_ADDR)
                continue;
#endif
            control_word_write(slave, CONTROLWORD_COMMAND_FAULTRESET);
        }
        osal_usleep(100000);
        for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
            if (slave == BRANCHER_ADDR)
                continue;
#endif
            control_word_write(slave, CONTROLWORD_COMMAND_DISABLEVOLTAGE);
        }
        osal_usleep(300000);

        memset(status, 0, sizeof(status));
        esc_get_status_word(status);
        bool fault_remains = false;
        for (int i = 0; i < configured_joints; ++i) {
            if (decodeStatusword(status[i]) == STATUSWORD_STATE_FAULT)
                fault_remains = true;
        }
        if (!fault_remains) {
            if (attempt > 1)
                printf("Fault reset succeeded on attempt %d/3.\n", attempt);
            return 0;
        }
        fprintf(stderr, "Fault reset attempt %d/3 did not clear the drive.\n",
                attempt);
    }
    return -41;
}

int ecatm_disable(void) {
    if (!master_initialized)
        return -50;
    operation_enabled_controlword = CONTROLWORD_COMMAND_ENABLEOPERATION;
    stop_cia402_thread();
    for (uint16_t slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        control_word_write(slave, CONTROLWORD_COMMAND_DISABLEVOLTAGE);
    }
    osal_usleep(100000);
    return 0;
}

void ecatm_control(int32_t *speed_pos_torque) {
    if (!master_initialized || esc_pdo_error)
        return;
    esc_control(speed_pos_torque);
}

void ecatm_stop(void) {
    if (!master_initialized)
        return;
    stop_cia402_thread();
    esc_stop();
    master_initialized = false;
    configured_joints = 0;

}

void esc_get_error_codes(uint32_t* errors) {
    if (ec_slavecount == 0 || !errors)
        return;
    int joint = 0;
    for (uint16_t cnt = FIRST_SLAVE; cnt <= ec_slavecount; ++cnt) {
#if USE_BRANCHER == 1
        if (cnt == BRANCHER_ADDR)
            continue;
#endif
        pthread_mutex_lock(&pdo_mutex);
        TCiA402PDO1A00 *input = (TCiA402PDO1A00 *)ec_slave[cnt].inputs;
        errors[joint++] = input ? input->ErrorCode : 0xffffffffu;
        pthread_mutex_unlock(&pdo_mutex);
    }
}

int ecatm_get_slave_identity(int joint_index, uint32_t *vendor_id,
                             uint32_t *product_code, uint32_t *revision) {
    if (!master_initialized || joint_index < 0)
        return -1;
    int joint = 0;
    for (int slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        if (joint++ == joint_index) {
            if (vendor_id) *vendor_id = ec_slave[slave].eep_man;
            if (product_code) *product_code = ec_slave[slave].eep_id;
            if (revision) *revision = ec_slave[slave].eep_rev;
            return 0;
        }
    }
    return -2;
}

int ecatm_read_sdo(int joint_index, uint16_t index, uint8_t subindex,
                   void *data, int *size) {
    if (!master_initialized || joint_index < 0 || !data || !size || *size <= 0)
        return -1;

    int joint = 0;
    for (int slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        if (joint++ != joint_index)
            continue;

        pthread_mutex_lock(&pdo_mutex);
        int wkc = ec_SDOread((uint16_t)slave, index, subindex, FALSE,
                             size, data, EC_TIMEOUTRXM);
        pthread_mutex_unlock(&pdo_mutex);
        return wkc > 0 ? 0 : -2;
    }
    return -3;
}

int ecatm_get_csp_diagnostics(int joint_index, int32_t *target,
                              int32_t *demand, int32_t *actual,
                              uint16_t *controlword, int8_t *mode_display) {
    if (!master_initialized || joint_index < 0)
        return -1;

    int joint = 0;
    for (int slave = FIRST_SLAVE; slave <= ec_slavecount; ++slave) {
#if USE_BRANCHER == 1
        if (slave == BRANCHER_ADDR)
            continue;
#endif
        if (joint++ != joint_index)
            continue;

        pthread_mutex_lock(&pdo_mutex);
        TCiA402PDO1600 *output = (TCiA402PDO1600 *)ec_slave[slave].outputs;
        TCiA402PDO1A00 *input = (TCiA402PDO1A00 *)ec_slave[slave].inputs;
        if (!output || !input) {
            pthread_mutex_unlock(&pdo_mutex);
            return -2;
        }
        if (target) *target = output->ObjTargetPosition;
        if (demand) *demand = input->ObjPositionDemandValue;
        if (actual) *actual = input->ObjPositionActualValue;
        if (controlword) *controlword = output->ObjControlWord;
        if (mode_display) *mode_display = input->ObjModesOfOperationDisplay;
        pthread_mutex_unlock(&pdo_mutex);
        return 0;
    }
    return -3;
}

bool ecatm_is_pdo_healthy(void) {
    return master_initialized && esc_enter_op && !esc_pdo_error;
}

int ecatm_get_bus_status(uint16_t *state, uint16_t *al_status,
                         int *actual_wkc, int *required_wkc) {
    if (!master_initialized)
        return -1;
    return ethercat_common_get_bus_status(FIRST_SLAVE, state, al_status,
                                          actual_wkc, required_wkc);
}

const char *ecatm_status_string(uint16_t statusword) {
    return cia402_state_to_string(decodeStatusword(statusword));

}
