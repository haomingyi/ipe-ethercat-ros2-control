#ifndef ECAT_MOTOR_MASTER_H
#define ECAT_MOTOR_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ethercat.h"
#include <stdbool.h>

uint32_t sdo_read(int slc, uint16 index);

int ecatm_set_interface(const char *ifname);

/* Start EtherCAT and automatically enable the CiA 402 drives. */
int ecatm_init(const char *mode, int n_joints);

/* Start EtherCAT without enabling motor torque. Intended for diagnostics. */
int ecatm_init_passive(const char *mode, int n_joints);

/* Enable drives after passive initialization and target priming. */
int ecatm_enable(void);

/* Switch cyclic mode while disabled and verify 0x6061 mode feedback. */
int ecatm_switch_mode(const char *mode);

/* IPE cyclic modes require a fresh mode-specific bit-4 edge for motion. */
int ecatm_arm_cyclic_motion(void);
int ecatm_commit_cyclic_motion(void);

/* IPE CSP requires a fresh mode-specific bit-4 edge for a new motion. */
int ecatm_arm_csp_motion(void);
int ecatm_commit_csp_motion(void);

/* Disable motor torque while keeping the EtherCAT session connected. */
int ecatm_disable(void);

/* Reset a latched CiA 402 fault without enabling motor torque. */
int ecatm_fault_reset(void);

void ecatm_control(int32_t *speed_pos_torque);

void esc_get_states(int32_t* positions, int32_t* velocities, int32_t* torques);

void esc_get_status_word(uint16_t* status);

void esc_get_error_codes(uint32_t* errors);

int ecatm_get_slave_identity(int joint_index, uint32_t *vendor_id,
                             uint32_t *product_code, uint32_t *revision);

/* Read one object-dictionary entry without changing drive parameters. */
int ecatm_read_sdo(int joint_index, uint16_t index, uint8_t subindex,
                   void *data, int *size);

int ecatm_get_csp_diagnostics(int joint_index, int32_t *target,
                              int32_t *demand, int32_t *actual,
                              uint16_t *controlword, int8_t *mode_display);

bool ecatm_is_pdo_healthy(void);

int ecatm_get_bus_status(uint16_t *state, uint16_t *al_status,
                         int *actual_wkc, int *required_wkc);

const char *ecatm_status_string(uint16_t statusword);

void print_status_word();

void ecatm_stop(void);

#ifdef __cplusplus
}
#endif

#endif
