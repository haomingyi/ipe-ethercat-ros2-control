#ifndef ETHERCAT_COMMON_H
#define ETHERCAT_COMMON_H

/* Default learning setup: PC -> one IPE EtherCAT joint, no brancher. */
#define USE_BRANCHER 0
#define BRANCHER_ADDR 2
#define DEFAULT_ECAT_IFNAME "enp130s0"

#if USE_BRANCHER == 1
    #define FIRST_SLAVE 3 //when with two legs
    //#define FIRST_SLAVE 2 //when with arms
#else
    #define FIRST_SLAVE 1
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include "ethercat.h"
#include <stdbool.h>

//#define DC_SYNC_CYCLE_MS 1
#define DC_SYNC_CYCLE_MS 1

typedef void (*esc_sdo_config_t)(void);
typedef int (*esc_preop_config_t)(void);

extern volatile bool esc_enter_op;
extern volatile bool esc_pdo_error;

int ethercat_common_start(esc_sdo_config_t sdo_config,
                          esc_preop_config_t preop_config);
void ethercat_common_stop(void);
int ethercat_common_set_interface(const char *ifname);
const char *ethercat_common_get_interface(void);
int ethercat_common_get_bus_status(uint16_t slave, uint16_t *state,
                                   uint16_t *al_status, int *actual_wkc,
                                   int *required_wkc);

#ifdef __cplusplus
}
#endif

#endif
