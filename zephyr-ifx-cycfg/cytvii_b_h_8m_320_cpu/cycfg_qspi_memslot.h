#ifndef CYCFG_QSPI_MEMSLOT_H
#define CYCFG_QSPI_MEMSLOT_H
#include "cy_smif_memslot.h"

#define CY_SMIF_CFG_TOOL_VERSION           (460)

#define CY_SMIF_DRV_VERSION_REQUIRED       (100)

#if !defined(CY_SMIF_DRV_VERSION)
    #define CY_SMIF_DRV_VERSION            (100)
#endif

#if (CY_SMIF_DRV_VERSION_REQUIRED > CY_SMIF_DRV_VERSION)
   #error The QSPI Configurator requires a newer version of the PDL.
#endif

typedef cy_stc_smif_mem_config_t cy_serial_flash_mem_config_t;
typedef cy_stc_smif_block_config_t cy_serial_flash_block_config_t;

#define CY_SMIF_DEVICE_NUM 2

#define SMIF0_DESELECT_DELAY 7

#ifdef SMIF_MODE_HYPERBUS /* HyperBus devices */
extern cy_stc_smif_hbmem_device_config_t deviceCfg_S26KL512S_SlaveSlot_0;
extern cy_stc_smif_mem_config_t S26KL512S_SlaveSlot_0;

extern cy_stc_smif_hbmem_device_config_t deviceCfg_S27KL0641_SlaveSlot_1;
extern cy_stc_smif_mem_config_t S27KL0641_SlaveSlot_1;

#else /* Dual QSPI devices */

extern cy_stc_smif_mem_cmd_t S25FL256S_readCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_writeEnCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_writeDisCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_eraseCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_chipEraseCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_programCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_readStsRegQeCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_readStsRegWipCmd;
extern cy_stc_smif_mem_cmd_t S25FL256S_writeStsRegQeCmd;

extern cy_stc_smif_mem_device_cfg_t deviceCfg_S25FL256S_SlaveSlot_0;
extern cy_stc_smif_mem_config_t S25FL256S_SlaveSlot_0;

extern cy_stc_smif_mem_device_cfg_t deviceCfg_S25FL256S_SlaveSlot_1;
extern cy_stc_smif_mem_config_t S25FL256S_SlaveSlot_1;
#endif

extern cy_stc_smif_mem_config_t* smifMemConfigs[CY_SMIF_DEVICE_NUM];
extern cy_stc_smif_block_config_t smif0BlockConfig;
extern cy_stc_smif_config_t SMIF0_config;

#endif /* CYCFG_QSPI_MEMSLOT_H */
