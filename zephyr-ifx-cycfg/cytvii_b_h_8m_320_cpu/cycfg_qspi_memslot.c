#include "cycfg_qspi_memslot.h"

/* Mode-dependent SMIF config */
#ifdef SMIF_MODE_HYPERBUS
cy_stc_smif_hbmem_device_config_t deviceCfg_S26KL512S_SlaveSlot_0 =
{
    .xipReadCmd = CY_SMIF_HB_READ_WRAPPED_BURST,
    .xipWriteCmd = CY_SMIF_HB_WRITE_WRAPPED_BURST,
    .hbDevType = CY_SMIF_HB_FLASH,
    .memSize = 0x4000000,   /* 64MB */
    .dummyCycles = 16,      /* factory default NVCR latency */
};

cy_stc_smif_mem_config_t S26KL512S_SlaveSlot_0 =
{
    /* Determines the slot number where the memory device is placed. */
    .slaveSelect = CY_SMIF_SLAVE_SELECT_0,
    /* Flags. */
#if (CY_IP_MXSMIF_VERSION >= 2)
    .flags = CY_SMIF_FLAG_SMIF_REV_3 | CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN | CY_SMIF_FLAG_MERGE_ENABLE,
#else
    .flags = CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN | CY_SMIF_FLAG_MERGE_ENABLE,
#endif /* CY_IP_MXSMIF_VERSION */
    /* The data-line selection options for a slave device. */
    .dataSelect = CY_SMIF_DATA_SEL0,
    /* The base address the memory slave is mapped to in the PSoC memory map.
    Valid when the memory-mapped mode is enabled. */
    .baseAddress = 0x60000000U,
    /* The size allocated in the PSoC memory map, for the memory slave device.
    The size is allocated from the base address. Valid when the memory mapped mode is enabled. */
    .memMappedSize = 0x4000000U,
    /* If this memory device is one of the devices in the dual quad SPI configuration.
    Valid when the memory mapped mode is enabled. */
    .dualQuadSlots = 0,
    /* The configuration of the device. */
    .deviceCfg = 0,
#if (CY_IP_MXSMIF_VERSION >= 2)
    /** Continous transfer merge timeout.
     * After this period the memory device is deselected. A later transfer, even from a
     * continuous address, starts with the overhead phases (command, address, mode, dummy cycles).
     * This configuration parameter is available for CAT1B devices. */
    .mergeTimeout = CY_SMIF_MERGE_TIMEOUT_1_CYCLE,
    .hbdeviceCfg = &deviceCfg_S26KL512S_SlaveSlot_0
#else
#error Features used by this file require CY_IP_MXSMIF_VERSION >= 2.
#endif /* CY_IP_MXSMIF_VERSION */
};

cy_stc_smif_hbmem_device_config_t deviceCfg_S27KL0641_SlaveSlot_1 =
{
    .xipReadCmd = CY_SMIF_HB_READ_WRAPPED_BURST,
    .xipWriteCmd = CY_SMIF_HB_WRITE_WRAPPED_BURST,
    .hbDevType = CY_SMIF_HB_SRAM,
    .memSize = 0x800000,    /* 8MB */
    .dummyCycles = 12,      /* CR0 default: 6clk x 2(fixed) = 12 */
};

cy_stc_smif_mem_config_t S27KL0641_SlaveSlot_1 =
{
    .slaveSelect = CY_SMIF_SLAVE_SELECT_1,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .flags = CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN,
#else
    .flags = CY_SMIF_FLAG_ALL_DISABLED,
#endif
    .dataSelect = CY_SMIF_DATA_SEL0,
    .baseAddress = 0x64000000U,    /* after S26KL512S 64MB region */
    .memMappedSize = 0x800000U,    /* 8MB */
    .dualQuadSlots = 0,
    .deviceCfg = 0,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .mergeTimeout = CY_SMIF_MERGE_TIMEOUT_1_CYCLE,
    .hbdeviceCfg = &deviceCfg_S27KL0641_SlaveSlot_1
#endif
};

cy_stc_smif_mem_config_t* smifMemConfigs[CY_SMIF_DEVICE_NUM] = {
    &S26KL512S_SlaveSlot_0,
    &S27KL0641_SlaveSlot_1,
};

cy_stc_smif_config_t SMIF0_config =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = SMIF0_DESELECT_DELAY,
    .rxClockSel = CY_SMIF_SEL_INVERTED_SPHB_RWDS_CLK,
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
    .delayTapEnable = CY_SMIF_DELAY_TAP_ENABLE,
    .delayLineSelect = CY_SMIF_1_NEW_SEL_PER_TAP,
};

#else /* SMIF_MODE_DUAL_QSPI */

/* Commands for S25FL256S interfaced in Dual QSPI mode */
cy_stc_smif_mem_cmd_t S25FL256S_readCmd =
{
    .command = 0xECU,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_QUAD,
    .mode = 0x01U,
    .modeWidth = CY_SMIF_WIDTH_QUAD,
    .dummyCycles = 5U,
    .dataWidth = CY_SMIF_WIDTH_QUAD,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_PRESENT_1BYTE,
    .modePresence = CY_SMIF_PRESENT_1BYTE,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_writeEnCmd =
{
    .command = 0x06U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_writeDisCmd =
{
    .command = 0x04U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_eraseCmd =
{
    .command = 0xDCU,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_chipEraseCmd =
{
    .command = 0x60U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_programCmd =
{
    .command = 0x34U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_QUAD,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_readStsRegQeCmd =
{
    .command = 0x35U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_readStsRegWipCmd =
{
    .command = 0x05U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_cmd_t S25FL256S_writeStsRegQeCmd =
{
    .command = 0x01U,
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    .mode = 0xFFFFFFFFU,
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    .dummyCycles = 0U,
    .dataWidth = CY_SMIF_WIDTH_SINGLE,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .dataRate = CY_SMIF_SDR,
    .dummyCyclesPresence = CY_SMIF_NOT_PRESENT,
    .modePresence = CY_SMIF_NOT_PRESENT,
    .modeH = 0x00,
    .modeRate = CY_SMIF_SDR,
    .addrRate = CY_SMIF_SDR,
    .cmdPresence = CY_SMIF_PRESENT_1BYTE,
    .commandH = 0x00,
    .cmdRate = CY_SMIF_SDR,
#endif
};

cy_stc_smif_mem_device_cfg_t deviceCfg_S25FL256S_SlaveSlot_0 =
{
    .numOfAddrBytes = 0x04U,
    .memSize = 0x2000000U,      /* 32MB */
    .readCmd = &S25FL256S_readCmd,
    .writeEnCmd = &S25FL256S_writeEnCmd,
    .writeDisCmd = &S25FL256S_writeDisCmd,
    .eraseCmd = &S25FL256S_eraseCmd,
    .eraseSize = 0x0040000U,    /* 256KB sectors */
    .chipEraseCmd = &S25FL256S_chipEraseCmd,
    .programCmd = &S25FL256S_programCmd,
    .programSize = 0x0000100U,  /* 256B page */
    .readStsRegQeCmd = &S25FL256S_readStsRegQeCmd,
    .readStsRegWipCmd = &S25FL256S_readStsRegWipCmd,
    .writeStsRegQeCmd = &S25FL256S_writeStsRegQeCmd,
    .stsRegBusyMask = 0x01U,
    .stsRegQuadEnableMask = 0x02U,
    .eraseTime = 2600U,
    .chipEraseTime = 330000U,
    .programTime = 750U,
#if (CY_SMIF_DRV_VERSION_MAJOR > 1) || (CY_SMIF_DRV_VERSION_MINOR >= 50)
    .hybridRegionCount = 0U,
    .hybridRegionInfo = 0,
#endif
    .readLatencyCmd = 0,
    .writeLatencyCmd = 0,
    .latencyCyclesRegAddr = 0x00U,
    .latencyCyclesMask = 0x00U,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .octalDDREnableSeq = 0,
    .readStsRegOeCmd = 0,
    .writeStsRegOeCmd = 0,
    .stsRegOctalEnableMask = 0x00U,
    .octalEnableRegAddr = 0x00U,
    .freq_of_operation = CY_SMIF_100MHZ_OPERATION,
#endif
};

cy_stc_smif_mem_device_cfg_t deviceCfg_S25FL256S_SlaveSlot_1 =
{
    .numOfAddrBytes = 0x04U,
    .memSize = 0x2000000U,
    .readCmd = &S25FL256S_readCmd,
    .writeEnCmd = &S25FL256S_writeEnCmd,
    .writeDisCmd = &S25FL256S_writeDisCmd,
    .eraseCmd = &S25FL256S_eraseCmd,
    .eraseSize = 0x0040000U,
    .chipEraseCmd = &S25FL256S_chipEraseCmd,
    .programCmd = &S25FL256S_programCmd,
    .programSize = 0x0000100U,
    .readStsRegQeCmd = &S25FL256S_readStsRegQeCmd,
    .readStsRegWipCmd = &S25FL256S_readStsRegWipCmd,
    .writeStsRegQeCmd = &S25FL256S_writeStsRegQeCmd,
    .stsRegBusyMask = 0x01U,
    .stsRegQuadEnableMask = 0x02U,
    .eraseTime = 2600U,
    .chipEraseTime = 330000U,
    .programTime = 750U,
#if (CY_SMIF_DRV_VERSION_MAJOR > 1) || (CY_SMIF_DRV_VERSION_MINOR >= 50)
    .hybridRegionCount = 0U,
    .hybridRegionInfo = 0,
#endif
    .readLatencyCmd = 0,
    .writeLatencyCmd = 0,
    .latencyCyclesRegAddr = 0x00U,
    .latencyCyclesMask = 0x00U,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .octalDDREnableSeq = 0,
    .readStsRegOeCmd = 0,
    .writeStsRegOeCmd = 0,
    .stsRegOctalEnableMask = 0x00U,
    .octalEnableRegAddr = 0x00U,
    .freq_of_operation = CY_SMIF_100MHZ_OPERATION,
#endif
};

/* SEL0: DQ0-DQ3 */
cy_stc_smif_mem_config_t S25FL256S_SlaveSlot_0 =
{
    .slaveSelect = CY_SMIF_SLAVE_SELECT_0,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .flags = CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN,
#else
    .flags = CY_SMIF_FLAG_ALL_DISABLED,
#endif
    .dataSelect = CY_SMIF_DATA_SEL0,
    .baseAddress = 0x60000000U,
    .memMappedSize = 0x2000000U,    /* 32MB */
    .dualQuadSlots = CY_SMIF_SLAVE_SELECT_0 | CY_SMIF_SLAVE_SELECT_1,
    .deviceCfg = &deviceCfg_S25FL256S_SlaveSlot_0,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .mergeTimeout = CY_SMIF_MERGE_TIMEOUT_1_CYCLE,
#endif
};

/* SEL1: DQ4-DQ7 */
cy_stc_smif_mem_config_t S25FL256S_SlaveSlot_1 =
{
    .slaveSelect = CY_SMIF_SLAVE_SELECT_1,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .flags = CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN,
#else
    .flags = CY_SMIF_FLAG_ALL_DISABLED,
#endif
    .dataSelect = CY_SMIF_DATA_SEL2,
    .baseAddress = 0x62000000U,
    .memMappedSize = 0x2000000U,    /* 32MB */
    .dualQuadSlots = CY_SMIF_SLAVE_SELECT_0 | CY_SMIF_SLAVE_SELECT_1,
    .deviceCfg = &deviceCfg_S25FL256S_SlaveSlot_1,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .mergeTimeout = CY_SMIF_MERGE_TIMEOUT_1_CYCLE,
#endif
};

cy_stc_smif_mem_config_t* smifMemConfigs[CY_SMIF_DEVICE_NUM] = {
    &S25FL256S_SlaveSlot_0,
    &S25FL256S_SlaveSlot_1,
};

cy_stc_smif_config_t SMIF0_config =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = SMIF0_DESELECT_DELAY,
    .rxClockSel = CY_SMIF_SEL_INVERTED_FEEDBACK_CLK,
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
    .delayTapEnable = CY_SMIF_DELAY_TAP_DISABLE,
    .delayLineSelect = CY_SMIF_NO_DELAY_SEL,
};
#endif

cy_stc_smif_block_config_t smif0BlockConfig =
{
    .memCount = CY_SMIF_DEVICE_NUM,
    .memConfig = (cy_stc_smif_mem_config_t**)smifMemConfigs,
    .majorVersion = CY_SMIF_DRV_VERSION_MAJOR,
    .minorVersion = CY_SMIF_DRV_VERSION_MINOR,
};
