#ifndef ZEPHYR_INCLUDE_ARCH_TRICORE_MPU_H_
#define ZEPHYR_INCLUDE_ARCH_TRICORE_MPU_H_

#include <stdint.h>
#include <zephyr/sys/dlist.h>

#define TRICORE_MPU_ACCESS_P_R (1 << 0)
#define TRICORE_MPU_ACCESS_U_R (1 << 1)
#define TRICORE_MPU_ACCESS_P_W (1 << 2)
#define TRICORE_MPU_ACCESS_U_W (1 << 3)
#define TRICORE_MPU_ACCESS_P_X (1 << 4)
#define TRICORE_MPU_ACCESS_U_X (1 << 5)

#define TRICORE_MPU_ACCESS_P_NA_U_NA (0)
#define TRICORE_MPU_ACCESS_P_RO_U_NA (TRICORE_MPU_ACCESS_P_R)
#define TRICORE_MPU_ACCESS_P_RO_U_RO (TRICORE_MPU_ACCESS_P_R | TRICORE_MPU_ACCESS_U_R)
#define TRICORE_MPU_ACCESS_P_RW_U_NA (TRICORE_MPU_ACCESS_P_R | TRICORE_MPU_ACCESS_P_W)
#define TRICORE_MPU_ACCESS_P_RW_U_RO                                                               \
	(TRICORE_MPU_ACCESS_P_R | TRICORE_MPU_ACCESS_P_W | TRICORE_MPU_ACCESS_U_R)
#define TRICORE_MPU_ACCESS_P_RW_U_RW                                                               \
	(TRICORE_MPU_ACCESS_P_R | TRICORE_MPU_ACCESS_P_W | TRICORE_MPU_ACCESS_U_R |                \
	 TRICORE_MPU_ACCESS_U_W)
#define TRICORE_MPU_ACCESS_P_RX_U_RX                                                               \
	(TRICORE_MPU_ACCESS_P_R | TRICORE_MPU_ACCESS_P_X | TRICORE_MPU_ACCESS_U_R |                \
	 TRICORE_MPU_ACCESS_U_X)
#define TRICORE_MPU_ACCESS_P_RWX_U_RWX                                                             \
	(TRICORE_MPU_ACCESS_P_R | TRICORE_MPU_ACCESS_P_W | TRICORE_MPU_ACCESS_P_X |                \
	 TRICORE_MPU_ACCESS_U_R | TRICORE_MPU_ACCESS_U_W | TRICORE_MPU_ACCESS_U_X)

#define K_MEM_PARTITION_IS_EXECUTABLE(access_rights)                                               \
	(access_rights & (TRICORE_MPU_ACCESS_P_X | TRICORE_MPU_ACCESS_U_X))

#define K_MEM_PARTITION_IS_WRITABLE(access_rights)                                                 \
	(access_rights & (TRICORE_MPU_ACCESS_P_W | TRICORE_MPU_ACCESS_U_W))

/* Read-Write access permission attributes */
#define K_MEM_PARTITION_P_RW_U_RW ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_RW_U_RW})
#define K_MEM_PARTITION_P_RW_U_NA ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_RW_U_NA})
#define K_MEM_PARTITION_P_RO_U_RO ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_RO_U_RO})
#define K_MEM_PARTITION_P_RO_U_NA ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_RO_U_NA})
#define K_MEM_PARTITION_P_NA_U_NA ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_NA_U_NA})

/* Execution-allowed attributes */
#define K_MEM_PARTITION_P_RX_U_RX   ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_RX_U_RX})
#define K_MEM_PARTITION_P_RWX_U_RWX ((k_mem_partition_attr_t){TRICORE_MPU_ACCESS_P_RWX_U_RWX})

struct tricore_mem_partition_attr {
    uint32_t access_rights: 6;
    uint32_t dpr : 5;
    uint32_t cpr : 5;
};

typedef struct tricore_mem_partition_attr k_mem_partition_attr_t;

struct arch_mem_domain {
	sys_dnode_t loaded_node;
	uint32_t cpxe;
	uint32_t dpre;
	uint32_t dpwe;
};

#define Z_TRICORE_STACK_GUARD_SIZE  Z_POW2_CEIL(MAX(ARCH_STACK_PTR_ALIGN, CONFIG_MPU_STACK_GUARD_MIN_SIZE))

#endif
