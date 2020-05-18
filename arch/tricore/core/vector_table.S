#include <toolchain.h>
#include <linker/sections.h>
#include "vector_table.h"

_ASM_FILE_PROLOGUE

GDATA(z_main_stack)

SECTION_FUNC(_vector_table_section,_vector_table)

    _tricore_trap_ 0 z_tricore_reset
    _tricore_trap_ 1 z_tricore_internal_protection
    _tricore_trap_ 2 z_tricore_instruction_error
    _tricore_trap_ 3 z_tricore_context_mgmt
    _tricore_trap_ 4 z_tricore_bus_error
    _tricore_trap_ 5 z_tricore_assertion
    _tricore_trap_ 6 z_tricore_syscall
    _tricore_trap_ 7 z_tricore_nmi
