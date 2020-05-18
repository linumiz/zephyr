#define IVT_MAX_IRQ	255
#define IVT_ALIGN	8192
#define IVT_ENTRY_SIZE	32

uint8_t __ivt_table[MAX_IVT_IRQ * IVT_ENTRY_SIZE] __attribute__((aligned(IVT_ALIGN)));

#define DEFINE_INT _tricore_ivt_
DEFINE_INT(1);
