#ifndef _W5500_
#define _W5500_

/*
 * W5500 common registers
 */
#define W5500_COMMON_REGS	0x0000
#define W5500_MR		0x0000 /* Mode Register */
#define MR_RST			0x80 /* S/W reset */
#define MR_PB			0x10 /* Ping block */
#define MR_AI			0x02 /* Address Auto-Increment */
#define MR_IND			0x01 /* Indirect mode */
#define W5500_SHAR		0x0009 /* Source MAC address */
#define W5500_IR		0x0015 /* Interrupt Register */
#define W5500_COMMON_REGS_LEN	0x0040

#define W5500_Sn_MR		0x0000 /* Sn Mode Register */
#define W5500_Sn_CR		0x0001 /* Sn Command Register */
#define W5500_Sn_IR		0x0002 /* Sn Interrupt Register */
#define W5500_Sn_SR		0x0003 /* Sn Status Register */
#define W5500_Sn_TX_FSR		0x0020 /* Sn Transmit free memory size */
#define W5500_Sn_TX_RD		0x0022 /* Sn Transmit memory read pointer */
#define W5500_Sn_TX_WR		0x0024 /* Sn Transmit memory write pointer */
#define W5500_Sn_RX_RSR		0x0026 /* Sn Receive free memory size */
#define W5500_Sn_RX_RD		0x0028 /* Sn Receive memory read pointer */

#define Sn_REGS(priv, idx)		((priv)->s0_regs + (idx * 4))

#define W5500_Sn_MR(priv, idx)		(Sn_REGS(priv, idx) + W5500_Sn_MR)
#define Sn_MR_TCP			  0x01 /* TCP mode */
#define Sn_MR_UDP			  0x02 /* UDP mode */
#define Sn_MR_MACRAW			  0x04 /* MAC RAW mode */
#define Sn_MR_MF			  0x40 /* MAC Filter for W5500 */
#define W5500_Sn_MR_MF			  0x80 /* MAC Filter for W5500 */
#define W5500_Sn_CR(priv, idx)		(Sn_REGS(priv, idx) + W5500_Sn_CR)
#define Sn_CR_OPEN			  0x01 /* OPEN command */
#define Sn_CR_CLOSE			  0x10 /* CLOSE command */
#define Sn_CR_SEND			  0x20 /* SEND command */
#define Sn_CR_RECV			  0x40 /* RECV command */
#define W5500_Sn_IR(priv, idx)		(Sn_REGS(priv, idx) + W5500_Sn_IR)
#define Sn_IR_SENDOK			  0x10 /* complete sending */
#define Sn_IR_RECV			  0x04 /* receiving data */
#define W5500_Sn_SR(priv, idx)		(Sn_REGS(priv, idx) + W5500_Sn_SR)
#define Sn_SR_MACRAW			  0x42 /* mac raw mode */
#define W5500_Sn_TX_FSR(priv, idx)	(Sn_REGS(priv, idx) + W5500_Sn_TX_FSR)
#define W5500_Sn_TX_RD(priv, idx)	(Sn_REGS(priv, idx) + W5500_Sn_TX_RD)
#define W5500_Sn_TX_WR(priv, idx)	(Sn_REGS(priv, idx) + W5500_Sn_TX_WR)
#define W5500_Sn_RX_RSR(priv, idx)	(Sn_REGS(priv, idx) + W5500_Sn_RX_RSR)
#define W5500_Sn_RX_RD(priv, idx)	(Sn_REGS(priv, idx) + W5500_Sn_RX_RD)

#define W5500_Sn_REGS_LEN	0x0040

/*
 * W5500 specific register and memory
 *
 * W5500 register and memory are organized by multiple blocks.  Each one is
 * selected by 16bits offset address and 5bits block select bits.  So we
 * encode it into 32bits address. (lower 16bits is offset address and
 * upper 16bits is block select bits)
 */
#define W5500_SIMR		0x0018 /* Socket Interrupt Mask Register */
#define W5500_RTR		0x0019 /* Retry Time-value Register */

#define W5500_S0_REGS		0x10000

#define W5500_Sn_RXMEM_SIZE(n)	\
		(0x1001e + (n) * 0x40000) /* Sn RX Memory Size */
#define W5500_Sn_TXMEM_SIZE(n)	\
		(0x1001f + (n) * 0x40000) /* Sn TX Memory Size */

#define W5500_Sn_TX_MEM_START(idx)	(0x20000 + (idx * 4))
#define W5500_TX_MEM_SIZE		0x00800
#define W5500_Sn_RX_MEM_START(idx)	(0x30000 + (idx * 4))
#define W5500_RX_MEM_SIZE		0x00800

#define W5500_OFFLOAD_MAX_SOCKETS	8

struct w5500_config {
	const char *gpio_port;
	uint8_t gpio_pin;
	gpio_dt_flags_t gpio_flags;
	const char *spi_port;
	gpio_pin_t spi_cs_pin;
	gpio_dt_flags_t spi_cs_dt_flags;
	const char *spi_cs_port;
	uint32_t spi_freq;
	uint8_t spi_slave;
	uint8_t full_duplex;
	int32_t timeout;
};

struct w5500_off_socket {
	struct net_context *context;
	/* Socket 0 register offset address */
	uint32_t s0_regs;
	/* Socket 0 TX buffer offset address and size */
	uint32_t s0_tx_buf;
	uint16_t s0_tx_buf_size;
	/* Socket 0 RX buffer offset address and size */
	uint32_t s0_rx_buf;
	uint16_t s0_rx_buf_size;
};

struct w5500_runtime {
	struct net_if *iface;
	K_THREAD_STACK_MEMBER(thread_stack,
			      CONFIG_ETH_W5500_RX_THREAD_STACK_SIZE);
	struct k_thread thread;
	uint8_t mac_address[6];
	struct device *gpio;
	struct device *spi;
	struct spi_cs_control spi_cs;
	struct spi_config spi_cfg;
	struct gpio_callback gpio_cb;
	struct k_sem tx_rx_sem;
	struct k_sem int_sem;
	struct w5500_off_socket socket[W5500_OFFLOAD_MAX_SOCKETS]
};

#endif /*_W5500_*/
