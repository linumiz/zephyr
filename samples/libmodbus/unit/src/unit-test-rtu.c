/*
 * Copyright © 2008-2010 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <zephyr.h>
#include <sys/printk.h>

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>
#include <net/net_if.h>

#include "unit-test.h"

enum {
    TCP,
    TCP_PI,
    RTU
};

int main(int argc, char *argv[])
{
    uint8_t *tab_rp_bits;
    uint16_t *tab_rp_registers;
    uint16_t *tab_rp_registers_bad;
    modbus_t *ctx = NULL;
    int i;
    uint8_t value;
    int nb_points;
    int rc;
    float real;
    uint32_t ireal;
    struct zsock_timeval old_response_timeout;
    struct zsock_timeval response_timeout;
    int use_backend;

	net_dhcpv4_start(net_if_get_default());
	k_msleep(5000);
    if (argc > 1) {
        if (strcmp(argv[1], "tcp") == 0) {
            use_backend = TCP;
	} else if (strcmp(argv[1], "tcppi") == 0) {
            use_backend = TCP_PI;
        } else if (strcmp(argv[1], "rtu") == 0) {
            use_backend = RTU;
        } else {
            printk("Usage:\n  %s [tcp|tcppi|rtu] - Modbus client for unit testing\n\n", argv[0]);
            exit(1);
        }
    } else {
        /* By default */
        use_backend = RTU;
    }

	k_msleep(10 * 1000);
    if (use_backend == TCP) {
        ctx = modbus_new_tcp("192.168.8.138", 1502);
    } else if (use_backend == TCP_PI) {
        ctx = modbus_new_tcp_pi("::1", "1502");
    } else {
        ctx = modbus_new_rtu("UART_4", 115200, 'N', 8, 1);
    }
    if (ctx == NULL) {
        printk("Unable to allocate libmodbus context\n");
        return -1;
    }
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx,
                              MODBUS_ERROR_RECOVERY_LINK |
                              MODBUS_ERROR_RECOVERY_PROTOCOL);

    if (use_backend == RTU) {
          modbus_set_slave(ctx, SERVER_ID);
    }

    if (modbus_connect(ctx) == -1) {
        printk("Connection failed: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    /* Allocate and initialize the memory to store the bits */
    nb_points = (UT_BITS_NB > UT_INPUT_BITS_NB) ? UT_BITS_NB : UT_INPUT_BITS_NB;
    tab_rp_bits = (uint8_t *) malloc(nb_points * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb_points * sizeof(uint8_t));

    /* Allocate and initialize the memory to store the registers */
    nb_points = (UT_REGISTERS_NB > UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    tab_rp_registers = (uint16_t *) malloc(nb_points * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    printk("** UNIT TESTING **\n");

    printk("\nTEST WRITE/READ:\n");

    /** COIL BITS **/

    /* Single */
    rc = modbus_write_bit(ctx, UT_BITS_ADDRESS, ON);
    printk("1/2 modbus_write_bit: ");
    if (rc == 1) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
    printk("2/2 modbus_read_bits: ");
    if (rc != 1) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }

    if (tab_rp_bits[0] != ON) {
        printk("FAILED (%0X = != %0X)\n", tab_rp_bits[0], ON);
        goto close;
    }
    printk("OK\n");
    /* End single */

    /* Multiple bits */
    {
        uint8_t tab_value[UT_BITS_NB];

        modbus_set_bits_from_bytes(tab_value, 0, UT_BITS_NB, UT_BITS_TAB);
        rc = modbus_write_bits(ctx, UT_BITS_ADDRESS,
                               UT_BITS_NB, tab_value);
        printk("1/2 modbus_write_bits: ");
        if (rc == UT_BITS_NB) {
            printk("OK\n");
        } else {
            printk("FAILED\n");
            goto close;
        }
    }

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_rp_bits);
    printk("2/2 modbus_read_bits: ");
    if (rc != UT_BITS_NB) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }

    i = 0;
    nb_points = UT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        if (value != UT_BITS_TAB[i]) {
            printk("FAILED (%0X != %0X)\n", value, UT_BITS_TAB[i]);
            goto close;
        }

        nb_points -= nb_bits;
        i++;
    }
    printk("OK\n");
    /* End of multiple bits */

    /** DISCRETE INPUTS **/
    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB, tab_rp_bits);
    printk("1/1 modbus_read_input_bits: ");

    if (rc != UT_INPUT_BITS_NB) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }

    i = 0;
    nb_points = UT_INPUT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        if (value != UT_INPUT_BITS_TAB[i]) {
            printk("FAILED (%0X != %0X)\n", value, UT_INPUT_BITS_TAB[i]);
            goto close;
        }

        nb_points -= nb_bits;
        i++;
    }
    printk("OK\n");

    /** HOLDING REGISTERS **/

    /* Single register */
    rc = modbus_write_register(ctx, UT_REGISTERS_ADDRESS, 0x1234);
    printk("1/2 modbus_write_register: ");
    if (rc == 1) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               1, tab_rp_registers);
    printk("2/2 modbus_read_registers: ");
    if (rc != 1) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }

    if (tab_rp_registers[0] != 0x1234) {
        printk("FAILED (%0X != %0X)\n",
               tab_rp_registers[0], 0x1234);
        goto close;
    }
    printk("OK\n");
    /* End of single register */

    /* Many registers */
    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                UT_REGISTERS_NB, UT_REGISTERS_TAB);
    printk("1/5 modbus_write_registers: ");
    if (rc == UT_REGISTERS_NB) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printk("2/5 modbus_read_registers: ");
    if (rc != UT_REGISTERS_NB) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }

    for (i=0; i < UT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != UT_REGISTERS_TAB[i]) {
            printk("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i],
                   UT_REGISTERS_TAB[i]);
            goto close;
        }
    }
    printk("OK\n");

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               0, tab_rp_registers);
    printk("3/5 modbus_read_registers (0): ");
    if (rc != -1 && errno == EMBMDATA) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }
    printk("OK\n");

    nb_points = (UT_REGISTERS_NB >
                 UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    /* Write registers to zero from tab_rp_registers and store read registers
       into tab_rp_registers. So the read registers must set to 0, except the
       first one because there is an offset of 1 register on write. */
    rc = modbus_write_and_read_registers(ctx,
                                         UT_REGISTERS_ADDRESS + 1, UT_REGISTERS_NB - 1,
                                         tab_rp_registers,
                                         UT_REGISTERS_ADDRESS,
                                         UT_REGISTERS_NB,
                                         tab_rp_registers);
    printk("4/5 modbus_write_and_read_registers: ");
    if (rc != UT_REGISTERS_NB) {
        printk("FAILED (nb points %d != %d)\n", rc, UT_REGISTERS_NB);
        goto close;
    }

    if (tab_rp_registers[0] != UT_REGISTERS_TAB[0]) {
        printk("FAILED (%0X != %0X)\n",
               tab_rp_registers[0], UT_REGISTERS_TAB[0]);
    }

    for (i=1; i < UT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != 0) {
            printk("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i], 0);
            goto close;
        }
    }
    printk("OK\n");

    /* End of many registers */


    /** INPUT REGISTERS **/
    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     UT_INPUT_REGISTERS_NB,
                                     tab_rp_registers);
    printk("1/1 modbus_read_input_registers: ");
    if (rc != UT_INPUT_REGISTERS_NB) {
        printk("FAILED (nb points %d)\n", rc);
        goto close;
    }

    for (i=0; i < UT_INPUT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != UT_INPUT_REGISTERS_TAB[i]) {
            printk("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i], UT_INPUT_REGISTERS_TAB[i]);
            goto close;
        }
    }
    printk("OK\n");

    printk("\nTEST FLOATS\n");
    /** FLOAT **/
    printk("1/2 Set float: ");
    modbus_set_float(UT_REAL, tab_rp_registers);
    if (tab_rp_registers[1] == (UT_IREAL >> 16) &&
        tab_rp_registers[0] == (UT_IREAL & 0xFFFF)) {
        printk("OK\n");
    } else {
        /* Avoid *((uint32_t *)tab_rp_registers)
         * https://github.com/stephane/libmodbus/pull/104 */
        ireal = (uint32_t) tab_rp_registers[0] & 0xFFFF;
        ireal |= (uint32_t) tab_rp_registers[1] << 16;
        printk("FAILED (%x != %x)\n", ireal, UT_IREAL);
        goto close;
    }

    printk("2/2 Get float: ");
    real = modbus_get_float(tab_rp_registers);
    if (real == UT_REAL) {
        printk("OK\n");
    } else {
        printk("FAILED (%f != %f)\n", real, UT_REAL);
        goto close;
    }

    printk("\nAt this point, error messages doesn't mean the test has failed\n");

    /** ILLEGAL DATA ADDRESS **/
    printk("\nTEST ILLEGAL DATA ADDRESS:\n");

    /* The mapping begins at 0 and ends at address + nb_points so
     * the addresses are not valid. */

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS,
                          UT_BITS_NB + 1, tab_rp_bits);
    printk("* modbus_read_bits: ");
    if (rc == -1 && errno == EMBXILADD) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB + 1, tab_rp_bits);
    printk("* modbus_read_input_bits: ");
    if (rc == -1 && errno == EMBXILADD)
        printk("OK\n");
    else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB + 1, tab_rp_registers);
    printk("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBXILADD)
        printk("OK\n");
    else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     UT_INPUT_REGISTERS_NB + 1,
                                     tab_rp_registers);
    printk("* modbus_read_input_registers: ");
    if (rc == -1 && errno == EMBXILADD)
        printk("OK\n");
    else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_write_bit(ctx, UT_BITS_ADDRESS + UT_BITS_NB, ON);
    printk("* modbus_write_bit: ");
    if (rc == -1 && errno == EMBXILADD) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS + UT_BITS_NB,
                           UT_BITS_NB, tab_rp_bits);
    printk("* modbus_write_coils: ");
    if (rc == -1 && errno == EMBXILADD) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS + UT_REGISTERS_NB,
                                UT_REGISTERS_NB, tab_rp_registers);
    printk("* modbus_write_registers: ");
    if (rc == -1 && errno == EMBXILADD) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }


    /** TOO MANY DATA **/
    printk("\nTEST TOO MANY DATA ERROR:\n");

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS,
                          MODBUS_MAX_READ_BITS + 1, tab_rp_bits);
    printk("* modbus_read_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                MODBUS_MAX_READ_BITS + 1, tab_rp_bits);
    printk("* modbus_read_input_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               MODBUS_MAX_READ_REGISTERS + 1,
                               tab_rp_registers);
    printk("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     MODBUS_MAX_READ_REGISTERS + 1,
                                     tab_rp_registers);
    printk("* modbus_read_input_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS,
                           MODBUS_MAX_WRITE_BITS + 1, tab_rp_bits);
    printk("* modbus_write_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printk("OK\n");
    } else {
        goto close;
        printk("FAILED\n");
    }

    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                MODBUS_MAX_WRITE_REGISTERS + 1,
                                tab_rp_registers);
    printk("* modbus_write_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    /** SLAVE REPLY **/
    printk("\nTEST SLAVE REPLY:\n");
    modbus_set_slave(ctx, INVALID_SERVER_ID);
    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    if (use_backend == RTU) {
        const int RAW_REQ_LENGTH = 6;
        uint8_t raw_req[] = { INVALID_SERVER_ID, 0x03, 0x00, 0x01, 0xFF, 0xFF };
        uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

        /* No response in RTU mode */
        printk("1/4-A No response from slave %d: ", INVALID_SERVER_ID);

        if (rc == -1 && errno == ETIMEDOUT) {
            printk("OK\n");
        } else {
            printk("FAILED\n");
            goto close;
        }

        /* Send an invalid query with a wrong slave ID */
        modbus_send_raw_request(ctx, raw_req,
                                RAW_REQ_LENGTH * sizeof(uint8_t));
        rc = modbus_receive_confirmation(ctx, rsp);

        printk("1/4-B No response from slave %d with invalid request: ",
               INVALID_SERVER_ID);

        if (rc == -1 && errno == ETIMEDOUT) {
            printk("OK\n");
        } else {
            printk("FAILED (%d)\n", rc);
            goto close;
        }

    } else {
        /* Response in TCP mode */
        printk("1/4 Response from slave %d: ", 18);

        if (rc == UT_REGISTERS_NB) {
            printk("OK\n");
        } else {
            printk("FAILED\n");
            goto close;
        }
    }

    rc = modbus_set_slave(ctx, MODBUS_BROADCAST_ADDRESS);
    if (rc == -1) {
        printk("Invalid broacast address\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printk("2/4 Reply after a broadcast query: ");
    if (rc == UT_REGISTERS_NB) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    /* Restore slave */
    if (use_backend == RTU) {
        modbus_set_slave(ctx, SERVER_ID);
    } else {
        modbus_set_slave(ctx, MODBUS_TCP_SLAVE);
    }

    printk("3/4 Report slave ID: \n");
    /* tab_rp_bits is used to store bytes */
    rc = modbus_report_slave_id(ctx, tab_rp_bits);
    if (rc == -1) {
        printk("FAILED\n");
        goto close;
    }

    /* Slave ID is an arbitraty number for libmodbus */
    if (rc > 0) {
        printk("OK Slave ID is %d\n", tab_rp_bits[0]);
    } else {
        printk("FAILED\n");
        goto close;
    }

    /* Run status indicator */
    if (rc > 1 && tab_rp_bits[1] == 0xFF) {
        printk("OK Run Status Indicator is %s\n", tab_rp_bits[1] ? "ON" : "OFF");
    } else {
        printk("FAILED\n");
        goto close;
    }

    /* Print additional data as string */
    if (rc > 2) {
        printk("Additional data: ");
        for (i=2; i < rc; i++) {
            printk("%c", tab_rp_bits[i]);
        }
        printk("\n");
    }

    /* Save original timeout */
    modbus_get_response_timeout(ctx, &old_response_timeout);

    /* Define a new and too short timeout */
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 0;
    modbus_set_response_timeout(ctx, &response_timeout);

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printk("4/4 Too short timeout: ");
    if (rc == -1 && errno == ETIMEDOUT) {
        printk("OK\n");
    } else {
        printk("FAILED (can fail on slow systems or Windows)\n");
    }

    /* Restore original timeout */
    modbus_set_response_timeout(ctx, &old_response_timeout);

    /* A wait and flush operation is done by the error recovery code of
     * libmodbus */

    /** BAD RESPONSE **/
    printk("\nTEST BAD RESPONSE ERROR:\n");

    /* Allocate only the required space */
    tab_rp_registers_bad = (uint16_t *) malloc(
        UT_REGISTERS_NB_SPECIAL * sizeof(uint16_t));
    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB_SPECIAL, tab_rp_registers_bad);
    printk("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBBADDATA) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    free(tab_rp_registers_bad);

    /** MANUAL EXCEPTION **/
    printk("\nTEST MANUAL EXCEPTION:\n");

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS_SPECIAL,
                               UT_REGISTERS_NB, tab_rp_registers);
    printk("* modbus_read_registers at special address: ");
    if (rc == -1 && errno == EMBXSBUSY) {
        printk("OK\n");
    } else {
        printk("FAILED\n");
        goto close;
    }

    /** RAW REQUEST */
    printk("\nTEST RAW REQUEST:\n");
    {
        const int RAW_REQ_LENGTH = 6;
        uint8_t raw_req[] = { (use_backend == RTU) ? SERVER_ID : 0xFF,
                              0x03, 0x00, 0x01, 0x0, 0x05 };
        int req_length;
        uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

        req_length = modbus_send_raw_request(ctx, raw_req,
                                             RAW_REQ_LENGTH * sizeof(uint8_t));

        printk("* modbus_send_raw_request: ");
        if ((use_backend == RTU && req_length == (RAW_REQ_LENGTH + 2)) ||
            ((use_backend == TCP || use_backend == TCP_PI) &&
             req_length == (RAW_REQ_LENGTH + 6))) {
            printk("OK\n");
        } else {
            printk("FAILED (%d)\n", req_length);
            goto close;
        }

        printk("* modbus_receive_confirmation: ");
        rc  = modbus_receive_confirmation(ctx, rsp);
        if ((use_backend == RTU && rc == 15) ||
            ((use_backend == TCP || use_backend == TCP_PI) &&
             rc == 19)) {
            printk("OK\n");
        } else {
            printk("FAILED (%d)\n", rc);
            goto close;
        }
    }

    printk("\nALL TESTS PASS WITH SUCCESS.\n");

close:
    /* Free the memory */
    free(tab_rp_bits);
    free(tab_rp_registers);

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
