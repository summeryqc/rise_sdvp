/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "comm_cc1120.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "commands.h"
#include "cc1120.h"
#include "utils.h"
#include "packet.h"

// Threads
static THD_WORKING_AREA(rx_thread_wa, 2048);
static THD_WORKING_AREA(tx_thread_wa, 512);
static THD_FUNCTION(rx_thread, arg);
static THD_FUNCTION(tx_thread, arg);

void comm_cc1120_init(void) {
	cc1120_init();

	chThdCreateStatic(rx_thread_wa, sizeof(rx_thread_wa),
			NORMALPRIO, rx_thread, NULL);
	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa),
			NORMALPRIO, tx_thread, NULL);
}

static THD_FUNCTION(rx_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC1120 RX");

	for(;;) {
//		cc1120_state();
//		if (cc1120_state() == 17) {
//			uint8_t rxbytes;
//			cc1120_burst_read(CC1120_NUM_RXBYTES, &rxbytes, 1);
//			commands_printf("rxbytes %d", rxbytes);
//
//			cc1120_off();
//			cc1120_on();
////			cc1120_flushrx();

//		commands_printf("State: %s", cc1120_state_name());

//		char b[10];
//		utils_byte_to_binary(cc1120_strobe(CC1120_SNOP), b);
//		commands_printf("Status: %s", b);
//		commands_printf("RX: %d\n", cc1120_read_rxbytes());

//		uint8_t d[50];
//		for (int i = 0;i < 50;i++) {
//			d[i] = i % 2;
//		}
//		int res = cc1120_transmit(d, 50);
//		commands_printf("Res: %d", res);

		chThdSleepMilliseconds(1000);
	}
}

static THD_FUNCTION(tx_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC1120 TX");

	for(;;) {
		chThdSleepMilliseconds(1);
	}
}