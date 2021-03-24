/*
 * terminal.c
 *
 *  Created on: 6 Sep 2020
 *      Author: arion
 */

#include "debug/terminal.h"

#include "debug/profiler.h"
#include "debug/monitor.h"
#include "debug/board_io.h"
#include "can_transmission.h"
#include "storage/flash_logging.h"
#include "flash.h"
#include "telemetry/xbee.h"

#include <stm32f4xx_hal.h>
#include <stdlib.h> // Even though not recommended


#define EQUALS(index, str) (cmd->num_components > (index) && component_matches(&cmd->components[(index)], (str)))

bool verbose;

uint8_t __parse_hex_char(char c) {
	if(c >= '0' && c <= '9') {
		return c - '0';
	} else if(c >= 'A' && c <= 'F') {
		return 10 + (c - 'A');
	} else if(c >= 'a' && c <= 'f') {
		return 10 + (c - 'a');
	} else {
		return 0xFF; // Error
	}
}

void terminal_execute(ShellCommand* cmd, void (*respond)(const char* format, ...)) {
	if(cmd->num_components > 0) {
		if(has_io_mode(IO_INPUT & IO_DIRECT)) {
			if(EQUALS(0, "exit")) {
				disable_io_mode(IO_INPUT & IO_DIRECT); // Disables direct input mode
				respond("> Board IO mode set to 0x%08x\n", get_io_mode());
			} else if(has_io_mode(IO_INPUT & IO_DIRECT & IO_CAN)) {
				CAN_msg message;

				if(cmd->num_components < 2) {
					respond("> Format: id < float > data <timestamp>\n");
					return;
				}

				message.id_CAN = get_board_id();
				message.id = atoi(cmd->components[0].component);
				message.timestamp = HAL_GetTick();

				if(EQUALS(1, "float") && cmd->num_components >= 3) {
					message.data = (uint32_t) atof(cmd->components[2].component);

					if(cmd->num_components == 4) {
						message.timestamp = atoi(cmd->components[3].component);
					}
				} else if(EQUALS(1, "hex")) {
					message.data = 0; // TODO

					if(cmd->num_components == 4) {
						message.timestamp = atoi(cmd->components[3].component);
					}
				} else {
					message.data = atoi(cmd->components[1].component);

					if(cmd->num_components == 3) {
						message.timestamp = atoi(cmd->components[2].component);
					}
				}

				can_setFrame(message.data, message.id, message.timestamp);
			} else if(has_io_mode(IO_INPUT & IO_DIRECT & IO_TELEMETRY)) {
				const char* packet_buffer = cmd->components[0].component;

				for(uint8_t i = 0; i < cmd->components[0].length; i += 2) {
					uint8_t byte = __parse_hex_char(packet_buffer[i]) << 4 | __parse_hex_char(packet_buffer[i + 1]);
					processReceivedByte(byte);
				}
			}

			return;
		}

		if(EQUALS(0, "help")) {
			respond("> BellaLui Terminal available commands:\r\n");
			respond("> bridge: creates a I/O bridge with the given board\r\n");
			respond("> flash: performs basic bulk operations on the flash memory\r\n");
			respond("> help: shows this help page\r\n");
			respond("> id: prints the shell's board ID\r\n");
			respond("> input: redirects the shell's input to the specified input stream\r\n");
			respond("> monitor: enables or disables a specific monitor\r\n");
			respond("> output: redirects the specified output stream to the shell\r\n");
			respond("> profiler: enables or disables the embedded profiler\r\n");
			respond("> reset: performs a software reset of the avionics\r\n");
			respond("> shutdown: shutdowns the avionics\r\n");
			respond("> version: displays the current terminal version\r\n");
		} else if(EQUALS(0, "shutdown")) {
			respond("> BellaLui shutting down immediately\r\n");
			while(1);
		} else if(EQUALS(0, "version")) {
			respond("> BellaLui Terminal v1.0 by Arion Zimmermann\r\n");
		} else if(EQUALS(0, "reset")) {
			HAL_NVIC_SystemReset();
		} else if(EQUALS(0, "time")) {
			respond("%d\n", HAL_GetTick());
		} else if(EQUALS(0, "clear")) {
			respond("\x1b[2J\x1b[H\e7");
		} else if(EQUALS(0, "id")) {
			int32_t bridge = get_shell_bridge();
			uint32_t board_id = get_board_id();

			if(bridge < 0) {
				respond("> Direct shell to board %u\r\n", board_id);
			} else {
				respond("> Bridged shell to board %d\r\n", bridge);
			}
		} else if(EQUALS(0, "profiler")) {
			if(EQUALS(1, "enable")) {
				enable_profiler();
				respond("\x1b[2J");
				respond("> Profiler now enabled\r\n");
			} else if(EQUALS(1, "disable")) {
				disable_profiler();
				respond("\x1b[2J");
				respond("> Profiler now disabled\r\n");
			} else {
				respond("> Usage: profiler { enable | disable }\r\n");
			}
		} else if(EQUALS(0, "verbose")) {
			if(EQUALS(1, "on")) {
				verbose = true;
				respond("> Verbose mode enabled\r\n");
			} else if(EQUALS(1, "off")) {
				verbose = false;
				respond("> Verbose mode disabled\r\n");
			} else {
				respond("> Usage: verbose { on | off }\r\n");
			}
		} else if(EQUALS(0, "monitor")) {
			if(EQUALS(1, "enable") && cmd->num_components >= 3) {
				uint8_t location = 0;
				uint8_t refresh_rate = 1;

				if(cmd->num_components > 3) {
					location = atoi(cmd->components[3].component);
				}

				if(cmd->num_components > 4) {
					refresh_rate = atoi(cmd->components[4].component);
				}

				respond("\x1b[2J");

				if(EQUALS(2, "sensor")) {
					enable_monitor(SENSOR_MONITOR, location, refresh_rate);
					respond("> Sensor monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "state")) {
					enable_monitor(STATE_MONITOR, location, refresh_rate);
					respond("> State monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "kalman")) {
					enable_monitor(KALMAN_MONITOR, location, refresh_rate);
					respond("> Kalman monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "flash")) {
					enable_monitor(FLASH_MONITOR, location, refresh_rate);
					respond("> Flash monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "can")) {
					enable_monitor(CAN_MONITOR, location, refresh_rate);
					respond("> CAN bus monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "telemetry")) {
					enable_monitor(TELEMETRY_MONITOR, location, refresh_rate);
					respond("> Telemetry monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "gps")) {
					enable_monitor(GPS_MONITOR, location, refresh_rate);
					respond("> GPS monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "airbrakes")) {
					enable_monitor(AIRBRAKES_MONITOR, location, refresh_rate);
					respond("> Airbrakes monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else if(EQUALS(2, "propulsion")) {
					enable_monitor(PROPULSION_MONITOR, location, refresh_rate);
					respond("> Propulsion monitor enabled with %dHz frequency\r\n", refresh_rate);
				} else {
					respond("> Usage: monitor enable { sensor | state | kalman | flash | can | telemetry | airbrakes } location [refresh rate; default: 1Hz]\r\n");
				}
			} else if(EQUALS(1, "disable") && cmd->num_components == 3) {
				respond("\x1b[2J");

				if(EQUALS(2, "sensor")) {
					disable_monitor(SENSOR_MONITOR);
					respond("> Sensor monitor disabled\n");
				} else if(EQUALS(2, "state")) {
					disable_monitor(STATE_MONITOR);
					respond("> State monitor disabled\n");
				} else if(EQUALS(2, "kalman")) {
					disable_monitor(KALMAN_MONITOR);
					respond("> Kalman monitor disabled\n");
				} else if(EQUALS(2, "flash")) {
					disable_monitor(FLASH_MONITOR);
					respond("> Flash monitor disabled\n");
				} else if(EQUALS(2, "can")) {
					disable_monitor(CAN_MONITOR);
					respond("> CAN bus monitor disabled\n");
				} else if(EQUALS(2, "telemetry")) {
					disable_monitor(TELEMETRY_MONITOR);
					respond("> Telemetry monitor disabled\n");
				} else if(EQUALS(2, "gps")) {
					disable_monitor(GPS_MONITOR);
					respond("> GPS monitor disabled\n");
				} else if(EQUALS(2, "airbrakes")) {
					disable_monitor(AIRBRAKES_MONITOR);
					respond("> Airbrakes monitor disabled\n");
				} else if(EQUALS(2, "propulsion")) {
					disable_monitor(PROPULSION_MONITOR);
					respond("> Propulsion monitor disabled\n");
				} else {
					respond("> Usage: monitor disable { sensor | state | kalman | flash | can | telemetry | airbrakes }\n");
				}
			} else {
				respond("> Usage: monitor { enable | disable } { sensor | state | kalman | flash | can | telemetry | airbrakes } [location] [refresh rate; default: 10]\n");
			}
		} else if(EQUALS(0, "bridge")) {
			if(EQUALS(1, "create")) {
				if(cmd->num_components > 2) {
					uint8_t target = atoi(cmd->components[2].component);
					can_setFrame(SHELL_BRIDGE_CREATE | target, DATA_ID_SHELL_CONTROL, HAL_GetTick());
					respond("> Creating bridge with board %u...\n", target);
				} else {
					respond("> Usage: bridge create target\n");
				}
			} else if(EQUALS(1, "destroy")) {
				uint8_t target = get_board_id();
				can_setFrame(SHELL_BRIDGE_DESTROY | target, DATA_ID_SHELL_CONTROL, HAL_GetTick());
				respond("> Bridge with board %u destroyed\n", target);
			} else {
				respond("> Usage: bridge { create | destroy } [target]\n");
			}
		} else if(EQUALS(0, "output")) {

			uint32_t mask = 0;

			if(EQUALS(1, "can")) {
				mask = IO_OUTPUT & IO_CAN;
			} else if(EQUALS(1, "telemetry")) {
				mask = IO_OUTPUT & IO_TELEMETRY;
			} else {
				respond("> IO target not recognised.\n");
				respond("> Usage: output { can | telemetry } { auto, pipe, direct }\n");
				return;
			}

			disable_io_mode(mask); // Disables direct input mode

			uint32_t mode = 0;

			if(EQUALS(2, "auto") || EQUALS(3, "auto") || EQUALS(4, "auto")) {
				mode |= IO_AUTO;
			}

			if(EQUALS(2, "pipe") || EQUALS(3, "pipe") || EQUALS(4, "pipe")) {
				mode |= IO_PIPE;

				respond("> If pipe mode enabled, reset the board to return to the standard terminal.\n");

				if(EQUALS(1, "telemetry")) {
					UART_HandleTypeDef* huart = get_console_uart();
					xbee_change_uart(huart);
					respond("> Warning: Telemetry DUPLEX I/O pipe enabled.\n");
				}
			}

			if(EQUALS(2, "direct") || EQUALS(3, "direct") || EQUALS(4, "direct")) {
				mode |= IO_DIRECT;
			}

			enable_io_mode(mask & mode);

			respond("> Board IO mode set to 0x%08x\n", get_io_mode());
		} else if(EQUALS(0, "input")) {

			uint32_t mask = 0;

			if(EQUALS(1, "can")) {
				mask = IO_INPUT & IO_CAN;
			} else if(EQUALS(1, "telemetry")) {
				mask = IO_INPUT & IO_TELEMETRY;
			} else {
				respond("> IO target not recognised.\n");
				respond("> Usage: input { can | telemetry } { auto, pipe, direct }\n");
				return;
			}

			disable_io_mode(mask);

			uint32_t mode = 0;

			if(EQUALS(2, "auto") || EQUALS(3, "auto") || EQUALS(4, "auto")) {
				mode |= IO_AUTO;
			}

			if(EQUALS(2, "pipe") || EQUALS(3, "pipe") || EQUALS(4, "pipe")) {
				mode |= IO_PIPE;

				respond("> If pipe mode enabled, reset the board to return to the standard terminal.\n");

				if(EQUALS(1, "telemetry")) {
					UART_HandleTypeDef* huart = get_console_uart();
					xbee_change_uart(huart);
					respond("> Warning: Telemetry DUPLEX I/O pipe enabled.\n");
				}
			}

			if(EQUALS(2, "direct") || EQUALS(3, "direct") || EQUALS(4, "direct")) {
				mode |= IO_DIRECT;
				respond("> Direct input mode entered. Type 'exit' to return to the standard terminal.\n");
			}

			enable_io_mode(mask & mode);

			respond("> Board IO mode set to 0x%08x\n", get_io_mode());
		} else if(EQUALS(0, "flash")) {
			if(EQUALS(1, "dump")) {
				on_fullsd_dump_request();
				respond("> Flash dump requested\n");
			} else if(EQUALS(1, "erase")) {
				respond("> Erasing flash memory... ");
				flash_erase_all();
				respond("done\nPlease reset the board to format the filesystem.\n");
			} else if(EQUALS(1, "download")) {
				for(uint8_t i = 2; i < cmd->num_components; i++) {
					uint16_t block = atoi(cmd->components[i].component);

					if(block < 4096) {
						respond("> Upload of block %d requested\n", block);
						on_upload_request(block);
					}
				}
			} else {
				respond("> Usage: flash { dump | erase | download } [blocks...]\n");
			}
		} else {
			respond("> %.*s: command not found\n", cmd->components[0].length, cmd->components[0].component);
		}
	}
}

bool is_verbose() {
	return verbose;
}

#undef EQUALS
