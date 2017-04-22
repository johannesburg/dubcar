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

#ifndef BLDC_INTERFACE_H_
#define BLDC_INTERFACE_H_

#include "datatypes.h"

#define PACKET_MAX_PL_LEN 512
#ifdef __cplusplus
extern "C" {
#endif

unsigned char* send_packet(unsigned char *data, unsigned int len);
void print_buffer(unsigned char *data, unsigned int len);

// Setters
// unsigned char * bldc_interface_terminal_cmd(char* cmd);
unsigned char * bldc_interface_set_duty_cycle(float dutyCycle);
unsigned char * bldc_interface_set_current(float current);
unsigned char * bldc_interface_set_current_brake(float current);
unsigned char * bldc_interface_set_rpm(int rpm);
unsigned char * bldc_interface_set_pos(float pos);
unsigned char * bldc_interface_set_servo_pos(float pos);
// unsigned char * bldc_interface_set_mcconf(const mc_configuration *mcconf);
// unsigned char * bldc_interface_set_appconf(const app_configuration *appconf);

// Getters
// unsigned char * bldc_interface_get_fw_version(void);
// unsigned char * bldc_interface_get_values(void);
// unsigned char * bldc_interface_get_mcconf(void);
// unsigned char * bldc_interface_get_appconf(void);
// unsigned char * bldc_interface_get_decoded_ppm(void);
// unsigned char * bldc_interface_get_decoded_adc(void);
// unsigned char * bldc_interface_get_decoded_chuk(void);

// Other functions
// unsigned char * bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty);
unsigned char * bldc_interface_reboot(void);
unsigned char * bldc_interface_send_alive(void);

#ifdef __cplusplus
}
#endif
#endif /* BLDC_INTERFACE_H_ */
