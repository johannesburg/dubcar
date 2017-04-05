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

/*
 * bldc_interface.c
 *
 * Compatible Firmware Versions
 * 2.16
 *
 */

#include "bldc_interface.h"
#include "buffer.h"
#include <stdio.h>
#include <string.h>
#include "crc.h"
#include <stdlib.h>

void print_buffer(unsigned char *data, unsigned int len) {
  for (int i = 0; i < len - 1; i++) {
    printf("%02x:", (int) data[i]);
  } 
  printf("%02x\n", (int) data[len - 1]);;
} 

// Private variables
static unsigned char send_buffer[PACKET_MAX_PL_LEN];


// wrap payload with proper serial headers/footers
unsigned char* send_packet(unsigned char *data, unsigned int len) {
	if (len > PACKET_MAX_PL_LEN) {
		return NULL;
	}

	int b_ind = 0;

	unsigned char *buf = (unsigned char *) malloc(PACKET_MAX_PL_LEN * sizeof(unsigned char));
	
	if (buf == NULL)
		return NULL;

	if (len <= 256) {
		buf[b_ind++] = 2;
		buf[b_ind++] = len;
	} else {
		buf[b_ind++] = 3;
		buf[b_ind++] = len >> 8;
		buf[b_ind++] = len & 0xFF;
	}

	memcpy(buf + b_ind, data, len);
	b_ind += len;

	unsigned short crc = crc16(data, len);
	buf[b_ind++] = (uint8_t)(crc >> 8);
	buf[b_ind++] = (uint8_t)(crc & 0xFF);
	buf[b_ind++] = 3;

	//print_buffer(buf, b_ind);
	memset(send_buffer, 0, PACKET_MAX_PL_LEN); 
	return buf;
}


// // Setters
// unsigned char * bldc_interface_terminal_cmd(char* cmd) {
// 	int32_t send_index = 0;
// 	int len = strlen(cmd);
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_TERMINAL_CMD;
// 	memcpy(send_buffer + send_index, cmd, len);
// 	send_index += len;
// 	return send_packet(send_buffer, send_index);
// }

unsigned char * bldc_interface_set_duty_cycle(float dutyCycle) {
  int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer, dutyCycle, 100000.0, &send_index);
	return send_packet(send_buffer, send_index);
}

unsigned char * bldc_interface_set_current(float current) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	return send_packet(send_buffer, send_index);
}

unsigned char * bldc_interface_set_current_brake(float current) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	return send_packet(send_buffer, send_index);
}

unsigned char * bldc_interface_set_rpm(int rpm) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer, rpm, &send_index);
	return send_packet(send_buffer, send_index);
}

unsigned char * bldc_interface_set_pos(float pos) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer, pos, 1000000.0, &send_index);
	return send_packet(send_buffer, send_index);
}

unsigned char * bldc_interface_set_servo_pos(float pos) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer, pos, 1000.0, &send_index);
	return send_packet(send_buffer, send_index);
}

// unsigned char * bldc_interface_set_mcconf(const mc_configuration *mcconf) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_SET_MCCONF;

// 	send_buffer[send_index++] = mcconf->pwm_mode;
// 	send_buffer[send_index++] = mcconf->comm_mode;
// 	send_buffer[send_index++] = mcconf->motor_type;
// 	send_buffer[send_index++] = mcconf->sensor_mode;

// 	buffer_append_float32(send_buffer, mcconf->l_current_max, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_current_min, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_in_current_max, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_in_current_min, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_abs_current_max, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_min_erpm, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_max_erpm, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_max_erpm_fbrake, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_max_erpm_fbrake_cc, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_min_vin, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_max_vin, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_battery_cut_start, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_battery_cut_end, 1000, &send_index);
// 	send_buffer[send_index++] = mcconf->l_slow_abs_current;
// 	send_buffer[send_index++] = mcconf->l_rpm_lim_neg_torque;
// 	buffer_append_float32(send_buffer,mcconf->l_temp_fet_start, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_temp_fet_end, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_temp_motor_start, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_temp_motor_end, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_min_duty, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->l_max_duty, 1000000, &send_index);

// 	buffer_append_float32(send_buffer,mcconf->sl_min_erpm, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->sl_min_erpm_cycle_int_limit, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->sl_max_fullbreak_current_dir_change, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->sl_cycle_int_limit, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->sl_phase_advance_at_br, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->sl_cycle_int_rpm_br, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->sl_bemf_coupling_k, 1000, &send_index);

// 	memcpy(send_buffer + send_index, mcconf->hall_table, 8);
// 	send_index += 8;
// 	buffer_append_float32(send_buffer,mcconf->hall_sl_erpm, 1000, &send_index);

// 	buffer_append_float32(send_buffer, mcconf->foc_current_kp, 1e5, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_current_ki, 1e5, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_f_sw, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_dt_us, 1e6, &send_index);
// 	send_buffer[send_index++] = mcconf->foc_encoder_inverted;
// 	buffer_append_float32(send_buffer, mcconf->foc_encoder_offset, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_encoder_ratio, 1e3, &send_index);
// 	send_buffer[send_index++] = mcconf->foc_sensor_mode;
// 	buffer_append_float32(send_buffer, mcconf->foc_pll_kp, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_pll_ki, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_motor_l, 1e8, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_motor_r, 1e5, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_motor_flux_linkage, 1e5, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_observer_gain, 1e0, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_duty_dowmramp_kp, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_duty_dowmramp_ki, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_openloop_rpm, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_sl_openloop_hyst, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_sl_openloop_time, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_sl_d_current_duty, 1e3, &send_index);
// 	buffer_append_float32(send_buffer, mcconf->foc_sl_d_current_factor, 1e3, &send_index);
// 	memcpy(send_buffer + send_index, mcconf->foc_hall_table, 8);
// 	send_index += 8;
// 	buffer_append_float32(send_buffer,mcconf->foc_hall_sl_erpm, 1000, &send_index);

// 	buffer_append_float32(send_buffer,mcconf->s_pid_kp, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->s_pid_ki, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->s_pid_kd, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->s_pid_min_erpm, 1000, &send_index);

// 	buffer_append_float32(send_buffer,mcconf->p_pid_kp, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->p_pid_ki, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->p_pid_kd, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->p_pid_ang_div, 1e5, &send_index);

// 	buffer_append_float32(send_buffer,mcconf->cc_startup_boost_duty, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->cc_min_current, 1000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->cc_gain, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->cc_ramp_step_max, 1000000, &send_index);

// 	buffer_append_int32(send_buffer, mcconf->m_fault_stop_time_ms, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->m_duty_ramp_step, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->m_duty_ramp_step_rpm_lim, 1000000, &send_index);
// 	buffer_append_float32(send_buffer,mcconf->m_current_backoff_gain, 1000000, &send_index);
// 	buffer_append_uint32(send_buffer, mcconf->m_encoder_counts, &send_index);

// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_set_appconf(const app_configuration *appconf) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_SET_APPCONF;
// 	send_buffer[send_index++] = appconf->controller_id;
// 	buffer_append_uint32(send_buffer, appconf->timeout_msec, &send_index);
// 	buffer_append_float32(send_buffer, appconf->timeout_brake_current, 1000.0, &send_index);
// 	send_buffer[send_index++] = appconf->send_can_status;
// 	buffer_append_uint16(send_buffer, appconf->send_can_status_rate_hz, &send_index);

// 	send_buffer[send_index++] = appconf->app_to_use;

// 	send_buffer[send_index++] = appconf->app_ppm_conf.ctrl_type;
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.pid_max_erpm, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.hyst, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.pulse_start, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.pulse_end, 1000.0, &send_index);
// 	send_buffer[send_index++] = appconf->app_ppm_conf.median_filter;
// 	send_buffer[send_index++] = appconf->app_ppm_conf.safe_start;
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.rpm_lim_start, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.rpm_lim_end, 1000.0, &send_index);
// 	send_buffer[send_index++] = appconf->app_ppm_conf.multi_esc;
// 	send_buffer[send_index++] = appconf->app_ppm_conf.tc;
// 	buffer_append_float32(send_buffer, appconf->app_ppm_conf.tc_max_diff, 1000.0, &send_index);

// 	send_buffer[send_index++] = appconf->app_adc_conf.ctrl_type;
// 	buffer_append_float32(send_buffer, appconf->app_adc_conf.hyst, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_adc_conf.voltage_start, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_adc_conf.voltage_end, 1000.0, &send_index);
// 	send_buffer[send_index++] = appconf->app_adc_conf.use_filter;
// 	send_buffer[send_index++] = appconf->app_adc_conf.safe_start;
// 	send_buffer[send_index++] = appconf->app_adc_conf.cc_button_inverted;
// 	send_buffer[send_index++] = appconf->app_adc_conf.rev_button_inverted;
// 	send_buffer[send_index++] = appconf->app_adc_conf.voltage_inverted;
// 	buffer_append_float32(send_buffer, appconf->app_adc_conf.rpm_lim_start, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_adc_conf.rpm_lim_end, 1000.0, &send_index);
// 	send_buffer[send_index++] = appconf->app_adc_conf.multi_esc;
// 	send_buffer[send_index++] = appconf->app_adc_conf.tc;
// 	buffer_append_float32(send_buffer, appconf->app_adc_conf.tc_max_diff, 1000.0, &send_index);
// 	buffer_append_uint16(send_buffer, appconf->app_adc_conf.update_rate_hz, &send_index);

// 	buffer_append_uint32(send_buffer, appconf->app_uart_baudrate, &send_index);

// 	send_buffer[send_index++] = appconf->app_chuk_conf.ctrl_type;
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.hyst, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.rpm_lim_start, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.rpm_lim_end, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.ramp_time_pos, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.ramp_time_neg, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.stick_erpm_per_s_in_cc, 1000.0, &send_index);
// 	send_buffer[send_index++] = appconf->app_chuk_conf.multi_esc;
// 	send_buffer[send_index++] = appconf->app_chuk_conf.tc;
// 	buffer_append_float32(send_buffer, appconf->app_chuk_conf.tc_max_diff, 1000.0, &send_index);

// 	send_buffer[send_index++] = appconf->app_nrf_conf.speed;
// 	send_buffer[send_index++] = appconf->app_nrf_conf.power;
// 	send_buffer[send_index++] = appconf->app_nrf_conf.crc_type;
// 	send_buffer[send_index++] = appconf->app_nrf_conf.retry_delay;
// 	send_buffer[send_index++] = appconf->app_nrf_conf.retries;
// 	send_buffer[send_index++] = appconf->app_nrf_conf.channel;
// 	memcpy(send_buffer + send_index, appconf->app_nrf_conf.address, 3);
// 	send_index += 3;
// 	send_buffer[send_index++] = appconf->app_nrf_conf.send_crc_ack;

// 	return send_packet(send_buffer, send_index);
// }

// // Getters
// unsigned char * bldc_interface_get_fw_version(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_FW_VERSION;
// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_get_values(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_GET_VALUES;
// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_get_mcconf(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_GET_MCCONF;
// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_get_appconf(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_GET_APPCONF;
// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_get_decoded_ppm(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_GET_DECODED_PPM;
// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_get_decoded_adc(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_GET_DECODED_ADC;
// 	return send_packet(send_buffer, send_index);
// }

// unsigned char * bldc_interface_get_decoded_chuk(void) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_GET_DECODED_CHUK;
// 	return send_packet(send_buffer, send_index);
// }

// // Other functions
// unsigned char * bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty) {
// 	int32_t send_index = 0;
// 	//fwd_can_append(send_buffer, &send_index);
// 	send_buffer[send_index++] = COMM_DETECT_MOTOR_PARAM;
// 	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, min_rpm, 1000.0, &send_index);
// 	buffer_append_float32(send_buffer, low_duty, 1000.0, &send_index);
// 	return send_packet(send_buffer, send_index);
// }

unsigned char * bldc_interface_reboot(void) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_REBOOT;
	return send_packet(send_buffer, send_index);
}

unsigned char * bldc_interface_send_alive(void) {
	int32_t send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_ALIVE;
	return send_packet(send_buffer, send_index);
}

