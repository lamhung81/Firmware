/****************************************************************************
 *
 *   Copyright (c) 2013, 2014, 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwmhung.c
 * a modified version of pwm, for debugging the direct output of pwm to the GPIO
 *
 *
 * PWM servo output configuration and monitoring tool.
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

static void	usage(const char *reason);
__EXPORT int	pwmhung_main(int argc, char *argv[]);


static void
usage(const char *reason)
{
	if (reason != NULL) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO(
		"usage:\n"
		"pwmhung arm|disarm|rate|failsafe|disarmed|min|max|test|steps|info  ...\n"  //lhnguyen: pwmhung instead of pwm
		"\n"
		"arm\t\t\t\tArm output\n"
		"disarm\t\t\t\tDisarm output\n"
		"\n"
		"oneshot ...\t\t\tConfigure Oneshot\n"
		"\t[-g <channel group>]\t(e.g. 0,1,2)\n"
		"\t[-m <channel mask> ]\t(e.g. 0xF)\n"
		"\t[-a]\t\t\tConfigure all outputs\n"
		"\n"
		"rate ...\t\t\tConfigure PWM rates\n"
		"\t[-g <channel group>]\t(e.g. 0,1,2)\n"
		"\t[-m <channel mask> ]\t(e.g. 0xF)\n"
		"\t[-a]\t\t\tConfigure all outputs\n"
		"\t-r <alt_rate>\t\tPWM rate (0 - oneshot, 50 to 400 Hz)\n"
		"\n"
		"failsafe ...\t\t\tFailsafe PWM\n"
		"disarmed ...\t\t\tDisarmed PWM\n"
		"min ...\t\t\t\tMinimum PWM\n"
		"max ...\t\t\t\tMaximum PWM\n"
//	     "trim ...\t\t\tTrim PWM\n"
		"\t[-e]\t\t\trobust error handling\n"
		"\t[-c <channels>]\t\t(e.g. 1234)\n"
		"\t[-m <channel mask> ]\t(e.g. 0xF)\n"
		"\t[-a]\t\t\tConfigure all outputs\n"
		"\t-p <pwm value>\t\tPWM value\n"
		"\n"
		"test ...\t\t\tDirectly set PWM\n"
		"\t[-c <channels>]\t\t(e.g. 1234)\n"
		"\t[-m <channel mask> ]\t(e.g. 0xF)\n"
		"\t[-a]\t\t\tConfigure all outputs\n"
		"\t-p <pwm value>\t\tPWM value\n"
		"\n"
		"steps ...\t\t\tRun 5 steps\n"
		"\t[-c <channels>]\t\t(e.g. 1234)\n"
		"\n"
		"info\t\t\t\tPrint information\n"
		"\n"
		"\t-v\t\t\tVerbose\n"
		"\t-d <dev>\t\t(default " PWM_OUTPUT0_DEVICE_PATH ")\n"
	);

}

static unsigned
get_parameter_value(const char *option, const char *paramDescription)
{
	unsigned result_value = 0;

	/* check if this is a param name */
	if (strncmp("p:", option, 2) == 0) {

		char paramName[32];
		strncpy(paramName, option + 2, 17);
		/* user wants to use a param name */
		param_t parm = param_find(paramName);

		if (parm != PARAM_INVALID) {
			int32_t pwm_parm;
			int gret = param_get(parm, &pwm_parm);

			if (gret == 0) {
				result_value = pwm_parm;

			} else {
				PX4_ERR("PARAM '%s' LOAD FAIL", paramDescription);
				return gret;
			}

		} else {
			PX4_ERR("PARAM '%s' NAME NOT FOUND", paramName);
			return 1;
		}

	} else {
		char *ep;
		result_value = strtoul(option, &ep, 0);

		if (*ep != '\0') {
			PX4_ERR("BAD '%s'", paramDescription);
			return 1;
		}
	}

	return result_value;
}

int
pwmhung_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int alt_rate = -1; // Default to indicate not set.
	uint32_t alt_channel_groups = 0;
	bool alt_channels_set = false;
	bool print_verbose = false;
	bool error_on_warn = false;
	bool oneshot = false;
	int ch;
	int ret;
	char *ep;
	uint32_t set_mask = 0;
	unsigned group;
	unsigned long channels;
	unsigned single_ch = 0;
	int pwm_value = 0;

	if (argc < 2) {
		usage(NULL);
		return 1;
	}

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "d:vec:g:m:ap:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			if (NULL == strstr(myoptarg, "/dev/")) {
				PX4_WARN("device %s not valid", myoptarg);
				usage(NULL);
				return 1;
			}

			dev = myoptarg;
			break;

		case 'v':
			print_verbose = true;
			break;

		case 'e':
			error_on_warn = false;
			break;

		case 'c':
			/* Read in channels supplied as one int and convert to mask: 1234 -> 0xF */
			channels = strtoul(myoptarg, &ep, 0);

			while ((single_ch = channels % 10)) {

				set_mask |= 1 << (single_ch - 1);
				channels /= 10;
			}

			break;

		case 'g':
			group = strtoul(myoptarg, &ep, 0);

			if ((*ep != '\0') || (group >= 32)) {
				usage("bad channel_group value");
				return 1;
			}

			alt_channel_groups |= (1 << group);
			alt_channels_set = true;
			PX4_INFO("alt channels set, group: %d", group);
			break;

		case 'm':
			/* Read in mask directly */
			set_mask = strtoul(myoptarg, &ep, 0);

			if (*ep != '\0') {
				usage("BAD set_mask VAL");
				return 1;
			}

			break;

		case 'a':
			for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
				set_mask |= 1 << i;
			}

			break;

		case 'p':
			pwm_value = get_parameter_value(myoptarg, "PWM Value");
			break;

		case 'r':
			alt_rate = get_parameter_value(myoptarg, "PWM Rate");
			break;

		default:
			usage(NULL);
			return 1;
		}
	}

	if (myoptind >= argc) {
		usage(NULL);
		return 1;
	}

	const char *command = argv[myoptind];

	if (print_verbose && set_mask > 0) {
		PX4_INFO("Channels: ");
		printf("    ");

		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
			if (set_mask & 1 << i) {
				printf("%d ", i + 1);
			}
		}

		printf("\n");
	}

	/* open for ioctl only */
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return 1;
	}

	/* get the number of servo channels */
	unsigned servo_count;
	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return error_on_warn;
	}

	oneshot = !strcmp(command, "oneshot");

	if (!strcmp(command, "arm")) {
		return 0;
	} else if (!strcmp(command, "disarm")) {
		return 0;
	} else if (oneshot || !strcmp(command, "rate")) {
		return 0;
	} else if (!strcmp(command, "min")) {
		return 0;
	} else if (!strcmp(command, "max")) {
		return 0;
	} else if (!strcmp(command, "disarmed")) {
		return 0;
	} else if (!strcmp(command, "failsafe")) {
		return 0;
	} else if (!strcmp(command, "test")) {

		if (set_mask == 0) {
			usage("no channels set");
			return 1;
		}

		if (pwm_value == 0) {
			usage("no PWM provided");
			return 1;
		}

		/* get current servo values */
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {


			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}

		/* perform PWM output */

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		PX4_INFO("Press CTRL-C or 'c' to abort.");

		while (1) {
			for (unsigned i = 0; i < servo_count; i++) {
				if (set_mask & 1 << i) {
					ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d)", i);
						return 1;
					}
				}
			}

			/* abort on user request */
			char c;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				ret = read(0, &c, 1);

				if (c == 0x03 || c == 0x63 || c == 'q') {
					/* reset output to the last value */
					for (unsigned i = 0; i < servo_count; i++) {
						if (set_mask & 1 << i) {
							ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

							if (ret != OK) {
								PX4_ERR("PWM_SERVO_SET(%d)", i);
								return 1;
							}
						}
					}

					PX4_INFO("User abort\n");
					return 0;
				}
			}

			/* Delay longer than the max Oneshot duration */

			usleep(2542);

#ifdef __PX4_NUTTX
			/* Trigger all timer's channels in Oneshot mode to fire
			 * the oneshots with updated values.
			 */

			up_pwm_update();
#endif
		}

		return 0;


	} else if (!strcmp(command, "steps")) {
		return 0;
	} else if (!strcmp(command, "info")) {

		printf("device: %s\n", dev);

		uint32_t info_default_rate;
		uint32_t info_alt_rate;
		uint32_t info_alt_rate_mask;

		ret = px4_ioctl(fd, PWM_SERVO_GET_DEFAULT_UPDATE_RATE, (unsigned long)&info_default_rate);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_DEFAULT_UPDATE_RATE");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_UPDATE_RATE, (unsigned long)&info_alt_rate);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_UPDATE_RATE");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_SELECT_UPDATE_RATE, (unsigned long)&info_alt_rate_mask);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_SELECT_UPDATE_RATE");
			return 1;
		}

		struct pwm_output_values failsafe_pwm;

		struct pwm_output_values disarmed_pwm;

		struct pwm_output_values min_pwm;

		struct pwm_output_values max_pwm;

		struct pwm_output_values trim_pwm;

		ret = px4_ioctl(fd, PWM_SERVO_GET_FAILSAFE_PWM, (unsigned long)&failsafe_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_FAILSAFE_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (unsigned long)&disarmed_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_DISARMED_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (unsigned long)&min_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_MIN_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (unsigned long)&max_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_MAX_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_TRIM_PWM, (unsigned long)&trim_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_TRIM_PWM");
			return 1;
		}

		/* print current servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t spos;

			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);

			if (ret == OK) {
				printf("channel %u: %u us", i + 1, spos);

				if (info_alt_rate_mask & (1 << i)) {
					printf(" (alternative rate: %d Hz", info_alt_rate);

				} else {
					printf(" (default rate: %d Hz", info_default_rate);
				}


				printf(" failsafe: %d, disarmed: %d us, min: %d us, max: %d us, trim: %5.2f)",
				       failsafe_pwm.values[i], disarmed_pwm.values[i], min_pwm.values[i], max_pwm.values[i],
				       (double)((int16_t)(trim_pwm.values[i]) / 10000.0f));
				printf("\n");

			} else {
				printf("%u: ERROR\n", i);
			}
		}

		/* print rate groups */
		for (unsigned i = 0; i < servo_count; i++) {
			uint32_t group_mask;

			ret = px4_ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);

			if (ret != OK) {
				break;
			}

			if (group_mask != 0) {
				printf("channel group %u: channels", i);

				for (unsigned j = 0; j < 32; j++) {
					if (group_mask & (1 << j)) {
						printf(" %u", j + 1);
					}
				}

				printf("\n");
			}
		}

		return 0;

	} else if (!strcmp(command, "forcefail")) {
		return 0;

	} else if (!strcmp(command, "terminatefail")) {
		return 0;
	}

	usage(NULL);
	return 0;
}
