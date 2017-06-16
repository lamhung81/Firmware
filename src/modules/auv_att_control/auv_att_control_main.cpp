/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file auv_att_control_main.cpp
 * AUV attitude controller.
 *
 *
 *
 * @author Lam-Hung NGUYEN		<lamhung81@gmail.com>
 *
 *
 */

// include:..........

//#include <conversion/rotation.h>
//#include <drivers/drv_hrt.h>
//#include <lib/geo/geo.h>
//#include <lib/mathlib/mathlib.h>
//#include <lib/tailsitter_recovery/tailsitter_recovery.h>
//#include <px4_config.h>
//#include <px4_defines.h>
//#include <px4_posix.h>
//#include <px4_tasks.h>
//#include <systemlib/circuit_breaker.h>
//#include <systemlib/err.h>
//#include <systemlib/param/param.h>
//#include <systemlib/perf_counter.h>
//#include <systemlib/systemlib.h>
//#include <uORB/topics/actuator_armed.h>
//#include <uORB/topics/actuator_controls.h>
//#include <uORB/topics/battery_status.h>
//#include <uORB/topics/control_state.h>
//#include <uORB/topics/manual_control_setpoint.h>
//#include <uORB/topics/mc_att_ctrl_status.h>
//#include <uORB/topics/multirotor_motor_limits.h>
//#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/sensor_correction.h>
//#include <uORB/topics/sensor_gyro.h>
//#include <uORB/topics/vehicle_attitude_setpoint.h>
//#include <uORB/topics/vehicle_control_mode.h>
//#include <uORB/topics/vehicle_rates_setpoint.h>
//#include <uORB/topics/vehicle_status.h>
//#include <uORB/uORB.h>
//
///**
// * AUV attitude control app start / stop handling function
// *
// * @ingroup apps
// */
//extern "C" __EXPORT int auv_att_control_main(int argc, char *argv[]);
//
//
////define: ................
//
//#define YAW_DEADZONE	0.05f
//#define MIN_TAKEOFF_THRUST    0.2f
//#define TPA_RATE_LOWER_LIMIT 0.05f
//#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
//#define ATTITUDE_TC_DEFAULT 0.2f
//
//#define AXIS_INDEX_ROLL 0
//#define AXIS_INDEX_PITCH 1
//#define AXIS_INDEX_YAW 2
//#define AXIS_COUNT 3
//
//#define MAX_GYRO_COUNT 3

/////////////////////////////////////////////////////////////////////////////
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
//#include <poll.h>
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

#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/uORB.h>

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

/////////////////////////////////////////////////////////////////////////////


extern "C" __EXPORT int auv_att_control_main(int argc, char *argv[]);


class AUVAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	AUVAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~AUVAttitudeControl();

	/**
	 * Start the AUV attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:


};


namespace auv_att_control
{

AUVAttitudeControl	*g_control;
}



AUVAttitudeControl::AUVAttitudeControl()
{

	}
AUVAttitudeControl::~AUVAttitudeControl()
{
	//debug lhnguyen: can phai viet destructor, for deleting memory
	auv_att_control::g_control = nullptr;

	}








//int
//AUVAttitudeControl::parameters_update()
//{

//}


/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
//void
//AUVAttitudeControl::control_attitude(float dt)
//{
//}

//void
//AUVAttitudeControl::task_main()
//{
//Flying with USB is not safe
//}
// nsh: mc_att_control: command not found
// default PWM output device

//error finding param: FW_ARSP_MODE
//nsh: mc_att_control: command not found
//


int
AUVAttitudeControl::start()
{

	//subcribe to set_attitude_target topic
	int vehicle_rates_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	// limit the update rate to 5 Hz
	px4_pollfd_struct_t fds = {};
	fds.events = POLLIN; 
	fds.fd = vehicle_rates_setpoint_sub_fd;


#if 0


orb_set_interval(vehicle_rates_setpoint_sub_fd, 200);
#endif


	const char *dev= PWM_OUTPUT0_DEVICE_PATH;

	/* open for ioctl only */
	int fd = px4_open(dev, 0);
	if (fd < 0) {
			PX4_ERR("can't open %s", dev);
			return 1;
		}

	int ret;
	int pwm_value;
	pwm_value = 1400;

	while (1) {
	
		/* wait for sensor update of 1 file descriptor for 10 ms (0.01 second) */
		int poll_ret = px4_poll(&fds, 1, 10);
		(void)poll_ret;

		// timed out - periodic check for _task_should_exit 
		// if (poll_ret == 0) {
		// 	continue;
		// }

                if (fds.revents & POLLIN) {
                /* obtained data for the first file descriptor */
 			struct vehicle_rates_setpoint_s raw;
			memset(&raw, 0, sizeof(raw));
			//copy sensors raw data into local buffer
			orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub_fd, &raw);
			PX4_INFO("Debug AUV:\t% 1.6f\t %1.6f\t %1.6f\t% 1.6f",
								 (double)raw.roll,
								 (double)raw.pitch,
								 (double)raw.yaw,
								 (double)raw.thrust);
   		 }

			//for (unsigned i = 0; i < 8; i++) {
		       { int i; i = 0;
		       {
			//		PX4_INFO("PWM_VALUE  %5d", pwm_value);
		    		ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d)", i);
						return 1;
					}
				}
		       pwm_value +=1;
		       if (pwm_value >= 1600) {
		    	   pwm_value = 1400;
		       }
			}

				/* Delay longer than the max Oneshot duration */

			usleep(2542*10); //micro second

#ifdef __PX4_NUTTX
			/* Trigger all timer's channels in Oneshot mode to fire
			 * the oneshots with updated values.
			 */

			up_pwm_update();
#endif
		}
}

int auv_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: auv_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (auv_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		auv_att_control::g_control = new AUVAttitudeControl;

		if (auv_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != auv_att_control::g_control->start()) {
			delete auv_att_control::g_control;
			auv_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (auv_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete auv_att_control::g_control;
		auv_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (auv_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}









