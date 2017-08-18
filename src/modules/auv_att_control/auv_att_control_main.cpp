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
	//int roll_pwm_value , pitch_pwm_value, yaw_pwm_value, thrust_pwm_value;



	float 		joystick_deadband(float value, float threshold);



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
	delete auv_att_control::g_control;
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

/* Function for create a deadband for joytick
theshold is a positive number
*/	
float AUVAttitudeControl::joystick_deadband(float joystick_value, float joystick_threshold)
{
	if ((float)fabs((double)joystick_value) < joystick_threshold){
		joystick_value = 0.0;
	} else if (joystick_value > joystick_threshold){
                joystick_value = (joystick_value - joystick_threshold) / ((float)1.0 - joystick_threshold);
        } else
        {
                joystick_value = -(-joystick_value - joystick_threshold) / ((float)1.0 - joystick_threshold);
        }

	return joystick_value;
}

int
AUVAttitudeControl::start()
{

	//subcribe to set_attitude_target topic
	int vehicle_rates_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	// limit the update rate to 5 Hz
	px4_pollfd_struct_t fds = {};
	fds.events = POLLIN; 
	fds.fd = vehicle_rates_setpoint_sub_fd;


	//bool updated;
	//orb_check(_v_rates_sp_sub, &updated);

	//if (updated) {
	//	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	//}


        #if 0  //Debug
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
	int roll_pwm_value , pitch_pwm_value, yaw_pwm_value, thrust_pwm_value ;
	roll_pwm_value = pitch_pwm_value = yaw_pwm_value = thrust_pwm_value = 1500;

	while (1) {
	
		/* wait for sensor update of 1 file descriptor for 10 ms (0.01 second) */
		int poll_ret = px4_poll(&fds, 1, 10);
		
                (void) poll_ret;  

		// timed out - periodic check for _task_should_exit 
                //lhnguyen debug: use the if following code block leads to the reaction of motors 
                //only when there are new messages from vehicle_rates_setpoint
		/*
                if (poll_ret == 0) {
			pwm_value = 1500; //Disarm pwm of BlueESC
                        px4_ioctl(fd, PWM_SERVO_SET(0), pwm_value);
                        px4_ioctl(fd, PWM_SERVO_SET(1), pwm_value);
                        px4_ioctl(fd, PWM_SERVO_SET(2), pwm_value);
                        px4_ioctl(fd, PWM_SERVO_SET(3), pwm_value);
                        continue;
		}
                */

                if (fds.revents & POLLIN) {
                /* obtained data for the first file descriptor */
 			struct vehicle_rates_setpoint_s raw;
			memset(&raw, 0, sizeof(raw));
			//copy sensors raw data into local buffer
			orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub_fd, &raw);
			

			//Apply joystick deadband, joystick_deadband = 0.1
   		 	raw.roll  = joystick_deadband(raw.roll,0.1);
   			raw.pitch = joystick_deadband(raw.pitch,0.1);
   			raw.yaw   = joystick_deadband(raw.yaw,0.1);
   			raw.thrust= joystick_deadband(raw.thrust,0.1);

   			/* debug lhnguyen
                        PX4_INFO("Debug AUV:\t% 1.6f\t %1.6f\t %1.6f\t% 1.6f",
								 (double)raw.roll,
								 (double)raw.pitch,
								 (double)raw.yaw,
								 (double)raw.thrust);
                        */
   			
   			//Convert joystick signals to pwm values, 
   			//Neutral value =1500, according to T200 Bluerobotics motor characteristic
   			//Take 1500 +/- 50 for giving small pwm value range
   			roll_pwm_value   = 1500 + (int)((float)50.0*raw.roll);
   			pitch_pwm_value  = 1500 + (int)((float)50.0*raw.pitch);
   			yaw_pwm_value    = 1500 + (int)((float)50.0*raw.yaw);
   			thrust_pwm_value = 1500 + (int)((float)50.0*raw.thrust); 

                        /* debug lhnguyen pwm output to motors
                        PX4_INFO("Debug AUV:\t% 6d\t %6d\t %6d\t% 6d",
                                                                 roll_pwm_value,
                                                                 pitch_pwm_value,
                                                                 yaw_pwm_value,
                                                                 thrust_pwm_value);
                        */



   		 }

                for (unsigned i = 0; i < 4; i++) {                                     
                        switch (i) {
                                case 0:
                                        pwm_value = roll_pwm_value;
                                        break;
                                case 1:
                                        pwm_value = pitch_pwm_value;
                                        break;
                                case 2:
                                        pwm_value = yaw_pwm_value;
                                        break;  
                                case 3:
                                        pwm_value = thrust_pwm_value;
                                        break;  
                                default:
                                        pwm_value = 1500;
                                        break;
                                }
                        
                                // PX4_INFO("PWM_VALUE  %5d", pwm_value);
                                //ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);
                                ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);       

                                if (ret != OK) {
                                        PX4_ERR("PWM_SERVO_SET(%d)", i);
                                        return 1;
                                }
                        
                   
                        }

                        /* Delay longer than the max Oneshot duration */
                        //usleep(2542*10); //micro second

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









