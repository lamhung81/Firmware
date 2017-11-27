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
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <uORB/topics/pressure.h>
#include <uORB/topics/optical_flow.h>  //lhnguyen debug: low pass filter for depth and depth velocity estimation


#include <math.h>

#include <mathlib/mathlib.h>

/////////////////////////////////////////////////////////////////////////////


extern "C" __EXPORT int auv_att_control_main(int argc, char *argv[]);

using math::Vector;
//using math::Vector3f;
using matrix::Vector3f;
using math::Matrix;
using math::Quaternion;


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
  	bool  	_task_should_exit;    /**< if true, task_main() should exit */
  	int   	_control_task;      /**< task handle */


  	int   	_v_rates_sp_sub;    /**< vehicle rates setpoint subscription */
  	int 	_pressure_sub;      // pressure subscription
  	int     _v_att_sp_sub;
  	int     _v_att_sub; 
  	//int     _sensor_gyro_sub;
  	int     _sensor_combined_sub;
  	
	float 	_vzr;
	float   _zr;
	float 	_Fcz;


	float   _Gamma_c_x;
	float   _Gamma_c_y;
	float   _Gamma_c_z;

	float 	_depth_estimated;
	float 	_v_depth_estimated; 


  	int   	_armed_sub;       /**< arming status subscription */

	struct  pressure_s                   _pressure;                      //pressure
  	struct 	vehicle_rates_setpoint_s     _v_rates_sp;    /**< vehicle rates setpoint */
  	struct  vehicle_attitude_setpoint_s  _v_att_sp; 
  	struct  optical_flow_s               _optical_flow_p_sp; // optical_flow_pressure; //lhnguyen debug using optical_flow to send pressure data
  	struct  vehicle_attitude_s           _v_att;
  	//struct  sensor_gyro_s                _sensor_gyro;
  	struct  sensor_combined_s            _sensor_combined;
  	
  	

  	
  	struct 	actuator_armed_s       _armed;             /**< actuator arming status */

  	orb_advert_t _optical_flow_p_pub;




  	perf_counter_t  _loop_perf;     /**< loop performance counter */
  	perf_counter_t  _controller_latency_perf;

  	TailsitterRecovery *_ts_opt_recovery; /**< Computes optimal rates for tailsitter recovery */



	float 	joystick_deadband(float value, float threshold);
  	int     pwm_lookup_table(double throttle);
  	void    ForceMoment2Throttle(double Force[3], double Moment[3], double & thottle_0, 
                                                                                 double & throttle_1,
                                                                                 double & throttle_2,
                                                                                 double & throttle_3,
                                                                                 double & throttle_4,
                                                                                 double & throttle_5);
  	void    vehicle_rates_setpoint_poll();
  	void    arming_status_poll();

  	void 	pressure_poll();

  	void 	depth_estimate(float dt);

  	void	control_depth(float dt);

  	void    control_att(float dt);

  	static 	void task_main_trampoline(int argc, char *argv[]);
  	int    	task_main();     


};


namespace auv_att_control
{

	AUVAttitudeControl	*g_control;
}



AUVAttitudeControl::AUVAttitudeControl():
  _task_should_exit(false),
  _control_task(-1),

  _vzr(5.0),
  _zr(0.0),
  _Fcz(0.0),

  _depth_estimated(0.0),
  _v_depth_estimated(0.0),
  
  _armed_sub(-1),

  _pressure{},
  _v_rates_sp{},
  _v_att_sp{},
  _optical_flow_p_sp{},
  _v_att{},
  //_sensor_gyro{},
  _sensor_combined{},

  
  _armed{},


  /* performance counters */
  _loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
  _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
  _ts_opt_recovery(nullptr)

{
  //_rates_sp.zero();
}

//AUVAttitudeControl::~AUVAttitudeControl()
//{
//	//debug lhnguyen: can phai viet destructor, for deleting memory
//	delete auv_att_control::g_control;
//	auv_att_control::g_control = nullptr;
//
//}



AUVAttitudeControl::~AUVAttitudeControl()
{
  if (_control_task != -1) {
    /* task wakes up every 100ms or so at the longest */
    _task_should_exit = true;

    /* wait for a second for the task to quit at our request */
    unsigned i = 0;

    do {
      /* wait 20ms */
      usleep(20000);

      /* if we have given up, kill it */
      if (++i > 50) {
        px4_task_delete(_control_task);
        break;
      }
    } while (_control_task != -1);
  }

  if (_ts_opt_recovery != nullptr) {
    delete _ts_opt_recovery;
  }

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


void AUVAttitudeControl::ForceMoment2Throttle(double Force[3], double Moment[3], double & throttle_0, 
                                                                                 double & throttle_1,
                                                                                 double & throttle_2,
                                                                                 double & throttle_3,
                                                                                 double & throttle_4,
                                                                                 double & throttle_5){
        
        //AUV configuration, ideal case
        //float H1 = 0.0;  // ideal case
        //float L6 = 0.0; //ideal case


        double H6 = 0.15; // for example

        double L3 = 0.17; // for example
        double L5 = 0.19; // for example

        double W1 = 0.11; // for example
        double W3 = 0.11; // for example


        //double throttle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //Without constraints on throttles

        throttle_0 =  0.5*Force[0] - 0.5*Moment[2]/W1;

        throttle_1 =  0.5*Force[0] + 0.5*Moment[2]/W1;

        throttle_2 = -0.5*H6/W3*Force[1] -0.5*L5/(L3+L5)*Force[2] - 0.5*Moment[0]/W3 + 0.5/(L3+L5)*Moment[1];

        throttle_3 =  0.5*H6/W3*Force[1] -0.5*L5/(L3+L5)*Force[2] + 0.5*Moment[0]/W3 + 0.5/(L3+L5)*Moment[1];

        throttle_4 = -L3/(L3+L5)*Force[2] - 1.0/(L3+L5)*Moment[1];

        throttle_5 = Force[1];

        //With contraints on throttles
        //to be continue

}












int AUVAttitudeControl::pwm_lookup_table(double throttle )
{
        /*for motor voltage = 15V
          Originally, from T200 Bluerobotics website, there exit thrust and pwm  for two cases: 12V and 16V
          Interpolation: Thrust@15V = (Thrust@16V - Thrust@12V)*0.75 + Thrust@12V
          Using online Polynomial regression: http://www.xuru.org/rt/PR.asp#Manually
        */

        double pwm = 1500.0;
        
        // for:    -3.8124 < throttle < 4.7037 kgf

        if ((throttle > -3.8124) and (throttle < -1.1317)){
                pwm = 0.8874*throttle*throttle + 86.1898*throttle + 1425.6971; 
        }

        if ((throttle >= -1.1317) and (throttle < -0.02)){
                pwm = 32.0868*throttle*throttle + 159.8091*throttle + 1468.9169; 
        }
 
        // assume that motor can not generate thrust between +/-0.02 kgf 
        if ((throttle >= -0.02) and (throttle <= 0.02)) {
                pwm = 1500;
        }

        if ((throttle > 0.02) and (throttle <= 1.7916)){
                pwm = 18.3647*throttle*throttle*throttle - 75.3368*throttle*throttle + 176.4630*throttle + 1520.9868; 
        }

        if ((throttle > 1.7916) and (throttle <= 4.7037)){
                pwm = -2.1030*throttle*throttle + 82.5754*throttle + 1558.4906; 
        }           


        return round(pwm);

}

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

// int
// AUVAttitudeControl::start()
// {

// 	//subcribe to set_attitude_target topic
// 	int vehicle_rates_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
// 	// limit the update rate to 5 Hz
// 	px4_pollfd_struct_t fds = {};
// 	fds.events = POLLIN; 
// 	fds.fd = vehicle_rates_setpoint_sub_fd;


// 	//bool updated;
// 	//orb_check(_v_rates_sp_sub, &updated);

// 	//if (updated) {
// 	//	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
// 	//}


//         #if 0  //Debug
//         orb_set_interval(vehicle_rates_setpoint_sub_fd, 200);
//         #endif


// 	const char *dev= PWM_OUTPUT0_DEVICE_PATH;

// 	/* open for ioctl only */
// 	int fd = px4_open(dev, 0);
// 	if (fd < 0) {
// 			PX4_ERR("can't open %s", dev);
// 			return 1;
// 	}

// 	int ret;
// 	int pwm_value[6]  = {1500, 1500, 1500, 1500, 1500, 1500};
//         double throttle[6] = {-3.0, -0.5, 0.0, 0.5, 2.5, 4.5 }; //debug, for testing approximation function

//         double Force[3]  = {0.0, 0.0, 0.0}; //debug, for testing 
//         double Moment[3] = {0.0, 0.0, 0.0}; //debug, for testing 

	
// 	//int roll_pwm_value , pitch_pwm_value, yaw_pwm_value, thrust_pwm_value ;
// 	//roll_pwm_value = pitch_pwm_value = yaw_pwm_value = thrust_pwm_value = 1500;

       

// 	while (1) {
	
// 		/* wait for sensor update of 1 file descriptor for 10 ms (0.01 second) */
// 		int poll_ret = px4_poll(&fds, 1, 10);
		
//                 (void) poll_ret;  

// 		// timed out - periodic check for _task_should_exit 
//                 //lhnguyen debug: use the if following code block leads to the reaction of motors 
//                 //only when there are new messages from vehicle_rates_setpoint
// 		/*
//                 if (poll_ret == 0) {
// 			pwm_value = 1500; //Disarm pwm of BlueESC
//                         px4_ioctl(fd, PWM_SERVO_SET(0), pwm_value);
//                         px4_ioctl(fd, PWM_SERVO_SET(1), pwm_value);
//                         px4_ioctl(fd, PWM_SERVO_SET(2), pwm_value);
//                         px4_ioctl(fd, PWM_SERVO_SET(3), pwm_value);
//                         continue;
// 		}
//                 */

//                 if (fds.revents & POLLIN) {
//                 /* obtained data for the first file descriptor */
//  			struct vehicle_rates_setpoint_s raw;
// 			memset(&raw, 0, sizeof(raw));
// 			//copy sensors raw data into local buffer
// 			orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub_fd, &raw);
			

// 			//Apply joystick deadband, joystick_deadband = 0.1
//    		 	raw.roll  = joystick_deadband(raw.roll,0.1);
//    			raw.pitch = joystick_deadband(raw.pitch,0.1);
//    			raw.yaw   = joystick_deadband(raw.yaw,0.1);
//    			raw.thrust= joystick_deadband(raw.thrust,0.1);

//    			/* debug lhnguyen
//                         PX4_INFO("Debug AUV:\t% 1.6f\t %1.6f\t %1.6f\t% 1.6f",
// 								 (double)raw.roll,
// 								 (double)raw.pitch,
// 								 (double)raw.yaw,
// 								 (double)raw.thrust);
//                         */
   			
//    			//Convert joystick signals to pwm values, 
//    			//Neutral value =1500, according to T200 Bluerobotics motor characteristic
//    			//Take 1500 +/- 50 for giving small pwm value range
//    			/*
//                         roll_pwm_value   = 1500 + (int)((float)50.0*raw.roll);
//    			pitch_pwm_value  = 1500 + (int)((float)50.0*raw.pitch);
//    			yaw_pwm_value    = 1500 + (int)((float)50.0*raw.yaw);
//    			thrust_pwm_value = 1500 + (int)((float)50.0*raw.thrust); 
//                         */

//                         /* debug lhnguyen pwm output to motors 
//                         PX4_INFO("Debug AUV:\t% 6d\t %6d\t %6d\t% 6d",
//                                                                  roll_pwm_value,
//                                                                  pitch_pwm_value,
//                                                                  yaw_pwm_value,
//                                                                  thrust_pwm_value);*/

//                         Force[0]  = (float)15.0*raw.thrust;
//                         Force[1]  = 0.0; 
//                         Force[2]  = 0.0;
//                         Moment[0] =  (float)2.0*raw.roll;   
//                         Moment[1] =  (float)2.0*raw.pitch;    
//                         Moment[2] =  (float)2.0*raw.yaw;   
//                         /* debug lhnguyen pwm output to motors */
//                         PX4_INFO("Debug AUV: %1.6f  %1.6f  %1.6f %1.6f ",
//                                                                  Force[0],
//                                                                  Moment[0],
//                                                                  Moment[1],
//                                                                  Moment[2]);
                        
//    		 }

//                 /*
//                 //Test with some values of Force (in N) and Moment (in N.m)
//                  Force[0]  =  0.0;  Force[1]   = 0.0; Force[2]  = 0.0;
//                  Moment[0] =  0.0;   Moment[1] = 0.0; Moment[2] = 2.0;
//                 */


//                  //Calculate throttle (in N) of motors with given Force (N) and Moment (N.m)
//                  ForceMoment2Throttle(Force, Moment, throttle[0], throttle[1], throttle[2], throttle[3], throttle[4], throttle[5]);

                 
//                  //Taking into account CW (Clock Wise) or CCW (Counter Clock Wise) directions
//                  //CW: Thruster 2 and 4
//                  throttle[1] = 1.0*throttle[1];
//                  throttle[3] = 1.0*throttle[3];
                  
//                  //CCW: Thruster 1, 3 and 6
//                  throttle[0] = -1.0*throttle[0];
//                  throttle[2] = -1.0*throttle[2];
//                  throttle[5] = -1.0*throttle[5];

//                  //Change direction  of thruster 5 (throttle[4]) to fit with long watertight body
//                  throttle[4] = -1.0*throttle[4];
                

//                  for (unsigned i = 0; i < 6; i++) {  
                        
                        

//                         //convert from  N to kgf
//                         throttle[i] = (double)throttle[i] / 9.80665;

//                         //throttle = {-3.0, -0.5, 0.0, 0.5, 2.5, 4.5 }; //debug, for testing approximation function

//                         //lookup values, with values defined in kgf
//                         pwm_value[i] = pwm_lookup_table((double)throttle[i]);

//                         PX4_INFO("PWM_VALUE %d   %5d", i+1, pwm_value[i]);
//                         ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value[i]);       

//                         if (ret != OK) {
//                                 PX4_ERR("PWM_SERVO_SET(%d)", i);
//                                 return 1;
//                         }                 
//                  }
                 


//                 /*        
//                 for (unsigned i = 0; i < 4; i++) {                                     
//                         switch (i) {
//                                 case 0:
//                                         pwm_value = roll_pwm_value;
//                                         break;
//                                 case 1:
//                                         pwm_value = pitch_pwm_value;
//                                         break;
//                                 case 2:
//                                         pwm_value = yaw_pwm_value;
//                                         break;  
//                                 case 3:
//                                         pwm_value = thrust_pwm_value;
//                                         break;  
//                                 default:
//                                         pwm_value = 1500;
//                                         break;
//                                 }
                        
//                                 // PX4_INFO("PWM_VALUE  %5d", pwm_value);
//                                 //ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);
//                                 ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);       

//                                 if (ret != OK) {
//                                         PX4_ERR("PWM_SERVO_SET(%d)", i);
//                                         return 1;
//                                 }
                        
                   
//                         }
//                 */        

//                         /* Delay longer than the max Oneshot duration */
//                         //usleep(2542*10); //micro second

//                 #ifdef __PX4_NUTTX
//                         /* Trigger all timer's channels in Oneshot mode to fire
//                          * the oneshots with updated values.
//                          */
//                         up_pwm_update();
//                 #endif


// 		}
       
// }

void
AUVAttitudeControl::vehicle_rates_setpoint_poll()
{
  /* check if there is a new setpoint */
  bool updated;
  orb_check(_v_rates_sp_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
  }
}


void
AUVAttitudeControl::arming_status_poll()
{
  /* check if there is a new setpoint */
  bool updated;
  orb_check(_armed_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
  }
}

void
AUVAttitudeControl::pressure_poll()
{
  /* check if there is a new setpoint */
  bool updated;
  orb_check(_pressure_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(pressure), _pressure_sub, &_pressure);
  }
}




void
AUVAttitudeControl::depth_estimate(float dt)
{
	float k1 = 20.0;//2.0;
    	float k2 = 100.0;//1.0;
        
    	//double u = 0.0; //depth velocity
    	//float pressure_zero_level = 1030; 
    	float pressure_zero_level = 980;

	//depth and depth velocity observator
        // x = (_pressure - pressure_zero_level)*(float)100.0/(float)1000.0/(float)9.81; //*100 to convert to pascal; 10000 kg/m3 for fresh water; 
        // x_hat_dot = u_hat + k1*(x - x_hat);
        // u_hat_dot = k2*(x - x_hat);
        // x_hat += x_hat_dot*dt;
        // u_hat += u_hat_dot*dt; 

        //o_f_p.pixel_flow_x_integral  = x_hat;                             //depth
        //o_f_p.pixel_flow_y_integral  = (float)-1.0*u_hat;        //depth velocity

        float 	x_hat_dot = 0.0;
        float 	u_hat_dot = 0.0;
        float 	_depth_measured = 0.0;

        orb_copy(ORB_ID(pressure), _pressure_sub, &_pressure);

        _depth_measured     = (_pressure.pressure_mbar - pressure_zero_level)*(float)100.0/(float)1000.0/(float)9.81; //*100 to convert to pascal; 10000 kg/m3 for fresh water; 
        x_hat_dot           = _v_depth_estimated + k1*(_depth_measured - _depth_estimated);
        u_hat_dot           = k2*(_depth_measured - _depth_estimated);
        _depth_estimated   += x_hat_dot*dt;
        _v_depth_estimated += u_hat_dot*dt; 

    	//PX4_INFO("Debug depth 1: %1.6f  %1.6f  %1.6f", (double)_depth_measured, (double) _depth_estimated, (double)_v_depth_estimated);

        /*
    	//lhnguyen debug: publish by hi-jacking optical flow message 
    	_optical_flow_p_sp.pixel_flow_x_integral  =             _depth_estimated;          //depth
        _optical_flow_p_sp.pixel_flow_y_integral  =       -1.0f*_v_depth_estimated;        //depth velocity

        _optical_flow_p_sp.timestamp = hrt_absolute_time();
        // _replay_mode ? now : hrt_absolute_time();
	
        orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
	*/
}


void
AUVAttitudeControl::control_depth(float dt)
{       
	
      	/*
	//vehicle_rates_setpoint_poll();  //lhnguyen: ko lam viec
	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	_vzr =(float)-1.0*_v_rates_sp.thrust;  
	*/

	_vzr = 0.0f;

	//Apply deadband
	_vzr = joystick_deadband(_vzr,0.1);

	// choose 0.1 for smaller reference depth velocity input from joystick
	_vzr = (float)0.1*_vzr;

	//Update reference depth value
	_zr += _vzr*dt;  

	//limit min and max depth 
	if (_zr <= (float)0.2){
		_zr  = (float) 0.2;
		_vzr = (float) 0.0;
	} 

	if (_zr >= (float)0.6) {
		_zr = (float) 0.6;
		_vzr = (float) 0.0;
	}

	//Control gains
	float kp = 2.0;
	float kd = 1.0;
	float mass_total = 11.62 ; 

	depth_estimate(dt);

	_Fcz = mass_total*(-kp*(_depth_estimated - _zr) - kd*(_v_depth_estimated - _vzr));

	//PX4_INFO("Debug depth 2: %1.6f  %1.6f  %1.6f", (double)_zr, (double)_vzr, (double)dt);

}

void
AUVAttitudeControl::control_att(float dt)
{       
	
	orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	
	//orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub, &_sensor_gyro);
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	
	//For referent angular velocites and thrust
	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);




	Quaternion Q_temp = _v_att.q;
	Matrix<3, 3> R_hat    = Q_temp.to_dcm();


	Vector <3> Euler_angle_in_rad = Q_temp.to_euler(); 
       /*
        PX4_INFO("Debug Euler: %1.6f  %1.6f  %1.6f ", (double)57.3*(double)Euler_angle_in_rad(0), 
        							      (double)57.3*(double)Euler_angle_in_rad(1),
                                                                      (double)57.3*(double)Euler_angle_in_rad(2));    

	*/


	R_hat = R_hat.transposed();

	/*
	Vector<3> gamma = (R_hat.data[0][2],
			   R_hat.data[1][2],
			   R_hat.data[2][2]);
	*/

	Vector<3> e3(0.0f, 0.0f, 1.0f);
	Vector<3> gamma = R_hat * e3;

	
	Vector<3> gamma_d (0.0f, 0.0f, 1.0f);  //Should be input from joystick
	float omega_d = 0.0f;                  //Should be input from joystick
	
	//lhnguyen debug: Becareful about sign of components of gamma_d
	//nghieng max 30 deg -> magnitude 0.5 
	gamma_d(0) = -0.5f*joystick_deadband(_v_rates_sp.roll ,0.2);
	gamma_d(1) = -0.5f*joystick_deadband(_v_rates_sp.pitch,0.2);
	gamma_d(2) = sqrt(1.0f - gamma_d(0)*gamma_d(0) - gamma_d(1)*gamma_d(1));

	//max yaw_angular_velocity = pi/12
	omega_d    = 0.2617f*joystick_deadband(_v_rates_sp.yaw ,0.2);

	/*
        	//lhnguyen debug: Quaternion defined from joystick
        	_optical_flow_p_sp.pixel_flow_x_integral  =          gamma_d(0)   ;//         _v_att_sp.q_d[0];          
        	_optical_flow_p_sp.pixel_flow_y_integral  =    -1.0f*gamma_d(1)   ;//-1.0f*_v_att_sp.q_d[1];
        	_optical_flow_p_sp.gyro_x_rate_integral =            gamma_d(2)   ;//     _v_att_sp.q_d[2];
		_optical_flow_p_sp.gyro_y_rate_integral =      -1.0f*omega_d      ;//-1.0f*        _v_att_sp.q_d[3];
	
		_optical_flow_p_sp.timestamp = hrt_absolute_time();
        	
        	orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
	
	*/

	//Gain
	float k1 = 0.5f; //0.1f;
	float k2 = 1.5f; //1.0f;
	Matrix<3, 3> K;
	K.zero();
	K(0, 0) = k1;  K(1, 1) = k2;   K(2, 2) = 1.0f;


	Vector<3> Omega_d;
	//In order: Vector and then scalar value, for multiplication 
	Omega_d = (gamma_d % gamma) * k1 + gamma_d *  omega_d;


	//PX4_INFO("Debug Omega_d: %1.6f  %1.6f  %1.6f", (double)Omega_d(0), (double)Omega_d(1), (double)Omega_d(2));

	//Inerial matrix
	Matrix<3, 3> J;
	J.zero();
	J(0, 0) = 0.0842f;  J(0, 1) = 0.004f;   J(0, 2) = 0.005f;
	J(1, 0) = 0.004f;   J(1, 1) = 0.2643f;  J(1, 2) = 0.007f;
	J(2, 0) = 0.005f;   J(2, 1) = 0.007f;   J(2, 2) = 0.3116f;

	//Vector<3> Omega(0.0f, 0.0f, 0.0f);   //Should read from sensor??
	//Vector<3> Omega(_sensor_gyro.x, _sensor_gyro.y, _sensor_gyro.z);                                          //from sensor_gyro directly
	Vector<3> Omega(_sensor_combined.gyro_rad[0], _sensor_combined.gyro_rad[1], _sensor_combined.gyro_rad[2]);  //From sensor_combined with average value
	

	Vector<3> Omega_tilde = Omega - Omega_d; 


	Vector<3> JOmega = J*Omega;
	Vector3f temp (JOmega(0), JOmega(1), JOmega(2)); 

	Vector3f temp1(Omega_d(0), Omega_d(1), Omega_d(2));
	
	Vector3f temp2; 
	temp2 = temp.cross(temp1);

	Vector<3> temp3(temp2(0), temp2(1), temp2(2));

	Vector<3> Gamma_C;
	Gamma_C = -(J * K)*Omega_tilde - temp3;
	
	_Gamma_c_x = Gamma_C(0);
	_Gamma_c_y = Gamma_C(1);
 	_Gamma_c_z = Gamma_C(2);

 	//PX4_INFO("Debug Gamma_c: %1.6f  %1.6f  %1.6f", (double)_Gamma_c_x , (double)_Gamma_c_y , (double)_Gamma_c_z );
 	//PX4_INFO("Debug Sensor_gyro: %1.6f  %1.6f  %1.6f", (double)_sensor_gyro.x , (double)_sensor_gyro.y , (double)_sensor_gyro.z);

 	/*
 	//For debugging
	_optical_flow_p_sp.gyro_x_rate_integral =  _sensor_combined.gyro_rad[0];//+57.3f * Euler_angle_in_rad(0); //_Gamma_c_x;
	
	_optical_flow_p_sp.gyro_y_rate_integral = -_sensor_combined.gyro_rad[1];//-57.3f * Euler_angle_in_rad(1); //_Gamma_c_y;
	
	_optical_flow_p_sp.gyro_z_rate_integral = -_sensor_combined.gyro_rad[2];//-57.3f * Euler_angle_in_rad(2); // _Gamma_c_z;

	_optical_flow_p_sp.timestamp = hrt_absolute_time();
        // _replay_mode ? now : hrt_absolute_time();

        orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);

        //Test commit
        */

 	/*
 	//For debugging
	_optical_flow_p_sp.gyro_x_rate_integral =   _Gamma_c_x;
	
	_optical_flow_p_sp.gyro_y_rate_integral = - _Gamma_c_y;
	
	_optical_flow_p_sp.gyro_z_rate_integral = - _Gamma_c_z;

	_optical_flow_p_sp.timestamp = hrt_absolute_time();
       

        orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);

	*/


}




void
AUVAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
  auv_att_control::g_control->task_main();
}

int
AUVAttitudeControl::task_main()
{
 	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
 	_pressure_sub   = orb_subscribe(ORB_ID(pressure));
 	_v_att_sp_sub 	= orb_subscribe(ORB_ID(vehicle_attitude_setpoint));  //for disarm

 	
 	
 	_v_att_sub       = orb_subscribe(ORB_ID(vehicle_attitude));


 	//_sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
 	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));


 	_optical_flow_p_pub = orb_advertise(ORB_ID(optical_flow), &_optical_flow_p_sp); //  _press_topic;



  	/* wakeup source: gyro data from sensor selected by the sensor app */
 	px4_pollfd_struct_t poll_fds = {};
 	poll_fds.events = POLLIN;

  	// //subcribe to set_attitude_target topic
 	 // int vehicle_rates_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
 	 // // limit the update rate to 5 Hz
  	// px4_pollfd_struct_t fds = {};
  	// fds.events = POLLIN; 
  	// fds.fd = vehicle_rates_setpoint_sub_fd;

  	const char *dev= PWM_OUTPUT0_DEVICE_PATH;

 	 /* open for ioctl only */
  	int fd = px4_open(dev, 0);
  	if (fd < 0) {
      		PX4_ERR("can't open %s", dev);
      		return 1;
  	}

  	//int ret;
  	int pwm_value[6]  = {1500, 1500, 1500, 1500, 1500, 1500};
  	//double throttle[6] = {-3.0, -0.5, 0.0, 0.5, 2.5, 4.5 }; //debug, for testing approximation function
  	double throttle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //debug, for testing approximation function


  	double Force[3]  = {0.0, 0.0, 0.0}; //debug, for testing 
  	double Moment[3] = {0.0, 0.0, 0.0}; //debug, for testing 

  	
  	while (!_task_should_exit) {

    		poll_fds.fd = _v_rates_sp_sub;
    		int pret = px4_poll(&poll_fds, 1, 10);

   		/* timed out - periodic check for _task_should_exit */
   		//lhnguyen debug: comment to avoid periodic check joystick changes!!! Important in practice!!!
    		//if (pret == 0) {
    		//	PX4_INFO("Debug AUV continue");
     	 	//	continue;
    		//}



     		//struct vehicle_attitude_setpoint_s raw_att;
        	//memset(&raw_att, 0, sizeof(raw_att));
        	//copy sensors raw data into local buffer
        	//orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &raw_att);
        	orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);

        	
        	

        	/*
        	//lhnguyen debug: Quaternion defined from joystick
        	_optical_flow_p_sp.pixel_flow_x_integral  =           _v_att_sp.q_d[0];          
        	_optical_flow_p_sp.pixel_flow_y_integral  =     -1.0f*_v_att_sp.q_d[1];
        	_optical_flow_p_sp.gyro_x_rate_integral =             _v_att_sp.q_d[2];
		_optical_flow_p_sp.gyro_y_rate_integral =       -1.0f*_v_att_sp.q_d[3];
	
		_optical_flow_p_sp.timestamp = hrt_absolute_time();        	
        	orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
        	*/
		

        	// end of lhnguyen debug: Quaternion defined from joystick

        	/*
        	if ((_v_att_sp.q_d[0] < -0.5f) && (_v_att_sp.q_d[3] < -0.5f))  {
    			PX4_INFO("Emergency stop from joystick");

    			//lhnguyen debug: disarm to pwm = 1500 uc
    			for (unsigned i = 0; i < 6; i++) {                          
      			    			
      				int ret = px4_ioctl(fd, PWM_SERVO_SET(i), 1500);       

      				if (ret != OK) {
        				PX4_ERR("PWM_SERVO_SET(%d)", i);
        				return 1;
      				}                 
    			}

    			//lhnguyen debug: Exit from auv_att_control
    			_task_should_exit = true;
     	 		continue;

    		}
		*/
		

    		/* this is undesirable but not much we can do - might want to flag unhappy status */
   		if (pret < 0) {
      			warn("auv att ctrl: poll error %d, %d", pret, errno);
      			/* sleep a bit before next try */
      			usleep(100000);
      			continue;
    		}

    		/*
    		static uint64_t last_run = 0;
		float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
		last_run = hrt_absolute_time();

		// guard against too small (< 2ms) and too large (> 20ms) dt's //
		if (dt < 0.002f) {
		dt = 0.002f;

		} else if (dt > 0.02f) {
		dt = 0.02f;
		}

    		control_depth(dt);
    		*/



    		perf_begin(_loop_perf);

    		/* run controller on joystick changes */

    		//lhnguyen debug: Comment for avoiding joystick change checking!!!
    		//if (poll_fds.revents & POLLIN) {
    		if (true){
    	
        		static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			//lhnguyen debug: 
			//PX4_INFO("Debug AUV last_run = %lld", last_run);
			//PX4_INFO("Debug AUV dt = %1.6f", (double)dt);

			//lhnguyen debug: do not use this guard
			// guard against too small (< 2ms) and too large (> 20ms) dt's 
			//if (dt < 0.002f) {
			//	dt = 0.002f;

			//} else if (dt > 0.02f) {
			//	dt = 0.02f;
			//}	
	

     			struct vehicle_rates_setpoint_s raw;
        		memset(&raw, 0, sizeof(raw));
        		//copy sensors raw data into local buffer
        		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &raw);

        		pressure_poll();
      

        		//Apply joystick deadband, joystick_deadband = 0.1
        		raw.roll  = joystick_deadband(raw.roll,0.1);
        		raw.pitch = joystick_deadband(raw.pitch,0.1);
        		raw.yaw   = joystick_deadband(raw.yaw,0.1);
        		raw.thrust= joystick_deadband(raw.thrust,0.1);

        		//Depth control, calculate _Fcz
        		control_depth(dt);
        		//PX4_INFO("Debug depth: %1.6f  %1.6f ", (double)_zr, (double)_vzr);


      			// Force[0]  = (float)15.0*raw.thrust;
      			// Force[1]  = 0.0; 
      			// Force[2]  = 0.0;
      			// Moment[0] =  (float)2.0*raw.roll;   
      			// Moment[1] =  (float)2.0*raw.pitch;    
      			// Moment[2] =  (float)2.0*raw.yaw;   

			//Attitude control, calculate _Gamma_c_x, _Gamma_c_y, _Gamma_c_z
			control_att(dt); 

      			Force[0]  =  0.0f;
      			Force[1]  =  0.0f; 
      			Force[2]  =  0.0f*_Fcz;
      			Moment[0] =  1.0f*_Gamma_c_x;   
      			Moment[1] =  1.0f*_Gamma_c_y;    
      			Moment[2] =  1.0f*_Gamma_c_z;  
                        
      			/* debug lhnguyen pwm output to motors */
      			//PX4_INFO("Debug AUV: %1.6f  %1.6f  %1.6f %1.6f ", Force[2], Moment[0], Moment[1], Moment[2]);

      			


                                                                                     
      							


    		}

    		//Calculate throttle (in N) of motors with given Force (N) and Moment (N.m)
    		ForceMoment2Throttle(Force, Moment, throttle[0], throttle[1], throttle[2], throttle[3], throttle[4], throttle[5]);

    		/*
    		//Output to thrusters 
    		//Taking into account CW (Clock Wise) or CCW (Counter Clock Wise) directions
    		//CW: Thruster 2 and 4
    		throttle[1] = -1.0*throttle[1];
    		throttle[3] = -1.0*throttle[3];
                  
    		//CCW: Thruster 1, 3 and 6
    		throttle[0] = +1.0*throttle[0];
    		throttle[2] = +1.0*throttle[2];
    		throttle[5] = +1.0*throttle[5];

    		//Change direction  of thruster 5 (throttle[4]) to fit with long watertight body
    		throttle[4] = +1.0*throttle[4];

    		*/

    		//Output to thrusters 
    		//Taking into account CW (Clock Wise) or CCW (Counter Clock Wise) directions
    		//CW: Thruster 2 and 4
    		throttle[1] =      throttle[1];
    		throttle[3] = +1.0*throttle[3]; //Throttle 4
                  
    		//CCW: Thruster 1, 3 and 6
    		throttle[0] =      throttle[0];
    		throttle[2] = +1.0*throttle[2]; //Throttle 3 
    		throttle[5] =      throttle[5];

    		//Change direction  of thruster 5 (throttle[4]) to fit with long watertight body
    		throttle[4] = +1.0*throttle[4]; //Throttle 5


        	//lhnguyen debug: Send throttle
        	_optical_flow_p_sp.pixel_flow_x_integral  =         throttle[0]   ;//           
        	_optical_flow_p_sp.pixel_flow_y_integral  =    -1.0*throttle[1]   ;//
        	_optical_flow_p_sp.gyro_x_rate_integral   =         throttle[2]   ;//   
		_optical_flow_p_sp.gyro_y_rate_integral   =    -1.0*throttle[3]   ;//
		_optical_flow_p_sp.gyro_z_rate_integral   =    -1.0*throttle[4]   ;
		_optical_flow_p_sp.ground_distance_m      =         throttle[5]   ;
	
		_optical_flow_p_sp.timestamp = hrt_absolute_time();


        	
        	orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
        	
		

    		for (unsigned i = 0; i < 6; i++) {  
                        
      			//convert from  N to kgf
      			throttle[i] = (double)throttle[i] / 9.80665;

      			//throttle = {-3.0, -0.5, 0.0, 0.5, 2.5, 4.5 }; //debug, for testing approximation function

      			//lookup values, with values defined in kgf
      			pwm_value[i] = pwm_lookup_table((double)throttle[i]);

      			//PX4_INFO("PWM_VALUE %d   %5d", i+1, pwm_value[i]);
      			int ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value[i]);       

      			if (ret != OK) {
        			PX4_ERR("PWM_SERVO_SET(%d)", i);
        			return 1;
      			}                 
    		}

   	 	#ifdef __PX4_NUTTX
      		/* Trigger all timer's channels in Oneshot mode to fire
     	 	* the oneshots with updated values.	
      		*/
      		up_pwm_update();
    		#endif



   		 perf_end(_loop_perf);
  	}
 	 _control_task = -1;
  	return 0;

}



int
AUVAttitudeControl::start()
{
  ASSERT(_control_task == -1);

  /* start the task */
  _control_task = px4_task_spawn_cmd("auv_att_control",
             SCHED_DEFAULT,
             SCHED_PRIORITY_MAX - 5,
             1700,
             (px4_main_t)&AUVAttitudeControl::task_main_trampoline,
             nullptr);

  if (_control_task < 0) {
    warn("task start failed");
    return -errno;
  }

  return OK;
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









