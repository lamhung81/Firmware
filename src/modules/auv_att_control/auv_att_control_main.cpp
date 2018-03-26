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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <uORB/topics/position_setpoint.h>


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
    int     _manual_control_sp_sub;
    int     _v_force_sp_sub;
    int     _position_sp_sub;
  	
	float 	_vzr;
	float   _zr;
  float   _Fcx;
  float   _Fcy;
	float 	_Fcz;
  float   _Fcx_manual;
  float   _z_depth;

  Vector<3> _z_omega;
  Vector<3> _Omega_hat;
  Vector<3> _Delta_G_hat;


	float   _Gamma_c_x;
	float   _Gamma_c_y;
	float   _Gamma_c_z;

	float 	_depth_estimated;
	float 	_v_depth_estimated; 

  int _printing_time;


  	int   	_armed_sub;       /**< arming status subscription */

	struct  pressure_s                   _pressure;                      //pressure
  	struct 	vehicle_rates_setpoint_s     _v_rates_sp;    /**< vehicle rates setpoint */
  	struct  vehicle_attitude_setpoint_s  _v_att_sp; 
  	struct  optical_flow_s               _optical_flow_p_sp; // optical_flow_pressure; //lhnguyen debug using optical_flow to send pressure data
  	struct  vehicle_attitude_s           _v_att;
  	//struct  sensor_gyro_s                _sensor_gyro;
  	struct  sensor_combined_s            _sensor_combined;
  	struct  manual_control_setpoint_s    _manual_control_sp;
    struct  vehicle_force_setpoint_s     _v_force_sp;
    struct  position_setpoint_s          _position_sp;
      	

  	
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

    float satFunction(float x, float delta_x);
    Vector<3> sat3Function(Vector<3> x, float delta_x);


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
  _Fcx(0.0),
  _Fcy(0.0),
  _Fcz(0.0),
  _Fcx_manual(0.0),
  _z_depth(0.0),

  _z_omega(0.0f, 0.0f, 0.0f),
  _Omega_hat(0.0f, 0.0f, 0.0f),
  _Delta_G_hat(0.0f, 0.0f, 0.0f),

  _Gamma_c_x(0.0f),
  _Gamma_c_y(0.0f),
  _Gamma_c_z(0.0f),

  _depth_estimated(0.0),
  _v_depth_estimated(0.0),
  _printing_time(0),
  
  _armed_sub(-1),

  _pressure{},
  _v_rates_sp{},
  _v_att_sp{},
  _optical_flow_p_sp{},
  _v_att{},
  //_sensor_gyro{},
  _sensor_combined{},
  _manual_control_sp{},
  _v_force_sp{},
  _position_sp{},
  
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

        /*
        double H6 = 0.15; // for example

        double L3 = 0.17; // for example
        double L5 = 0.19; // for example

        double W1 = 0.11; // for exampl e
        double W3 = 0.11; // for example

        */

        //Corresponding to NewROV-1
        double H6 = -0.15108; 

        double L3 = 0.20945; 
        double L5 = 0.23458; 

        double W1 = 0.1108; 
        double W3 = 0.1108; 






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

        if ((throttle >= -1.1317) and (throttle < -0.0555)){
                pwm = 32.0868*throttle*throttle + 159.8091*throttle + 1468.9169; 
        }
        /*

        if ((throttle >= -0.0555) and (throttle < -0.003)){
                pwm = -4573.4176*throttle*throttle - 10.89221*throttle + 1473.5445; 
        }
      

        // assume that motor can not generate thrust between +/-0.003 kgf 
        //Motor can not generate when  1474<= pwm  <= 1523
        if ((throttle >= -0.003) and (throttle <= 0.003)) {
                pwm = 1500;
        }

        
        if ((throttle > 0.003) and (throttle <= 0.0566)) {
                pwm = -3908.3441*throttle*throttle + 345.0584*throttle + 1523.0;
        }

        */
        if ((throttle >= -0.0555) and (throttle < -0.01)){
                pwm = -4294.6327*throttle*throttle + 9.5768*throttle + 1473.8331; 
        }
      

        // assume that motor can not generate thrust between +/-0.003 kgf 
        //Motor can not generate when  1474<= pwm  <= 1523
        if ((throttle >= -0.01) and (throttle <= 0.01)) {
                pwm = 1500;
        }

        
        if ((throttle > 0.01) and (throttle <= 0.0566)) {
                pwm = 502.3376*throttle*throttle + 94.9766*throttle + 1523.0;
        }




       
        if ((throttle > 0.0566) and (throttle <= 1.7916)){
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


/*
void
AUVAttitudeControl::manual_control_setpoint_poll()
{
  // check if there is a new setpoint 
  bool updated;
  orb_check(_manual_control_sp_sub, &updated);
auv_att_control
  if (updated) {
    orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
  }
}
*/

void
AUVAttitudeControl::depth_estimate(float dt)
{
	float k1 = 10.0; // 20.0;//2.0;
    	float k2 = 25.0; // 100.0;//1.0;
        
    	//double u = 0.0; //depth velocity
    	//float pressure_zero_level = 1030; 
    	float pressure_zero_level = 997.0; //1010.0; //980;

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

        
    	//lhnguyen debug: publish by hi-jacking optical flow message 
    	_optical_flow_p_sp.pixel_flow_x_integral  =             _depth_estimated;          //depth
      _optical_flow_p_sp.pixel_flow_y_integral  =       -1.0f*_v_depth_estimated;        //depth velocity

      _optical_flow_p_sp.timestamp = hrt_absolute_time();
      // _replay_mode ? now : hrt_absolute_time();
	
      orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
	
}


void
AUVAttitudeControl::control_depth(float dt)
{       

  /*
  orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);

  float gia_tri = (float)1.0 * _manual_control_sp.x;
  PX4_INFO("Debug gui gia tri: %1.6f  ", (double)gia_tri);
	
  _vzr = gia_tri;
  */
      	/*
	//vehicle_rates_setpoint_poll();  //lhnguyen: ko lam viec
	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	_vzr =(float)-1.0*_v_rates_sp.thrust;  
	*/

  orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
  if ((_v_att_sp.q_d[0] < -0.5f) && (_v_att_sp.q_d[3] > -0.5f) ) {

          //PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          _vzr =            (float)1.0; //  _v_att_sp.q_d[0];

          }

  if ((_v_att_sp.q_d[0] > -0.5f) && (_v_att_sp.q_d[3] < -0.5f) ) {

          //PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          _vzr =(float) -1.0; //(float)-1.0* _v_att_sp.q_d[0];

          }
   if ((_v_att_sp.q_d[0] > -0.5f) && (_v_att_sp.q_d[3] > -0.5f) ) {

          //PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          _vzr =              (float) 0.0;

          }

   if ((_v_att_sp.q_d[0] < -0.5f) && (_v_att_sp.q_d[3] < -0.5f) ) {

          //PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          _vzr =              (float) 0.0;

          }

   //PX4_INFO("Reference depth velocity  %1.6f ", (double) _vzr );
	

  //_vzr = 0.0f;

	//Apply deadband
	//_vzr = joystick_deadband(_vzr,0.1);

	// choose 0.1 for smaller reference depth velocity input from joystick
	_vzr = (float)0.1*_vzr;

	//Update reference depth value
	_zr += _vzr*dt;  

	 float depth_top = 0.1;
  float depth_bottom = 1.0;

  //limit min and max depth 
  if (_zr <= depth_top){
    _zr  = depth_top;
    _vzr = (float) 0.0;
  } 

  if (_zr >= depth_bottom) {
    _zr = depth_bottom;
    _vzr = (float) 0.0;
  }

  /* 
  //Control without integrator

	//Control gains
	float kp = 2.0; //3.0;
	float kd = 1.0;
	float mass_total =17.0; // 11.62 ; 

	depth_estimate(dt);

	_Fcz = mass_total*(-kp*(_depth_estimated - _zr) - kd*(_v_depth_estimated - _vzr));

	//PX4_INFO("Debug depth 2: %1.6f  %1.6f  %1.6f", (double)_zr, (double)_vzr, (double)dt);
  */

/*
  //Control with integrator
  depth_estimate(dt);
  float h_tilde  = _depth_estimated - _zr;
  float hr_dot   = 0.0; //_vzr; because the joystick do not give the good value by using two buttons as above
  float k1_depth = 1.0;

  float vzd = - k1_depth* h_tilde + hr_dot;

  float vzd_dot = 0.0; // = - k1_depth* (_v_depth_estimated - hr_dot) + hr_ddot; //hr_ddot = 0;

  float k2_depth      = 1.4142;
  float delta_z_depth = 0.8;
  float kz_depth      = 2.0;

  float vz_tilde    = _v_depth_estimated - vzd;
  float vz_bar      = vz_tilde + _z_depth;
  float z_depth_dot = kz_depth*(-_z_depth + satFunction(vz_bar, delta_z_depth));
  _z_depth += z_depth_dot*dt;

  float Fcz_Inertial = 19.0f* (vzd_dot + satFunction(-k2_depth*vz_bar,0.5) - z_depth_dot);  //mz = 19.0 kg





  orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
  
  //orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub, &_sensor_gyro);
  orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
  
  //For referent angular velocites and thrust
  orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);


  Quaternion Q_temp = _v_att.q;
  Matrix<3, 3> R_hat    = Q_temp.to_dcm();

  Vector <3> Euler_angle_in_rad = Q_temp.to_euler(); 
 
  R_hat = R_hat.transposed();

  Vector<3> e3(0.0f, 0.0f, 1.0f);
  Vector<3> gamma = R_hat * e3;

  //Fc_Intertial = (0.0f, 0.0f, Fcz_Inertial) = e3*Fcz_Inertial;
  Vector<3> Fc_body = gamma*Fcz_Inertial;
  _Fcx = Fc_body(0);
  _Fcy = Fc_body(1);
  _Fcz = Fc_body(2);
*/

orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
  
  //orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub, &_sensor_gyro);
  orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
  
  //For referent angular velocites and thrust
  orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);


  Quaternion Q_temp = _v_att.q;
  Matrix<3, 3> R    = Q_temp.to_dcm();
   
  Matrix<3, 3> R_hat = R.transposed();

  Vector<3> e3(0.0f, 0.0f, 1.0f);
  Vector<3> gamma = R_hat * e3;


  //Assume that pressure sensor P is exactly on vertical symmestrical plane
  float d_PP1 = 0.01; // Distance from pressure sensor to projection point P1, along z axe, in meter
  float L_P1B = 0.3;  // Distance from projection P1 to center of boyancy, along x axe, in meter

  //Depth control is according to center of boyancy.
  //Need to convert the depth and depth_velocity measured at P (end of the tube) to the value of B with taking into account AUV rotation kinematics
  //

  float omega_1  = _v_att.rollspeed;
  float omega_2  = _v_att.pitchspeed; 
  float omega_3  = _v_att.yawspeed;

  //float temp_A = omega_1*R(1,0) - omega_2*(R(0,0) + R(2,2)) + omega_3*R(2,1);
  //float temp_C = omega_1*(R(1,2) - R(2,1)) + omega_2*(R(2,0) - R(0,2));

  float temp_A = omega_3*R(2,1) - omega_2*R(2,2);
  float temp_C = omega_2*R(2,0) - omega_1*R(2,1);

  float depth_B   = (float)_depth_estimated   + R(2,0)*L_P1B  + R(2,2)*d_PP1; // AUV depth measured at the centre of boyancy
  float v_depth_B = (float)_v_depth_estimated + temp_A*L_P1B  + temp_C*d_PP1; //AUV depth velocity measured at the centre of boyancy


  /* The following paragraph of code is not good since the rollspeed, pitchspeed and yawspeed in vehicle_attitude message are omega_1, omega_2 and omega_3
  // They are not derivatives of roll, pitch and yaw angles.
  // The given names are confusing!!!!
  double roll_velocity  = _v_att.rollspeed;
  double pitch_velocity = _v_att.pitchspeed; 
  Vector <3> Euler_angle_in_rad = Q_temp.to_euler(); 
  float roll  = Euler_angle_in_rad(0);
  float pitch = Euler_angle_in_rad(1);  
  //Depth control is according to center of boyancy.
  //Need to convert the depth and depth_velocity measured at P (end of the tube) to the value of B with taking into account AUV rotation kinematics
  //
  double depth_B   = (double)_depth_estimated + d_PP1*cos(roll)*cos(pitch) - L_P1B*sin(pitch); // AUV depth measured at the centre of boyancy
  double v_depth_B = (double)_v_depth_estimated - d_PP1*sin(roll)*cos(pitch)*roll_velocity -(d_PP1*cos(roll)*sin(pitch) + L_P1B*cos(pitch))*pitch_velocity; //AUV depth velocity measured at the centre of boyancy
  */

  depth_estimate(dt);
  float h_tilde  = (float)depth_B - _zr;
  float hr_dot   = 0.0; //_vzr; because the joystick do not give the good value by using two buttons as above
  float k1_depth = 0.5; // 1.0;

  float vzd = - k1_depth* h_tilde + hr_dot;

  float vzd_dot = 0.0; // = - k1_depth* (v_depth_B - hr_dot) + hr_ddot; //hr_ddot = 0;

  float k2_depth      = 0.2; //1.0;//1.4142;
  float delta_z_depth = 0.5; // 0.8;
  float kz_depth      = 2.0;

  float vz_tilde    = (float)v_depth_B - vzd;
  float vz_bar      = vz_tilde + _z_depth;
  float z_depth_dot = kz_depth*(-_z_depth + satFunction(vz_bar, delta_z_depth));

  _z_depth += z_depth_dot*dt;

  float Fcz_Inertial = 19.0f* (vzd_dot + satFunction(-k2_depth*vz_bar,0.5) - z_depth_dot);  //mz = 19.0 kg


  //Fc_Intertial = (0.0f, 0.0f, Fcz_Inertial) = e3*Fcz_Inertial;
  Vector<3> Fc_body = gamma*Fcz_Inertial;
  _Fcx = Fc_body(0);
  _Fcy = Fc_body(1);
  _Fcz = Fc_body(2);


  //if (_printing_time > 99) {
  //     PX4_INFO("Debug _z_depth: %1.6f ", (double)_z_depth );             
  //}



}

float AUVAttitudeControl::satFunction(float x, float delta_x)
{
  if (x > delta_x) {
    return delta_x;
  }
  else if (x < -delta_x){
    return -delta_x;
  }
  else{
    return x;
  }

}


Vector<3> AUVAttitudeControl::sat3Function(Vector<3> x, float delta_x)
{//(unsigned i = 0; i < 6; i++)
  //for (int i=0; i < 3; ++i){
  for (unsigned i = 0; i < 3; i++){
    if (x(i) > delta_x) {
       x(i) = delta_x;
    }
    else if (x(i) < -delta_x){
      x(i) = -delta_x;
    }
  } 
  return Vector<3> (x(0), x(1), x(2));
}

void
AUVAttitudeControl::control_att(float dt)
{       
	
	orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	
	//orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub, &_sensor_gyro);
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	
	//For referent angular velocites and thrust
	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);

  orb_copy(ORB_ID(vehicle_force_setpoint), _v_force_sp_sub, &_v_force_sp);
  orb_copy(ORB_ID(position_setpoint), _position_sp_sub, &_position_sp);

  //                                           NED                   0.5                  0.6                      0.7
  //PX4_INFO("Debug force_setpoint: %1.6f  %1.6f  %1.6f", (double)_v_force_sp.x , (double)_v_force_sp.y , (double)_v_force_sp.z );

  //                                           ENU                   0.6                  0.5                     -0.7
  PX4_INFO("Debug posit_setpoint: %1.6f  %1.6f  %1.6f", (double)_position_sp.x , (double)_position_sp.y , (double)_position_sp.z );
  PX4_INFO("Debug posit_setpoint_velo: %1.6f  %1.6f  %1.6f", (double)_position_sp.vx , (double)_position_sp.vy , (double)_position_sp.vz );



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
	gamma_d(0) =  0.707f*joystick_deadband(_v_rates_sp.pitch ,0.2);
	gamma_d(1) =  0.707f*joystick_deadband(_v_rates_sp.roll,0.2);
	gamma_d(2) = sqrt(1.0f - gamma_d(0)*gamma_d(0) - gamma_d(1)*gamma_d(1));

	//max yaw_angular_velocity = pi/12
	omega_d    = 5.0f*0.2617f*joystick_deadband(_v_rates_sp.yaw ,0.2);

//  Get from joystick by hi-jacking vehicle_rates_setpoint to send it
//
//
//                    < -    ================    + >   Yaw
//
//
//                                  /\ -  Pitch
//                                  || 
//                                  ||
//                    < +    ================    - >   Roll
//                                  ||
//                                  || 
//                                  \/  +
//
//  
  /*
        	//lhnguyen debug: Quaternion defined from joystick
          //Multiply with -1.0 because optical_flow change sign
        	_optical_flow_p_sp.pixel_flow_x_integral  =   1.0f*_v_rates_sp.roll ;  //        gamma_d(0)   ;//         _v_att_sp.q_d[0];          
        	_optical_flow_p_sp.pixel_flow_y_integral  =  -1.0f*_v_rates_sp.pitch;  //  -1.0f*gamma_d(1)   ;//-1.0f*_v_att_sp.q_d[1];
        	_optical_flow_p_sp.gyro_x_rate_integral =     1.0f*_v_rates_sp.yaw;    //        gamma_d(2)   ;//     _v_att_sp.q_d[2];
		      _optical_flow_p_sp.gyro_y_rate_integral =    -1.0f*_v_rates_sp.thrust; // -1.0f*omega_d      ;//-1.0f*        _v_att_sp.q_d[3];
	
		      _optical_flow_p_sp.timestamp = hrt_absolute_time();
        	
        	orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
	*/
	
  _Fcx_manual = 5.0f*joystick_deadband(_v_rates_sp.thrust ,0.2);


/*
  float k1 = 0.8f;
  Vector<3> Omega_d;
  //In order: Vector and then scalar value, for multiplication 
  Omega_d = (gamma_d % gamma) * k1 + gamma_d *  omega_d;

  Matrix<3, 3> J;
  J.zero();
  //For New-BlueROV1, total innertial
  J(0, 0) = 0.3105f;  J(0, 1) = 0.0000f;   J(0, 2) = 0.0000f;
  J(1, 0) = 0.0000f;  J(1, 1) = 0.8486f;   J(1, 2) = 0.0000f;
  J(2, 0) = 0.0000f;  J(2, 1) = 0.0000f;   J(2, 2) = 0.7176f;

  Vector<3> Omega(_v_att.rollspeed, _v_att.pitchspeed, _v_att.yawspeed); 

  Vector<3> JOmega = J*Omega;
  Vector3f temp (JOmega(0), JOmega(1), JOmega(2)); 

  Vector3f temp1(Omega_d(0), Omega_d(1), Omega_d(2));
  
  Vector3f temp2; 
  temp2 = temp.cross(temp1);

  Vector<3> temp3(temp2(0), temp2(1), temp2(2));

  Vector<3> Gamma_C;

  float k2 = 3.0f; 
  Matrix<3, 3> K;
  K.zero();
  K(0, 0) = k2;  K(1, 1) = k2;   K(2, 2) = 1.5f;

  Vector<3> Omega_tilde = Omega - Omega_d; 

  Gamma_C = -(J * K)*Omega_tilde - temp3;
  
  _Gamma_c_x = Gamma_C(0);
  _Gamma_c_y = Gamma_C(1);
  _Gamma_c_z = Gamma_C(2);

  _printing_time += 1;
  if (_printing_time > 50) {
       PX4_INFO("Debug Gamma_c: %1.6f  %1.6f  %1.6f", (double)_Gamma_c_x , (double)_Gamma_c_y , (double)_Gamma_c_z );
       _printing_time = 0;
  }
 
*/



	//Gain
  float k1 = 0.3f;
	//float k1 = 0.8f; //0.5f; //0.1f;

  
	//float k2 = 7.0f; //2.0f; //1.0f;
	//Matrix<3, 3> K;
	//K.zero();
	//K(0, 0) = k2;  K(1, 1) = k2;   K(2, 2) = 1.5f;




	Vector<3> Omega_d;
	//In order: Vector and then scalar value, for multiplication 
	Omega_d = (gamma_d % gamma) * k1 + gamma_d *  omega_d;

  Vector<3> Omega_d_dot(0.0f, 0.0f, 0.0f); //Approximation

	//PX4_INFO("Debug Omega_d: %1.6f  %1.6f  %1.6f", (double)Omega_d(0), (double)Omega_d(1), (double)Omega_d(2));

	//Inerial matrix
	Matrix<3, 3> J;
	J.zero();
	//J(0, 0) = 0.0842f;  J(0, 1) = 0.004f;   J(0, 2) = 0.005f;
	//J(1, 0) = 0.004f;   J(1, 1) = 0.2643f;  J(1, 2) = 0.007f;
	//J(2, 0) = 0.005f;   J(2, 1) = 0.007f;   J(2, 2) = 0.3116f;

  ///For New-BlueROV1
  //J(0, 0) = 0.12242f;  J(0, 1) = 0.004f;   J(0, 2) = 0.005f;
  //J(1, 0) = 0.004f;    J(1, 1) = 0.3516f;  J(1, 2) = 0.007f;
  //J(2, 0) = 0.005f;    J(2, 1) = 0.007f;   J(2, 2) = 0.3448f;

  //For New-BlueROV1, total innertial
  J(0, 0) = 0.3105f;  J(0, 1) = 0.0000f;   J(0, 2) = 0.0000f;
  J(1, 0) = 0.0000f;  J(1, 1) = 0.8486f;   J(1, 2) = 0.0000f;
  J(2, 0) = 0.0000f;  J(2, 1) = 0.0000f;   J(2, 2) = 0.7176f;


  Matrix<3, 3> J_inverted;
  J_inverted.zero();
  
  //For New-BlueROV1, total innertial
  J_inverted(0, 0) = 3.2206f;  J_inverted(0, 1) = 0.0000f;   J_inverted(0, 2) = 0.0000f;
  J_inverted(1, 0) = 0.0000f;  J_inverted(1, 1) = 1.1784f;   J_inverted(1, 2) = 0.0000f;
  J_inverted(2, 0) = 0.0000f;  J_inverted(2, 1) = 0.0000f;   J_inverted(2, 2) = 1.3935f;







	//Vector<3> Omega(0.0f, 0.0f, 0.0f);   //Should read from sensor??
	//Vector<3> Omega(_sensor_gyro.x, _sensor_gyro.y, _sensor_gyro.z);                                          //from sensor_gyro directly
	Vector<3> Omega(_sensor_combined.gyro_rad[0], _sensor_combined.gyro_rad[1], _sensor_combined.gyro_rad[2]);  //From sensor_combined with average value
	

	Vector<3> Omega_tilde = Omega - Omega_d; 

  //Vector<3> z_omega_dot = Omega_tilde ;
  Vector<3> z_omega_dot = (- _z_omega  + sat3Function (_z_omega + Omega_tilde, 0.8f) ) *2.0f ;


  _z_omega     += z_omega_dot*dt; 





//   Vector<3>  G_feedforward;  
//   G_feedforward = (J*Omega) % Omega_d - J*Omega_d_dot; 

//    Matrix<3, 3> K_Omega; 
//  K_Omega(0, 0) = 3.0f*0.3105f;  K_Omega(0, 1) = 0.0000f;        K_Omega(0, 2) = 0.0000f;
//  K_Omega(1, 0) = 0.0000f;       K_Omega(1, 1) = 3.0f*0.8486f;   K_Omega(1, 2) = 0.0000f;
//  K_Omega(2, 0) = 0.0000f;       K_Omega(2, 1) = 0.0000f;        K_Omega(2, 2) = 3.0f*0.7176f;

//  float Ki_Omega = 6.0;

 // float eta3 = 8.0;
  
//  Vector<3> Gg;  
//  Gg = (e3 % gamma) * 14.2*9.81*0.1;  //m*g*l e3 x RT e3

//  float a0 = 0.5;
//  float k0 = 20.0;  
    
//  Vector<3> G_control(_Gamma_c_x, _Gamma_c_y, _Gamma_c_z);

 // Vector<3> temp(0.0f, 0.0f, 0.0f);  
 // temp = (J*Omega) % _Omega_hat + G_control + Gg + _Delta_G_hat;

//  Vector<3> Omega_hat_dot(0.0f, 0.0f, 0.0f);
//  Omega_hat_dot = J_inverted*temp + (Omega - _Omega_hat)* k0;
  
//  _Omega_hat += Omega_hat_dot*dt; //Approximation  

//  Vector<3> Delta_G_hat_dot;
//  Delta_G_hat_dot = J*(Omega - _Omega_hat)* a0*a0*k0*k0;
  
//  _Delta_G_hat += Delta_G_hat_dot*dt; //Approximation

 // Vector<3> Gamma_C;
//  Gamma_C = -sat3Function(K_Omega*Omega_tilde, eta3) - _z_omega*Ki_Omega - G_feedforward - _Delta_G_hat;
  
//  _Gamma_c_x = Gamma_C(0);
 // _Gamma_c_y = Gamma_C(1);
//  _Gamma_c_z = Gamma_C(2);


  Vector<3> JOmega = J*Omega;

  Vector3f JOmega_vector (JOmega(0), JOmega(1), JOmega(2)); 

  Vector3f Omega_d_vector (Omega_d(0), Omega_d(1), Omega_d(2));
  
  Vector3f JOmega_Omega_d_vector; 
  JOmega_Omega_d_vector = JOmega_vector.cross(Omega_d_vector);
  Vector<3> JOmega_Omega_d(JOmega_Omega_d_vector(0), JOmega_Omega_d_vector(1), JOmega_Omega_d_vector(2));
  
  Vector<3>  G_feedforward;
  G_feedforward = JOmega_Omega_d - J*Omega_d_dot; 

  Matrix<3, 3> K_Omega; 

  //K_Omega(0, 0) = 3.0f*0.3105f;  K_Omega(0, 1) = 0.0000f;        K_Omega(0, 2) = 0.0000f;
  //K_Omega(1, 0) = 0.0000f;       K_Omega(1, 1) = 3.0f*0.8486f;   K_Omega(1, 2) = 0.0000f;
  //K_Omega(2, 0) = 0.0000f;       K_Omega(2, 1) = 0.0000f;        K_Omega(2, 2) = 3.0f*0.7176f;

  K_Omega(0, 0) = 1.0f*0.3105f;  K_Omega(0, 1) = 0.0000f;        K_Omega(0, 2) = 0.0000f;
  K_Omega(1, 0) = 0.0000f;       K_Omega(1, 1) = 1.0f*0.8486f;   K_Omega(1, 2) = 0.0000f;
  K_Omega(2, 0) = 0.0000f;       K_Omega(2, 1) = 0.0000f;        K_Omega(2, 2) = 1.0f*0.7176f;

  float Ki_Omega = 3.0; //6.0;

  float eta3 = 8.0;
  

  Vector3f e3_vector(e3(0), e3(1), e3(2));
  Vector3f gamma_vector(gamma(0), gamma(1), gamma(2)); 
  Vector3f e3RTe3_vector;
  e3RTe3_vector = e3_vector.cross(gamma_vector);

  Vector<3> e3RTe3 (e3RTe3_vector(0), e3RTe3_vector(1), e3RTe3_vector(2));

  Vector<3> Gg;
  Gg = e3RTe3*14.2*9.81*0.1;  //m*g*l e3 x RT e3

  float a0 = 0.5;
  float k0 = 20.0;
  
  Vector3f Omega_hat_vector (_Omega_hat(0), _Omega_hat(1), _Omega_hat(2));
  
  Vector3f JOmega_Omega_hat_vector; 
  JOmega_Omega_hat_vector = JOmega_vector.cross(Omega_hat_vector);

  Vector<3> JOmega_Omega_hat (JOmega_Omega_hat_vector(0), JOmega_Omega_hat_vector(1), JOmega_Omega_hat_vector(2));
  
  Vector<3> temp(0.0f, 0.0f, 0.0f);
  Vector<3> G_control(_Gamma_c_x, _Gamma_c_y, _Gamma_c_z);

  temp = JOmega_Omega_hat + G_control + Gg + _Delta_G_hat;

  Vector<3> Omega_hat_dot(0.0f, 0.0f, 0.0f);
  Omega_hat_dot = J_inverted*temp + (Omega - _Omega_hat)* k0;
  
  _Omega_hat += Omega_hat_dot*dt; //Approximation  

  _Omega_hat = sat3Function(_Omega_hat, 0.8);  //To prevent increasing too big!!!

  Vector<3> Delta_G_hat_dot;
  Delta_G_hat_dot = J*(Omega - _Omega_hat)* a0*a0*k0*k0;
  
  _Delta_G_hat += Delta_G_hat_dot*dt; //Approximation
  _Delta_G_hat = sat3Function(_Delta_G_hat, 0.8);  //To prevent increasing too big!!!

  Vector<3> Gamma_C;
  Gamma_C = -sat3Function(K_Omega*Omega_tilde, eta3) - _z_omega*Ki_Omega - G_feedforward - _Delta_G_hat*0.0f;

  //Gamma_C(0) = -satFunction(K_Omega(0,0)*Omega_tilde(0), eta3) - _z_omega(0)*Ki_Omega - G_feedforward(0) - _Delta_G_hat(0);
  //Gamma_C(1) = -satFunction(K_Omega(1,1)*Omega_tilde(1), eta3) - _z_omega(1)*Ki_Omega - G_feedforward(1) - _Delta_G_hat(1);
  //Gamma_C(2) = -satFunction(K_Omega(2,2)*Omega_tilde(2), eta3) - _z_omega(2)*Ki_Omega - G_feedforward(2) - _Delta_G_hat(2);
  
  _Gamma_c_x = Gamma_C(0)*1.0f;
  _Gamma_c_y = Gamma_C(1)*1.0f;
  _Gamma_c_z = Gamma_C(2)*1.0f;

/*
  _printing_time += 1;
  if (_printing_time > 100) {

       PX4_INFO("Debug Gamma_c: %1.6f  %1.6f  %1.6f", (double)_Gamma_c_x , (double)_Gamma_c_y , (double)_Gamma_c_z );
       PX4_INFO("Debug Delta_G_hat: %1.6f  %1.6f  %1.6f", (double)_Delta_G_hat(0) , (double)_Delta_G_hat(1) , (double)_Delta_G_hat(2) );
       PX4_INFO("Debug _Omega_hat: %1.6f  %1.6f  %1.6f", (double)_Omega_hat(0) , (double)_Omega_hat(1) , (double)_Omega_hat(2) );
       PX4_INFO("Debug _z_omega: %1.6f  %1.6f  %1.6f", (double)_z_omega(0) , (double)_z_omega(1) , (double)_z_omega(2) );

       _printing_time = 0;
  }

  */

  


	

  

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

  _v_force_sp_sub  = orb_subscribe(ORB_ID(vehicle_force_setpoint));
  _position_sp_sub = orb_subscribe(ORB_ID(position_setpoint));


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
        	//orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);

        	
        	

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
          //PX4_INFO("Emergency stop from joystick 1 %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double)_v_att_sp.q_d[3] );


/*
          if ((_v_att_sp.q_d[0] < -0.5f) ) {
          PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          }

          if ((_v_att_sp.q_d[0] > 0.5f) ) {
          PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          }
*/
          /*
        	 
        	if ((_v_att_sp.q_d[0] < -0.5f) && (_v_att_sp.q_d[3] < -0.5f))  {
    			PX4_INFO("Emergency stop from joystick %1.6f  %1.6f ", (double) _v_att_sp.q_d[0], (double) _v_att_sp.q_d[3] );
          }
         

          



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
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.05f) {
				dt = 0.05f;
			}	
	
          /*
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

            */

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

      			Force[0]  =  1.0f*_Fcx + 1.0f*_Fcx_manual;
      			Force[1]  =  1.0f*_Fcy; 
      			Force[2]  =  1.0f*_Fcz;
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

        /*
        if (_printing_time > 99) {
          PX4_INFO("Debug Throttle: %1.6f  %1.6f  %1.6f  %1.6f  %1.6f  %1.6f", (double)throttle[0] , (double)throttle[1] , (double)throttle[2], (double)throttle[3] , (double)throttle[4] , (double)throttle[5] );
       ;
        }
        */

          /*
        	//lhnguyen debug: Send throttle
        	_optical_flow_p_sp.pixel_flow_x_integral  =         throttle[0]   ;//           
        	_optical_flow_p_sp.pixel_flow_y_integral  =    -1.0*throttle[1]   ;//
        	_optical_flow_p_sp.gyro_x_rate_integral   =         throttle[2]   ;//   
		      _optical_flow_p_sp.gyro_y_rate_integral   =    -1.0*throttle[3]   ;//
		      _optical_flow_p_sp.gyro_z_rate_integral   =    -1.0*throttle[4]   ;
		      _optical_flow_p_sp.ground_distance_m      =         throttle[5]   ;
	
		      _optical_flow_p_sp.timestamp = hrt_absolute_time();


        	
        	orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
        	*/
		

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

        /*
        //lhnguyen debug: Send throttle
          _optical_flow_p_sp.pixel_flow_x_integral  =         pwm_value[0]   ;//           
          _optical_flow_p_sp.pixel_flow_y_integral  =    -1.0*pwm_value[1]   ;//
          _optical_flow_p_sp.gyro_x_rate_integral   =         pwm_value[2]   ;//   
          _optical_flow_p_sp.gyro_y_rate_integral   =    -1.0*pwm_value[3]   ;//
          _optical_flow_p_sp.gyro_z_rate_integral   =    -1.0*pwm_value[4]   ;
          _optical_flow_p_sp.ground_distance_m      =         pwm_value[5]   ;
  
          _optical_flow_p_sp.timestamp = hrt_absolute_time();


          
          orb_publish(ORB_ID(optical_flow), _optical_flow_p_pub, &_optical_flow_p_sp);
        */

   	 	#ifdef __PX4_NUTTX
      		/* Trigger all timer's channels in Oneshot mode to fire
     	 	* the oneshots with updated values.	
      		*/
      		up_pwm_update();
    		#endif



   		 perf_end(_loop_perf);
  	}

  
  orb_unsubscribe(_v_rates_sp_sub);
  orb_unsubscribe(_pressure_sub);
  orb_unsubscribe(_v_att_sp_sub);
  orb_unsubscribe(_v_att_sub);
  orb_unsubscribe(_sensor_combined_sub);
  orb_unsubscribe(_v_force_sp_sub);


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
             2500,
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









