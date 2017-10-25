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

extern "C" __EXPORT int auv_depth_control_main(int argc, char *argv[]);

class AUVDepthControl
{
public:
	/**
	 * Constructor
	 */
	AUVDepthControl();

	/**
	 * Destructor, also kills the main task
	 */
	~AUVDepthControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int	_control_task;			/**< task handle */

	int	_v_rates_sp_sub;		// vehicle rates setpoint subscription  lhnguyen: hi-jacked to transfer joystick signals

	
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */

	/**
	 * Update our local parameter cache.
	 */
	//int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	//void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	//void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	//void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	//void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_depth_velocity_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_depth(float dt);

	/**
	 * Attitude rates controller.
	 */
	//void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	//math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Check for vehicle status updates.
	 */
	//void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	//void		vehicle_motor_limits_poll();

	/**
	 * Check for battery status updates.
	 */
	//void		battery_status_poll();

	/**
	 * Check for control state updates.
	 */
	//void		control_state_poll();

	/**
	 * Check for sensor thermal correction updates.
	 */
	//void		sensor_correction_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace auv_depth_control
{
AUVDepthControl	*g_control;
}

AUVDepthControl::AUVDepthControl() :

	_task_should_exit(false),
	_control_task(-1),

	//subscription
	_v_att_sp_sub(-1),


	_v_rates_sp{}, 

	_loop_perf(perf_alloc(PC_ELAPSED, "auv_depth_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
{

}

AUVDepthControl::~AUVDepthControl()
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

	auv_depth_control::g_control = nullptr;
}



// int
// AUVDepthControl::parameters_update() //lhnguyen: trong file nay can phai update gia tri depth thuc te
// {


// }

// void
// AUVDepthControl::parameter_update_poll() 
// {
// 	bool updated;

// 	/* Check if parameters have changed */
// 	orb_check(_params_sub, &updated);

// 	if (updated) {
// 		struct parameter_update_s param_update;
// 		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
// 		parameters_update();
// 	}
// }


void
MulticopterAttitudeControl::vehicle_depth_velocity_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);   //lhnguyen: Gia tri cua vz reference or vzr duoc lay tu joystick

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}





void
AUVDepthControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}


void
AUVDepthControl::control_depth(float dt)
{	
	//vehicle_attitude_setpoint_poll();
	//_thrust_sp = _v_att_sp.thrust;  


	//_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_rates_sp);  // lhnguyen: lay tu mavlink_receiver

	vehicle_depth_velocity_setpoint_poll();
	_vzr = _v_rates_sp.roll;         //lhnguyen: thong tin lay tu  _v_rates_sp_sub, duoc luu vao _v_rates_sp,. Gia su gia tri vzr duoc gui qua roll

	PX4_INFO("Vz reference:%1.6f ",  (double)_vzr);


}


void
AUVDepthControl::task_main_trampoline(int argc, char *argv[])
{
	auv_depth_control::g_control->task_main();
}





void
AUVDepthControl::task_main()
{	

	//_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));

	/* initialize parameters cache */
	//parameters_update();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	while (!_task_should_exit) {
		poll_fds.fd = _v_rates_sp_sub;

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("auv depth ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */  //lhnguyen: theo su thay doi cua joystick
			//orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);


			//auv_depth_control	

			control_depth(dt);

			//orb_publish

		}		

		perf_end(_loop_perf);


	}

	_control_task = -1;
}








int
AUVDepthControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("auv_depth_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&AUVDepthControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int auv_depth_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: auv_depth_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (auv_depth_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		auv_depth_control::g_control = new AUVDepthControl;

		if (auv_depth_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != auv_depth_control::g_control->start()) {
			delete auv_depth_control::g_control;
			auv_depth_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (auv_depth_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete auv_depth_control::g_control;
		auv_depth_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (auv_depth_control::g_control) {
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