############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
#
#   lhnguyen@i3s.unice.fr
#
############################################################################
px4_add_module(
	MODULE modules__auv_depth_control
	MAIN auv_depth_control
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS
	SRCS
		auv_depth_control_main.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
