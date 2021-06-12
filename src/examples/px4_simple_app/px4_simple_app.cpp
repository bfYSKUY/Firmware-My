/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/distance_sensor.h>  ///

// #include "../position_estimator_inav/position_estimator_inav_params.h"   ///

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int distance_sensor_subs[ORB_MULTI_MAX_INSTANCES];

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	// //订阅雷达数据
	// int lider_sub_fd = orb_subscribe(ORB_ID(distance_sensor));
	// orb_set_interval(lider_sub_fd, 200);

	///////
	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		distance_sensor_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	}

	bool updated;

	struct distance_sensor_s lidar;
	memset(&lidar, 0, sizeof(lidar));
	// struct position_estimator_inav_params params;
	// memset(&params, 0, sizeof(params));

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

		orb_check(distance_sensor_subs[i], &updated);

		if (updated) {

			orb_copy(ORB_ID(distance_sensor), distance_sensor_subs[i], &lidar);

			// if (lidar.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			// 	updated = false;

			// } else {
			// 	lidar.current_distance += params.lidar_calibration_offset;
			// 	break; // only the first valid distance sensor instance is used
			// }

			PX4_INFO("Dis_D:\t%8.4f",   //自定义   \tOrient:%d
					 (double)lidar.current_distance   //,(uint8_t)lider.orientation
					 );  //
		}
	}


	PX4_INFO("exiting");

	return 0;
}
