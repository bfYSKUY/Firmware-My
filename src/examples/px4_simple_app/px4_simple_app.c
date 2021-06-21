/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/distance_sensor.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int distance_sensor_subs[ORB_MULTI_MAX_INSTANCES];

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	// //订阅雷达数据
	// int lider_sub_fd = orb_subscribe(ORB_ID(distance_sensor));
	// orb_set_interval(lider_sub_fd, 200);

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */

		//
		// { .fd = lider_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		//
		// int poll_ret_lider = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				PX4_INFO("Gyro_rad:\t%8.4f\t%8.4f\t%8.4f",   //自定义
					 (double)raw.gyro_rad[0],
					 (double)raw.gyro_rad[1],
					 (double)raw.gyro_rad[2]);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}

		// /* handle the poll result */
		// if (poll_ret_lider == 0) {
		// 	/* this means none of our providers is giving us data */
		// 	PX4_ERR("Got no data within a second");

		// } else if (poll_ret_lider < 0) {
		// 	/* this is seriously bad - should be an emergency */
		// 	if (error_counter < 10 || error_counter % 50 == 0) {
		// 		/* use a counter to prevent flooding (and slowing us down) */
		// 		PX4_ERR("ERROR return value from poll(): %d", poll_ret_lider);
		// 	}

		// 	error_counter++;

		// } else {

		// 	if (fds[1].revents & POLLIN) {
		// 		/* obtained data for the first file descriptor */
		// 		struct distance_sensor_s raw_lider;
		// 		/* copy sensors raw data into local buffer */
		// 		orb_copy(ORB_ID(distance_sensor), lider_sub_fd, &raw_lider);

		// 		PX4_INFO("Dis_D:\t%8.4f\tOrient:%d",   //自定义
		// 			 (float)raw_lider.current_distance,
		// 			 (uint8_t)raw_lider.orientation);
		// 	}

		// 	/* there could be more file descriptors here, in the form like:
		// 	 * if (fds[1..n].revents & POLLIN) {}
		// 	 */
		// }
	}

	// ///////
	// for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
	// 	distance_sensor_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	// }

	// bool updated;

	// struct distance_sensor_s lidar;
	// memset(&lidar, 0, sizeof(lidar));

	// for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

	// 	orb_check(distance_sensor_subs[i], &updated);

	// 	if (updated) {

	// 		orb_copy(ORB_ID(distance_sensor), distance_sensor_subs[i], &lidar);

	// 		if (lidar.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) {
	// 			updated = false;

	// 		} else {
	// 			lidar.current_distance += params.lidar_calibration_offset;
	// 			break; // only the first valid distance sensor instance is used
	// 		}
	// 	}
	// }

	// if (updated) { //check if altitude estimation for lidar is enabled and new sensor data

	// 	if (params.enable_lidar_alt_est && lidar.current_distance > lidar.min_distance
	// 		&& lidar.current_distance < lidar.max_distance
	// 		&& (R(2, 2) > 0.7f)) {

	// 		if (!use_lidar_prev && use_lidar) {
	// 			lidar_first = true;
	// 		}

	// 		use_lidar_prev = use_lidar;

	// 		lidar_time = t;
	// 		dist_ground = lidar.current_distance * R(2, 2); //vertical distance

	// 		if (lidar_first) {
	// 			lidar_first = false;
	// 			lidar_offset = dist_ground + z_est[0];
	// 			mavlink_log_info(&mavlink_log_pub, "[inav] LIDAR: new ground offset");
	// 			warnx("[inav] LIDAR: new ground offset");

	// 			PX4_INFO("Dis_D:\t%8.4f\tOrient:%d",   //自定义
	// 				 (float)lidar.current_distance,
	// 				 (uint8_t)lider.orientation);
	// 		}

	// 		corr_lidar = lidar_offset - dist_ground - z_est[0];

	// 		if (fabsf(corr_lidar) > params.lidar_err) { //check for spike
	// 			corr_lidar = 0;
	// 			lidar_valid = false;
	// 			lidar_offset_count++;

	// 			if (lidar_offset_count > 3) { //if consecutive bigger/smaller measurements -> new ground offset -> reinit
	// 				lidar_first = true;
	// 				lidar_offset_count = 0;
	// 			}

	// 		} else {
	// 			corr_lidar = lidar_offset - dist_ground - z_est[0];
	// 			lidar_valid = true;
	// 			lidar_offset_count = 0;
	// 			lidar_valid_time = t;
	// 		}

	// 	} else {
	// 		lidar_valid = false;
	// 	}
	// }

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
