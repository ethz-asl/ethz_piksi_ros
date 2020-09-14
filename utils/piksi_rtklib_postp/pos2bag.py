import rosbag
import rospy
import sys
import csv

from datetime import datetime
import time
import calendar
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped

import geotf


class pos2bag(object):
    def __init__(self, observed_pos_dir, bag_dir):
        self.bag_dir = bag_dir
        self.observed_pos_dir = observed_pos_dir
        self.crop_ppk_solution_to_bag_ = False
        self.pos_ecef_topic = "/kiwi/piksi/position_receiver_0/ros/pos_ecef_cov"
        self.ppk_solution_topic = "/kiwi/ppk_position"
        self.reference_pos = np.array([47.234219638, 7.682116020, 477.5765])

        self.global_frame = "llh_wgs84"
        self.base_frame = "enu_base"
        self.obs_timestamps = []
        self.ros_msg_time = []
        self.obs_position = np.empty((0, 3), float)
        self.gc = geotf.GeodeticConverter()

        self.load_pos_file()
        self.initGeodeticConverter()

    def load_pos_file(self):
        # takes file from observed pos and loads and stores it in a somewhat smart manner
        with open(self.observed_pos_dir, "r") as file:
            reader = csv.reader(file, delimiter=" ", skipinitialspace=True)

            # Run through header
            in_header = True
            observation_time = None
            while in_header:
                header_line = next(reader)
                # Store which time is being used
                if len(header_line) > 1 and header_line[1] == "obs":
                    observation_time = header_line[6]
                if header_line[0] != "%":
                    in_header = False

            # Abort if not UTC time used
            if observation_time != "UTC":
                print(
                    "ERROR: Observation time has to be in UTC. Anything else is not supported."
                )
                sys.exit(1)

            # load content
            time_format_str = "%Y/%m/%d %H:%M:%S.%f-%Z"
            for row in reader:
                self.obs_timestamps.append(
                    datetime.strptime(row[0] + " " + row[1] + "-UTC", time_format_str)
                )
                self.obs_position = np.append(
                    self.obs_position,
                    np.array([[float(row[2]), float(row[3]), float(row[4])]]),
                    axis=0,
                )

    def initGeodeticConverter(self):
        if not self.gc.addFrameByENUOrigin(
            self.base_frame,
            self.reference_pos[0],
            self.reference_pos[1],
            self.reference_pos[2],
        ):
            print("Error: Could not add Frame: " + self.base_frame + " by ENU origin.")
            sys.exit(1)

        if not self.gc.addFrameByGCSCode(self.global_frame, "WGS84"):
            print("Error: Could not add Frame: " + self.global_frame + " by GCSCode.")
            sys.exit(1)

        if self.gc.canConvert(self.global_frame, self.base_frame):
            return True
        else:
            print(
                "Error: Loading of frames failed. Cannot convert between global and local frame"
            )
            sys.exit(1)

    def add_observations_to_bag(self, bag_suffix, outbag_dir=None):
        if bag_suffix == "":
            bag_suffix = "_with_observations"
        if outbag_dir == None:
            outbag_dir = self.bag_dir.split(".")[0] + bag_suffix
        else:
            inbag_name = self.bag_dir.split("/")[-1].split(".")[0]
            outbag_dir = outbag_dir + "/" + inbag_name + bag_suffix

        # Opening dem bags
        inbag = rosbag.Bag(self.bag_dir)
        outbag = rosbag.Bag(outbag_dir + ".bag", "w")

        print("Computing time difference")
        time_diff = np.empty(0, float)
        for topic, msg, t in inbag.read_messages(topics=[self.pos_ecef_topic]):
            time_diff = np.append(time_diff, msg.header.stamp.to_sec() - t.to_sec())

        if time_diff.size == 0:
            print("Error: No time difference received from bag.")
            sys.exit(1)
        # Skip first value because its akwardly huge and skews mean
        time_diff = time_diff[1:]
        mean_time_offset = np.mean(time_diff)

        # Convert timestamps to UTC
        obs_ros_time = []
        for dt_time in self.obs_timestamps:
            # Convert to utc and add bag offset
            dt_time_utc_offset = (
                calendar.timegm(dt_time.timetuple())
                + dt_time.microsecond / 1e6
                + mean_time_offset
            )
            # Make it ros time object
            obs_ros_time.append(rospy.Time.from_sec(dt_time_utc_offset))

        # Crop observations to bag start and stop time
        if self.crop_ppk_solution_to_bag_:
            # Store start & end time of bag
            inbag_t_start = inbag.get_start_time()
            inbag_t_end = inbag.get_end_time()
            obs_ros_time_cropped = []
            obs_position_cropped = np.empty((0, 3), float)

            # Cropping info..
            print("Cropping Observations to bag start and end time.")
            print(
                "Bag start/end time:\t" + str(inbag_t_start) + "\t" + str(inbag_t_end)
            )
            print(
                "Obs start/end time:\t"
                + str(obs_ros_time[0])
                + "\t"
                + str(obs_ros_time[-1])
            )
            # # Get first and last timestamp of observation
            # obs_t_start = (
            #     calendar.timegm(self.obs_timestamps[0].timetuple())
            #     + self.obs_timestamps[0].microsecond / 1e6
            #     + mean_time_offset
            # )
            # obs_t_end = (
            #     calendar.timegm(self.obs_timestamps[-1].timetuple())
            #     + self.obs_timestamps[-1].microsecond / 1e6
            #     + mean_time_offset
            # )

            # Crop messages
            for timestamp, g_position in zip(obs_ros_time, self.obs_position):
                if timestamp.to_sec() >= inbag_t_start and timestamp.to_sec() <= inbag_t_end:
                    obs_ros_time_cropped.append(timestamp)
                    obs_position_cropped = np.append(
                        obs_position_cropped, [g_position], axis=0
                    )

            if len(obs_ros_time_cropped) == 0 or len(obs_position_cropped) == 0:
                print("ERROR: Observations seem not to overlap with bag times.")
                sys.exit(1)
        else:
            # Don't change size of observations
            print(
                "WARNING: Observation messages are not being cropped to rosbag start & end time."
            )
            obs_ros_time_cropped = obs_ros_time
            obs_position_cropped = self.obs_position

        # Writing the topic from inbag
        print("Copying all topics to new bag")
        for topic, msg, t in inbag.read_messages():
            outbag.write(topic, msg, t)

        print("Adding robot positions to new bag")
        msg_idx = 0
        for obs_timestamp, g_position in zip(
            obs_ros_time_cropped, obs_position_cropped
        ):
            # Convert from global to base frame
            b_position = self.gc.convert(self.global_frame, g_position, self.base_frame)

            msg = PointStamped()
            msg.header.frame_id = self.base_frame
            msg.header.seq = msg_idx

            msg.header.stamp = obs_timestamp

            msg.point.x = b_position[0]
            msg.point.y = b_position[1]
            msg.point.z = b_position[2]
            outbag.write(self.ppk_solution_topic, msg, obs_timestamp)
            msg_idx += 1

        outbag.close()

        plt.plot(
            np.arange(0, len(time_diff), 1),
            time_diff,
            "b",
            np.arange(0, len(time_diff), 1),
            np.ones(len(time_diff)) * mean_time_offset,
            "r",
        )
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    dir_arg = sys.argv[1]
    sol_arg = sys.argv[2]
    pos2bag = pos2bag(sol_arg, dir_arg)
    if len(sys.argv) == 4:
        pos2bag.add_observations_to_bag("_with_ppk_obs", outbag_dir=sys.argv[3])
    else:
        pos2bag.add_observations_to_bag("_with_ppk_obs")
