#!/usr/bin/env python3
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from collections import defaultdict
import numpy as np
from scipy import stats

class DiagnosticStats:
    def __init__(self):
        # Store all data points
        self.data = defaultdict(list)

        rospy.Subscriber("/planner_diagnostics", DiagnosticArray, self.callback)
        rospy.loginfo("Listening to /planner_diagnostics...")

    def callback(self, msg):
        for status in msg.status:
            if status.name != "planner":
                continue
            values = {}
            for kv in status.values:
                values[kv.key] = float(kv.value)
            if values.get("Expansion Count", 0) < 10:
                continue
            # Store values
            self.data["Expansion Count"].append(values.get("Expansion Count", 0))
            self.data["Time Taken (s)"].append(values.get("Time Taken (s)", 0.0))
            self.data["Expansions/second"].append(values.get("Expansions/second", 0.0))

            self.print_stats()

    def print_stats(self):
        rospy.loginfo("==== Planner Diagnostics Statistics ====")
        for key in ["Expansion Count", "Time Taken (s)", "Expansions/second"]:
            arr = np.array(self.data[key])
            n = len(arr)
            if n < 2:
                mean = np.mean(arr)
                rospy.loginfo(f"{key}: mean = {mean:.4f} (waiting for more data for confidence interval)")
            else:
                mean = np.mean(arr)
                sem = stats.sem(arr)
                ci = stats.t.interval(0.95, df=n-1, loc=mean, scale=sem)
                rospy.loginfo(f"{key}: mean = {mean:.4f}, 95% CI = [{ci[0]:.4f}, {ci[1]:.4f}]")

        rospy.loginfo("========================================\n")

if __name__ == "__main__":
    rospy.init_node("diagnostic_listener")
    DiagnosticStats()
    rospy.spin()
