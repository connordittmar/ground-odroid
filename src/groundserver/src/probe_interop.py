#!/usr/bin/env python
# Tester for interop validity

import rospy
import sys
from interop import Telemetry
from interop import AsyncClient
from pymavlink import mavutil
from std_msgs.msg import Float32

def usage():
    print('args: url, username, password')

def rate_publisher(client):
    pub = rospy.Publisher('/connections/judges/hz', Float32, queue_size=10)
    rospy.init_node('interop', anonymous=True)
    period = 4
    sent_since_print = 0
    last_print = rospy.get_time()
    while not rospy.is_shutdown():
        telemetry = Telemetry(0,0,0,0)
        telemetry_resp = client.post_telemetry(telemetry)
        sent_since_print += 1
        now = rospy.get_time()
        since_print = now - last_print
        if since_print > period:
            telem_rate = sent_since_print / since_print
            pub.publish(telem_rate)
            sent_since_print = 0
            last_print = now 
if __name__ == "__main__":
    if len(sys.argv) == 4:
        url = sys.argv[1]
        username = sys.argv[2]
        password = sys.argv[3]
        client = AsyncClient(url,username,password)
    else:
        usage()
        sys.exit(1)
    try:
        rate_publisher(client)
    except rospy.ROSInterruptException:
        rospy.logerr('ROSInterruptException Error')

