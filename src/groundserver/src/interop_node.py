#!/usr/bin/env python

import rospy
import sys
from interop import Telemetry
from interop import AsyncClient
from pymavlink import mavutil
from std_msgs.msg import Float32
default_dev = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AL01NTBR-if00-port0'

def usage():
    print('args: url, username, password, device')

def rate_publisher(device,client):
    pub = rospy.Publisher('/connections/judges/hz', Float32, queue_size=10)
    rospy.init_node('interop', anonymous=True)
    period = 4 
    mav = mavutil.mavlink_connection(device, autoreconnect=True)
    sent_since_print = 0
    last_print = rospy.get_time()
    while not rospy.is_shutdown():
        msg = mav.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=10.0)
        if msg is None:
            rospy.loginfo('Did not receive mavlink message')
            sys.exit(1)
        telemetry = Telemetry(
            latitude = mavlin_latlon(msg.lat),
            longitude = mavlink_latlon(msg.lon),
            altitude_msl = mavlink_alt(msg.alt),
            uas_heading = mavlink_heading(msg.hdg))
        try:
            client.post_telemetry(telemetry)
        except:
            rospy.logerr('Telemetry Post Error')
            sys.exit(1)
        sent_since_print  += 1
        now = rospy.get_time()
        since_print = now - last_print
        if since_print > period:
            telem_rate = sent_since_print / since_print
            pub.publish(telem_rate)
            sent_since_print = 0
            last_print = now
            

if __name__ == "__main__":
    if len(sys.argv) == 5:
        url = sys.argv[1]
        username = sys.argv[2]
        password = sys.argv[3]
        device = sys.argv[4]
        client = AsyncClient(url,username,password)
    else:
        usage()
        sys.exit(1)
    try:
        rate_publisher(device,client)
    except rospy.ROSInterruptException:
        rospy.logerr('ROSInterruptException Error')
    
