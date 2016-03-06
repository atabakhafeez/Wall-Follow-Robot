#!/usr/bin/env python
import roslib 
import rospy
from std_msgs.msg import String
#import hokuyo_node
from sensor_msgs.msg import LaserScan
import json
from math import tan, cos, fabs

def slope(x1,y1,x2,y2):
    return  fabs(y2-y1)/fabs(x2-x1)

def segregate_lines(normalized_plot):
    result = []
    prev_slope = slope(normalized_plot[0]["tangent"], normalized_plot[0]["cosine"], normalized_plot[1]["tangent"], normalized_plot[1]["cosine"])
    begin_index, end_index = 0, 0
    for x in xrange(1,len(normalized_plot)-2):
        slp = slope(normalized_plot[x]["tangent"], normalized_plot[x]["cosine"], normalized_plot[x+1]["tangent"], normalized_plot[x+1]["cosine"])
        if slp >= prev_slope - 0.1 and slp <= prev_slope + 0.1:
            prev_slope = slp
            end_index += 1
            continue
        else:
            prev_slope = slp
            end_index += 1
            if end_index - begin_index > 3:
                result.append({"type": "line", "indices":[begin_index, end_index]})
                begin_index = end_index + 1
                end_index = begin_index
    return result
        
        
def find_circle(normalized_plot):
    result = []
    circle_threshold = 0
    prev_slope = slope(normalized_plot[0]["tangent"], normalized_plot[0]["cosine"], normalized_plot[1]["tangent"], normalized_plot[1]["cosine"])
    begin_index, end_index = 0, 0
    for x in xrange(1,len(normalized_plot)-2):
        slp = slope(normalized_plot[x]["tangent"], normalized_plot[x]["cosine"], normalized_plot[x+1]["tangent"], normalized_plot[x+1]["cosine"])
        if not slp >= prev_slope - 0.1 and slp <= prev_slope + 0.1:
            end_index = x
            circle_threshold += 1
        else:
            if circle_threshold > 15:
                result.append({"type":"circle", "indices":[end_index-circle_threshold, end_index]})
            begin_index, end_index = x, x
            circle_threshold = 0
        
    return result

def segregate(normalized_plot):
    return segregate_lines(normalized_plot) + find_circle(normalized_plot)



def callback(header):
    normalized_plot = [{"tangent":tan((index+1)*0.005826), "cosine":1/(value*cos((index+1)*0.005826)), "distance":value, "index":index} for index, value in enumerate(header.ranges)]
    segregate(normalized_plot)
    
#def circle_detect():
if __name__ == "__main__":

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'circle_detect' node so that multiple circle_detects can
    # run simultaneously.
    rospy.init_node('circle_detect', anonymous=True, log_level=rospy.INFO)
    sub = rospy.Subscriber("base_scan", LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("Starting spin")
    rospy.spin()

#if __name__ == '__main__':
#    circle_detect()
