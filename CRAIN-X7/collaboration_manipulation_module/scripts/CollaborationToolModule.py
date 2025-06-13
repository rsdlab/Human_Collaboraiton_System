#!/usr/bin/env python3
# coding: UTF-8

import rospy

class CollaborationTool:
    #ノードを初期化する.
    @classmethod
    def init_node(cls, name):
        rospy.init_node(name)
    @classmethod
    def signal_shutdown(cls, reason):
        rospy.signal_shutdown(reason)
    @classmethod
    def wait_for_service(cls, servicename):
        rospy.wait_for_service(servicename)
    @classmethod
    def loginfo(cls, message, *args, **kwargs):
        rospy.loginfo(message, *args, **kwargs)
    @classmethod
    def logwarn(cls, message, *args, **kwargs):
        rospy.logwarn(message, *args, **kwargs)
    @classmethod
    def wait_time(cls, time):
        rospy.sleep(time)
        #return 'succeeded'
    @classmethod
    def create_time(cls, time):
        return rospy.Time(time)
    @classmethod
    def create_duration(cls, num):
        return rospy.Duration(num)
    @classmethod
    def is_shutdown(cls):
        return rospy.is_shutdown()
    @classmethod
    def ServiceException(cls):
        return rospy.exceptions()