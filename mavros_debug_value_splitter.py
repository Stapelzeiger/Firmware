import rospy
from mavros_msgs.msg import DebugValue

publishers = {}

def callback(data):
    key = data.name
    if key not in publishers:
        publishers[key] = rospy.Publisher('/'+key, DebugValue, queue_size=10)
        rospy.loginfo(rospy.get_caller_id() + "new topic %s", key)
    publishers[key].publish(data)

if __name__ == '__main__':
    rospy.init_node('mavros_debug_value_splitter', anonymous=True)
    rospy.Subscriber("/mavros/debug_value/debug_vector", DebugValue, callback)
    rospy.Subscriber("/mavros/debug_value/named_value_float", DebugValue, callback)
    rospy.Subscriber("/mavros/debug_value/named_value_int", DebugValue, callback)

    rospy.spin()
