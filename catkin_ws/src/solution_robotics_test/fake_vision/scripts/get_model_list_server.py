#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from fake_vision.srv import GetModelList, GetModelListResponse

class ModelListService:
    def __init__(self):
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.service = rospy.Service('get_model_list', GetModelList, self.handle_get_model_list)
        rospy.loginfo("Model List Service Ready.")

    def handle_get_model_list(self, req):
        try:
            substring = req.target
            world_properties = self.get_world_properties()
            filtered_model_names = []

            for model_name in world_properties.model_names:
                if substring in model_name:
                    model_state = self.get_model_state(model_name, "world")
                    pose = model_state.pose.position
                    # Y coordinate of the position is not lower-bounded to test for potential errors during the pick phase.
                    if pose.z > 0.7 and -0.5 < pose.x < 0.5 and pose.y < 0:  
                        filtered_model_names.append(model_name)

            return GetModelListResponse(filtered_model_names)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return []

def main():
    rospy.init_node('model_list_service')
    service = ModelListService()
    rospy.spin()

if __name__ == "__main__":
    main()