import numpy as np
import rospy
from controller import BaseController


class PurePursuitController(BaseController):
    def __init__(self):
        super(PurePursuitController, self).__init__()

        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose):
        '''
        get_reference_index finds the index i in the controller's path
            to compute the next control control against
        input:
            pose - current pose of the car, represented as [x, y, heading]
        output:
            i - referencence index
        '''
        with self.path_lock:
            # TODO: INSERT CODE HERE
            # Use the pure pursuit lookahead method described in the
            # handout for determining the reference index.
            #
            # Note: this method must be computationally efficient
            # as it is running directly in the tight control loop.
            dis = np.zeros(len(self.path))
            for i in range(len(self.path)):
                dis[i] = np.sqrt((pose[0] - self.path[i][0])**2 + (pose[1] - self.path[i][1])**2)-self.pose_lookahead
            idx = np.argmin(dis)
            coord = self.path[idx, :2]
            index=idx

            for i in range(idx + 1, len(self.path)):
                if np.linalg.norm(self.path[i, :2] - coord) > self.pose_lookahead:                 
                    return i

            return index

            #assert False, "Complete this function"

    def get_control(self, pose, index):
        '''
        get_control - computes the control action given an index into the
            reference trajectory, and the current pose of the car.
            Note: the output velocity is given in the reference point.
        input:
            pose - the vehicle's current pose [x, y, heading]
            index - an integer corresponding to the reference index into the
                reference path to control against
        output:
            control - [velocity, steering angle]
        '''
        # TODO: INSERT CODE HERE
        # First, compute the appropriate error.
        #
        # Then, use the pure pursuit control method to compute the
        # steering angle. Refer to the hand out and referenced
        # articles for more details about this strategy.
        B =0.285
        ref_pose = self.get_reference_pose(index)
        error_x = ref_pose[0]-pose[0]
        error_y = ref_pose[1]-pose[1]
        alpha = np.arctan2(error_y, error_x)-pose[2]
        L = np.sqrt(error_x**2+error_y**2)
        R = L/(2*np.sin(alpha))
        theta_dot = ref_pose[3]/R
        u = np.arctan2(2*B*np.sin(alpha),L)
        return [ref_pose[3], u]
        #assert False, "Complete this function"

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        pass

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.speed = float(rospy.get_param("/pid/speed", 1.0))
            self.finish_threshold = float(rospy.get_param("/pid/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/pid/exceed_threshold", 4.0))
            # Lookahead distance from current pose to next waypoint. Different from
            # waypoint_lookahead in the other controllers, as those are distance from
            # the reference point.
            self.pose_lookahead = float(rospy.get_param("/purepursuit/pose_lookahead", 1.0))
