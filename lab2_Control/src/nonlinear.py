import numpy as np
import rospy

from controller import BaseController


# Uses Proportional-Differential Control from
# https://www.a1k0n.net/2018/11/13/fast-line-following.html
class NonLinearController(BaseController):
    def __init__(self):
        super(NonLinearController, self).__init__()
        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose):
        with self.path_lock:
            # TODO: INSERT CODE HERE
            # Determine a strategy for selecting a reference point
            # in the path. One option is to find the nearest reference
            # point and chose the next point some distance away along
            # the path.
            
            # Note: this method must be computationally efficient
            # as it is running directly in the tight control loop.

            dis = np.zeros(len(self.path))
            for i in range(len(self.path)):
                dis[i] = np.sqrt((pose[0] - self.path[i][0])**2 + (pose[1] - self.path[i][1])**2)
            idx = np.argmin(dis)
            coord = self.path[idx, :2]


            index = idx
            for i in range(idx + 1, len(self.path)):
               if np.linalg.norm(self.path[i, :2] - coord) > self.waypoint_lookahead:
                    return i

                    # if i<len(self.path)-1:
                    #     index=i
                    # else:
                    #     index=len(self.path)-1
                    #     exit()                        
            return index

    def get_control(self, pose, index):
        ref_pose = self.get_reference_pose(index)

        ep = self.get_error(pose, index)
        e_ct = ep[1]
        # if ref_pose[2] > np.pi:
        #     ref_pose[2] -=  2 * np.pi
        theta_e = pose[2] - ref_pose[2]
        
        while theta_e > np.pi:
            theta_e -= 2 * np.pi
        while theta_e < -np.pi:
            theta_e += 2 * np.pi
        
        print('error', theta_e) 
        # print('theta: ', pose[2], ref_pose[2])
        V_e = ref_pose[3] * np.sin(theta_e)
        u = np.arctan(-self.k1 * e_ct * self.car_length / theta_e * np.sin(theta_e) - self.k2 * self.car_length / self.speed * theta_e)
        # u += np.pi/2
        
        return [ref_pose[3], u]

    def reset_state(self):
        with self.path_lock:
            pass

    def reset_params(self):
        with self.path_lock:
            self.k1 = float(rospy.get_param("/nlc/k1", 5.0))
            self.k2 = float(rospy.get_param("/nlc/k2", 3.0))
            self.car_length = float(rospy.get_param("/nlc/car_length", 0.33))
            self.speed = float(rospy.get_param("/nlc/speed", 1.0))
            self.waypoint_lookahead = float(rospy.get_param("/nlc/waypoint_lookahead", 1.0))
            self.finish_threshold = float(rospy.get_param("/nlc/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/nlc/exceed_threshold", 4.0))
