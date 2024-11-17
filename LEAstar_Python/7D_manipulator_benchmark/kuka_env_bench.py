import numpy as np
import pybullet as p
import pybullet_data
import pickle
from time import time, sleep
from timer import Timer
    
import matplotlib.pyplot as plt

class KukaEnv:
    '''
    Interface class for maze environment
    '''
    RRT_EPS = 0.1 #0.5

    def __init__(self, GUI=False):
        # print("Initializing environment...")

        self.collision_check_count = 0
        self.collision_time = 0
        self.collision_point = None
        self.collision_info = []

        if GUI:
            p.connect(p.GUI, options='--width=1200 --height=900')
        else:
            p.connect(p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, lightPosition = [0, 0, 0.1])

        self.timer = Timer()

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
        p.performCollisionDetection()

        self.tableId = p.loadURDF("table/table.urdf", [0, 0, -.6], [0, 0, 0, 1], useFixedBase=True)
        self.table2Id = p.loadURDF("table/table.urdf", [0, 1, -.6], [0, 0, 0, 1], useFixedBase=True)
        self.planeId = p.loadURDF("plane.urdf", [0, 0, -.6], [0, 0, 0, 1], useFixedBase=True)


        self.config_dim = p.getNumJoints(self.kukaId)

        if self.config_dim == 7:
            # width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()

            camera_target_position = [0, .5, 0.5]
            cam_yaw = 180
            cam_pitch = -20
            cam_dist = 1.5
            
            p.resetDebugVisualizerCamera(
                cameraDistance=cam_dist,
                cameraYaw=cam_yaw,
                cameraPitch=cam_pitch,
                cameraTargetPosition=camera_target_position)

            self.projectionMatrix = p.computeProjectionMatrixFOV(
                                    fov=55.0,
                                    aspect=1.0,
                                    nearVal=0.1,
                                    farVal=8.0
                                    )

            self.viewMatrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=camera_target_position,
                distance=cam_dist,
                yaw= cam_yaw,
                pitch=cam_pitch,
                roll=0,
                upAxisIndex = 2
            )
            # # alternate way to represent camera
            # self.viewMatrix = p.computeViewMatrix(
            #     cameraEyePosition=[2, 2, 2],
            #     cameraTargetPosition=camera_target_position,
            #     cameraUpVector= [0,0,1]
            # )
            
            self.viewMatrix_2 = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=camera_target_position,
                distance=cam_dist,
                yaw= -45,
                pitch=cam_pitch,
                roll=0,
                upAxisIndex = 2
            )


        self.pose_range = [(p.getJointInfo(self.kukaId, jointId)[8], p.getJointInfo(self.kukaId, jointId)[9]) for
                           jointId in range(p.getNumJoints(self.kukaId))]
        self.bound = np.array(self.pose_range).T.reshape(-1)
        self.kukaEndEffectorIndex = self.config_dim - 1
        p.setGravity(0, 0, -10)
    
    def disconnect(self):
        p.disconnect()

    '''
    def add_visual_cube(self, pos):
        bodyid = p.loadURDF("cube.urdf", pos, globalScaling=0.05, flags=p.URDF_IGNORE_COLLISION_SHAPES) # visualize vertex
        return bodyid
    
    def get_collision_position(self):
        if self.collision_info:
            col_arr = np.array(self.collision_info)
            dists = np.linalg.norm(col_arr, axis=1)
            return [col_arr[np.argmin(dists)], col_arr[np.argmax(dists)]]
        else:
            return []

    def get_links_position(self, config=None):
        # Gets lines in the center of the robot links
        points = []
        if config is not None:
            for i in range(p.getNumJoints(self.kukaId)):
                p.resetJointState(self.kukaId, i, config[i])
        for effector in range(self.kukaEndEffectorIndex + 1):
            point = p.getLinkState(self.kukaId, effector)[4]
            point = (point[0], point[1], point[2])
            points.append(point)
        return points
    '''

    def remove_body(self, bodyid):
        p.removeBody(bodyid)

    def create_voxel(self, halfExtents, basePosition):
        groundColId = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
        groundVisID = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          rgbaColor=np.random.uniform(0, 1, size=3).tolist() + [0.8],
                                          specularColor=[0.4, .4, 0],
                                          halfExtents=halfExtents)
        groundId = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=groundColId,
                                     baseVisualShapeIndex=groundVisID,
                                     basePosition=basePosition)
        return groundId

    def get_robot_points(self, config=None, end_point=True):
        points = []
        if config is not None:
            for i in range(p.getNumJoints(self.kukaId)):
                p.resetJointState(self.kukaId, i, config[i])
        if end_point:
            point = p.getLinkState(self.kukaId, self.kukaEndEffectorIndex)[0]
            point = (point[0], point[1], point[2])
            return point
        for effector in range(self.kukaEndEffectorIndex + 1):
            point = p.getLinkState(self.kukaId, effector)[0]
            point = (point[0], point[1], point[2])
            points.append(point)
        return points

    def sample_n_points(self, n=1):
        '''
        Uniformlly sample in the configuration space
        '''
        self.timer.start()
        sample = np.random.uniform(np.array(self.pose_range)[:, 0], np.array(self.pose_range)[:, 1], size=(n, self.config_dim))
        if n==1:
            self.timer.finish(Timer.SAMPLE)
            return sample.reshape(-1)
        else:
            self.timer.finish(Timer.SAMPLE)
            return sample

    def distance(self, from_state, to_state):
        '''
        Distance metric
        '''
        diff = abs(to_state - from_state)
        return np.sqrt(sum(diff ** 2))

    def set_config(self, c, kukaId=None):
        if kukaId is None:
            kukaId = self.kukaId
        for i in range(p.getNumJoints(kukaId)):
            p.resetJointState(kukaId, i, c[i])
        p.performCollisionDetection()

    def plot(self, path, make_gif=False):
        path = np.array(path)

        self.set_config(path[0])
        prev_pos = p.getLinkState(self.kukaId, self.kukaEndEffectorIndex)[0]

        target_kukaId = self.kukaId
        self.set_config(path[-1], target_kukaId)
        final_pos = p.getLinkState(target_kukaId, self.kukaEndEffectorIndex)[0]

        # for data in p.getVisualShapeData(new_kuka):  # change transparency
            #     color = list(data[-1])
            #     color[-1] = 0.5
            #     p.changeVisualShape(new_kuka, data[1], rgbaColor=color)

        gifs = []
        current_state_idx = 0

        while True:
            new_kuka = self.kukaId
            disp = path[current_state_idx + 1] - path[current_state_idx]
            d = self.distance(path[current_state_idx], path[current_state_idx + 1])
            K = int(np.ceil(d / 0.2))
            for k in range(0, K):
                c = path[current_state_idx] + k * 1. / K * disp
                self.set_config(c, new_kuka)
                new_pos = p.getLinkState(new_kuka, self.kukaEndEffectorIndex)[0]
                p.addUserDebugLine(prev_pos, new_pos, [1, 0, 0], 10, 0)
                prev_pos = new_pos
                p.loadURDF("sphere2red.urdf", new_pos, globalScaling=0.05, flags=p.URDF_IGNORE_COLLISION_SHAPES)
                if make_gif:
                    gifs.append(p.getCameraImage(width=1600, height=900, lightDirection=[0, 0, -1], shadow=0,
                                                 renderer=p.ER_BULLET_HARDWARE_OPENGL)[2])
            p.loadURDF("cube.urdf", prev_pos, globalScaling=0.05, flags=p.URDF_IGNORE_COLLISION_SHAPES) # visualize vertex

            current_state_idx += 1
            if current_state_idx == len(path) - 1:
                self.set_config(path[-1], new_kuka)
                p.addUserDebugLine(prev_pos, final_pos, [1, 0, 0], 10, 0)
                p.loadURDF("sphere2red.urdf", final_pos, globalScaling=0.05, flags=p.URDF_IGNORE_COLLISION_SHAPES)
                break

        return gifs

    def get_joint_angles(self):
        j = p.getJointStates(self.kukaId, [0,1,2,3,4,5,6])
        joints = [i[0] for i in j]
        return joints

    def joint_angles_control(self, joint_angles):
        p.setJointMotorControlArray(
            self.kukaId, [0,1,2,3,4,5,6],
            p.POSITION_CONTROL,
            targetPositions=joint_angles,
            targetVelocities=[0]*len(joint_angles),
            positionGains=[0.4]*len(joint_angles),
            # forces=forces
        )

    # =====================internal collision check module=======================

    def _point_in_free_space(self, state):
        self.collision_info = []
        t0 = time()

        for i in range(p.getNumJoints(self.kukaId)):
            p.resetJointState(self.kukaId, i, state[i])
        p.performCollisionDetection()
        collision_data = p.getContactPoints(bodyA=self.kukaId)
        if len(collision_data) == 0:
            return True
        else:
            return False

    def _state_fp(self, state):
        self.timer.start()
        free = self._point_in_free_space(state)
        self.timer.finish(Timer.VERTEX_CHECK)
        return free

    def _iterative_check_segment(self, left, right):
        if np.sum(np.abs(left - right)) > 0.1:
            mid = (left + right) / 2.0
            if not self._state_fp(mid):
                self.collision_point = mid
                return False
            return self._iterative_check_segment(left, mid) and self._iterative_check_segment(mid, right)
        return True

    def _edge_fp(self, state, new_state):
        self.timer.start()
        assert state.size == new_state.size
        '''
        if not self._point_in_free_space(state) or not self._point_in_free_space(new_state):
            self.timer.finish(Timer.EDGE_CHECK)
            return False
        '''
        disp = new_state - state
        d = self.distance(state, new_state)
        K = int(d / self.RRT_EPS)
        for k in range(1, K):
            c = state + k / K * disp
            if not self._point_in_free_space(c):
                self.timer.finish(Timer.EDGE_CHECK)
                return False
        self.timer.finish(Timer.EDGE_CHECK)
        return True

if __name__ == "__main__":
    sim = KukaEnv(GUI=True)
    initial = np.array(sim.get_joint_angles())
    goal = initial + np.array([0,1,-1,0.6,1.2,-0.2,-0.5])
    # initial = np.array([-0.48,-1.56,0.977,3.17,-1.14,0.24])
    # goal = np.array([-1.32, -0.69, 0.06, 2.8, -0.58, 0.9])
    for i in range(480):
        joints = initial + i * (goal - initial) / 479
        joint_angles = list(joints)
        sim.joint_angles_control(joint_angles)
        p.stepSimulation()
        sleep(1. / 240.)
    print(sim.get_joint_angles())
    sim.disconnect()
