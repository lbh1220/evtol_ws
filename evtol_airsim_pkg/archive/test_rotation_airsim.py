import airsim
import numpy as np
import tf.transformations as tf




client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
state = client.simGetGroundTruthKinematics()

print(state)

quaternion_cur = tf.quaternion_from_euler(0, 0, 0)

Rotation_martix = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])
R_gazebo = tf.quaternion_matrix(quaternion_cur)[:3, :3]
R_airsim = np.dot(Rotation_martix, R_gazebo)

# quaternion_A = tf.quaternion_from_euler(0, np.radians(90.0), 0)
quaternion_A = tf.quaternion_from_matrix(np.vstack([np.hstack([R_airsim, [[0], [0], [0]]]), [0, 0, 0, 1]]))
euler_airsim = tf.euler_from_quaternion(quaternion_A)
print("airsim euler: ", euler_airsim)
position_airsim = airsim.Vector3r(state.position.x_val, state.position.y_val, state.position.z_val)



quaternion_airsim = airsim.Quaternionr(quaternion_A[0],
                                    quaternion_A[1],
                                    quaternion_A[2],
                                    quaternion_A[3])     
client.simSetVehiclePose(airsim.Pose(position_airsim,quaternion_airsim),False)

state = client.simGetGroundTruthKinematics()

print(state)