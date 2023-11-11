import gtsam as gt
import numpy as np
import read_g2o
import matplotlib.pyplot as plt

input_file = "parking-garage.g2o"
poses, edges = read_g2o.read_3D(input_file)

graph = gt.NonlinearFactorGraph()
initial = gt.Values()

for pose in poses:
    r = gt.Rot3.Quaternion(pose[7], pose[4], pose[5], pose[6])
    t = gt.Point3(pose[1], pose[2], pose[3])
        
    if pose[0] == 0:
        prior = gt.noiseModel.Diagonal.Variances(np.array([0.3, 0.3, 0.3, 0.1, 0.1, 0.1]))        
        graph.add(gt.PriorFactorPose3(0, gt.Pose3(r, t), prior))
    
    initial.insert(pose[0], gt.Pose3(r, t))
    
for edge in edges:
    info = edge[9:]
    omega = np.array([[info[0], info[1], info[2], info[3], info[4], info[5]],
                      [info[1], info[6], info[7], info[8], info[9], info[10]],
                      [info[2], info[7], info[11], info[12], info[13], info[14]],
                      [info[3], info[8], info[12], info[15], info[16], info[17]],
                      [info[4], info[9], info[13], info[16], info[18], info[19]],
                      [info[5], info[10], info[14], info[17], info[19], info[20]]])   
    
    r = gt.Rot3.Quaternion(edge[8], edge[5], edge[6], edge[7])
    t = gt.Point3(edge[2], edge[3], edge[4])
     
    cov = np.linalg.inv(omega)
    noise = gt.noiseModel.Gaussian.Covariance(cov)
    graph.add(gt.BetweenFactorPose3(edge[0], edge[1], gt.Pose3(r, t), noise))

parameters = gt.GaussNewtonParams()
parameters.setRelativeErrorTol(1e-5)
parameters.setMaxIterations(100)
    
optimizer = gt.GaussNewtonOptimizer(graph, initial, parameters)
result = optimizer.optimize()
resultPoses = gt.utilities.extractPose3(result)

poses = np.array(poses)

ax = plt.axes(projection = '3d')
plt.plot(poses[:, 1], poses[:, 2], poses[:, 3])
plt.plot(resultPoses[:, 9], resultPoses[:, 10], resultPoses[:, 11])
plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
plt.title('Batch Solution 3D')
plt.axis('equal')
plt.show()