import gtsam as gt
import numpy as np
import read_g2o
import matplotlib.pyplot as plt

input_file = "input_INTEL_g2o.g2o"
poses, edges = read_g2o.read_2D(input_file)
    
graph = gt.NonlinearFactorGraph()
initial = gt.Values()

for pose in poses:
    if pose[0] == 0:
        prior = gt.noiseModel.Diagonal.Variances(gt.Point3(0.3, 0.3, 0.1))
        graph.add(gt.PriorFactorPose2(0, gt.Pose2(pose[1], pose[2], pose[3]), prior))
    initial.insert(pose[0], gt.Pose2(pose[1], pose[2], pose[3]))

for edge in edges:
    info = edge[5:]
    omega = np.array([[info[0], info[1], info[2]],
                      [info[1], info[3], info[4]],
                      [info[2], info[4], info[5]]])    
    cov = np.linalg.inv(omega)
    noise = gt.noiseModel.Gaussian.Covariance(cov)
    graph.add(gt.BetweenFactorPose2(edge[0], edge[1], gt.Pose2(edge[2], edge[3], edge[4]), noise))

parameters = gt.GaussNewtonParams()
parameters.setRelativeErrorTol(1e-5)
parameters.setMaxIterations(100)
    
optimizer = gt.GaussNewtonOptimizer(graph, initial, parameters)
result = optimizer.optimize()
resultPoses = gt.utilities.extractPose2(result)

poses = np.array(poses)

print(poses[3], resultPoses[3])

plt.plot(poses[:, 1], poses[:, 2])
plt.plot(resultPoses[:, 0], resultPoses[:, 1])
plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
plt.title('Batch Solution')
plt.axis('equal')
plt.show()