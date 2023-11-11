import gtsam as gt
import numpy as np
import read_g2o
import matplotlib.pyplot as plt

counter = 0
input_file = "input_INTEL_g2o.g2o"
poses, edges = read_g2o.read_2D(input_file)

parameters = gt.ISAM2Params()
parameters.setRelinearizeThreshold(0.01)
parameters.relinearizeSkip = 1
isam = gt.ISAM2(parameters)

for pose in poses:
    graph = gt.NonlinearFactorGraph()
    initial = gt.Values()

    if pose[0] == 0:
        prior = gt.noiseModel.Diagonal.Variances(gt.Point3(0.3, 0.3, 0.1))
        graph.add(gt.PriorFactorPose2(0, gt.Pose2(pose[1], pose[2], pose[3]), prior))
        initial.insert(pose[0], gt.Pose2(pose[1], pose[2], pose[3]))
    else:
        initial.insert(pose[0], result.atPose2(counter - 1))

        for edge in edges:
            if edge[1] == pose[0]:
                info = edge[5:]
                omega = np.array([[info[0], info[1], info[2]],
                                [info[1], info[3], info[4]],
                                [info[2], info[4], info[5]]])    
                cov = np.linalg.inv(omega)
                noise = gt.noiseModel.Gaussian.Covariance(cov)
                graph.add(gt.BetweenFactorPose2(edge[0], edge[1], gt.Pose2(edge[2], edge[3], edge[4]), noise))
    
    counter += 1
    isam.update(graph, initial)
    result = isam.calculateEstimate()

resultPoses = gt.utilities.extractPose2(result)
poses = np.array(poses)

plt.plot(poses[:, 1], poses[:, 2])
plt.plot(resultPoses[:, 0], resultPoses[:, 1])
plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
plt.title('Incremental Solution')
plt.axis('equal')
plt.show()