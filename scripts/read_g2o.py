import gtsam as gt
import numpy as np

def read_2D(file_name):
    file = open(file_name, "r")

    poses = []
    edges = []
    
    for f in file:
        f = f.split()
        if f[0] == "VERTEX_SE2":
            pose = [int(f[1]), float(f[2]), float(f[3]), float(f[4])]
            poses.append(pose)
        elif f[0] == "EDGE_SE2":
            edge = [int(f[1]), int(f[2]), float(f[3]), float(f[4]), float(f[5]), float(f[6]), float(f[7]), float(f[8]), float(f[9]), float(f[10]), float(f[11])]
            edges.append(edge)

    return poses, edges


def read_3D(file_name):
    file = open(file_name, "r")

    poses = []
    edges = []
    
    for f in file:
        f = f.split()
        if f[0] == "VERTEX_SE3:QUAT":
            pose = [int(f[1]), float(f[2]), float(f[3]), float(f[4]), float(f[5]), float(f[6]), float(f[7]), float(f[8])]
            poses.append(pose)
        elif f[0] == "EDGE_SE3:QUAT":
            edge = [int(f[1]), int(f[2]), float(f[3]), float(f[4]), float(f[5]), float(f[6]), float(f[7]), float(f[8]), float(f[9]), float(f[10]), float(f[11]),
                    float(f[12]),float(f[13]),float(f[14]),float(f[15]),float(f[16]),float(f[17]),float(f[18]),float(f[19]),float(f[20]), float(f[21]),float(f[22]),
                    float(f[23]),float(f[24]),float(f[25]),float(f[26]),float(f[27]),float(f[28]),float(f[29]),float(f[30])]
            edges.append(edge)

    return poses, edges
