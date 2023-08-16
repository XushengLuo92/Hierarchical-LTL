import numpy as np
import open3d as o3d


DH = np.matrix([[0, 0, 0.05, -np.pi / 2],
               [-np.pi / 2, 0, 0.44, np.pi],
               [0, 0, 0.035, -np.pi / 2],
               [0, -0.42, 0, np.pi / 2],
               [0, 0, 0, -np.pi / 2]])

ee_DH = [0, -0.08, 0, np.pi]
tool_DH = [0, -0.1815, 0, np.pi]
tool_assemble_DH = [0, -0.1815, 0.0078, np.pi]
tool_disassemble_DH = [0.0, -0.191, 0, np.pi]

# 0.0112
# print(DH)
theta = [8.043, 82.156, 49.497, -0.008, -57.382, -8.022]
print(theta)
robot_base = [0, 0, 0.33]
for i in range(DH.shape[0]):
    theta[i] = theta[i] / 180 * np.pi
    DH[i, 0] = DH[i, 0] + theta[i]
ee_DH[0] += theta[-1]
tool_DH[0] += theta[-1]
tool_assemble_DH[0] += theta[-1]
tool_disassemble_DH[0] += theta[-1]

origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
j1_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
j2_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
j3_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
j4_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
j5_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
j6_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
ee_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
ee_tool_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
ee_assemble_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02, origin=[0, 0, 0])
ee_disassemble_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02, origin=[0, 0, 0])
frame_list = [origin_frame, base_frame, j1_frame, j2_frame, j3_frame, j4_frame, j5_frame, j6_frame, ee_frame, ee_tool_frame, ee_assemble_frame, ee_disassemble_frame]

T = np.identity(4)
T[0, 3] = robot_base[0]
T[1, 3] = robot_base[1]
T[2, 3] = robot_base[2]
frame_list[1].transform(T)

for i in range(DH.shape[0]):
    R = np.matrix([[np.cos(DH[i, 0]), -np.sin(DH[i, 0]) * np.cos(DH[i, 3]), np.sin(DH[i, 0]) * np.sin(DH[i, 3])],
                   [np.sin(DH[i, 0]), np.cos(DH[i, 0]) * np.cos(DH[i, 3]), -np.cos(DH[i, 0]) * np.sin(DH[i, 3])],
                   [0, np.sin(DH[i, 3]), np.cos(DH[i, 3])]])
    t = np.matrix([[DH[i, 2] * np.cos(DH[i, 0])],
                   [DH[i, 2] * np.sin(DH[i, 0])],
                   [DH[i, 1]]]).reshape(3, )
    
    mtx4 = np.identity(4)
    mtx4[:3, :3] = R
    mtx4[:3, 3] = t
    T = np.matmul(T, mtx4)
    
    frame_list[i + 2].transform(T)



R = np.matrix([[np.cos(ee_DH[0]), -np.sin(ee_DH[0]) * np.cos(ee_DH[3]), np.sin(ee_DH[0]) * np.sin(ee_DH[3])],
                   [np.sin(ee_DH[0]), np.cos(ee_DH[0]) * np.cos(ee_DH[3]), -np.cos(ee_DH[0]) * np.sin(ee_DH[3])],
                   [0, np.sin(ee_DH[3]), np.cos(ee_DH[3])]])
t = np.matrix([[ee_DH[2] * np.cos(ee_DH[0])],
                   [ee_DH[2] * np.sin(ee_DH[0])],
                   [ee_DH[1]]]).reshape(3, )
    
mtx4 = np.identity(4)
mtx4[:3, :3] = R
mtx4[:3, 3] = t
trans = np.matmul(T, mtx4)
frame_list[8].transform(trans)


R = np.matrix([[np.cos(tool_DH[0]), -np.sin(tool_DH[0]) * np.cos(tool_DH[3]), np.sin(tool_DH[0]) * np.sin(tool_DH[3])],
                   [np.sin(tool_DH[0]), np.cos(tool_DH[0]) * np.cos(tool_DH[3]), -np.cos(tool_DH[0]) * np.sin(tool_DH[3])],
                   [0, np.sin(tool_DH[3]), np.cos(tool_DH[3])]])
t = np.matrix([[tool_DH[2] * np.cos(tool_DH[0])],
                   [tool_DH[2] * np.sin(tool_DH[0])],
                   [tool_DH[1]]]).reshape(3, )
    
mtx4 = np.identity(4)
mtx4[:3, :3] = R
mtx4[:3, 3] = t
trans = np.matmul(T, mtx4)
print(trans[:3, 3])
frame_list[9].transform(trans)

R = np.matrix([[np.cos(tool_assemble_DH[0]), -np.sin(tool_assemble_DH[0]) * np.cos(tool_assemble_DH[3]), np.sin(tool_assemble_DH[0]) * np.sin(tool_assemble_DH[3])],
                   [np.sin(tool_assemble_DH[0]), np.cos(tool_assemble_DH[0]) * np.cos(tool_assemble_DH[3]), -np.cos(tool_assemble_DH[0]) * np.sin(tool_assemble_DH[3])],
                   [0, np.sin(tool_assemble_DH[3]), np.cos(tool_assemble_DH[3])]])
t = np.matrix([[tool_assemble_DH[2] * np.cos(tool_assemble_DH[0])],
                   [tool_assemble_DH[2] * np.sin(tool_assemble_DH[0])],
                   [tool_assemble_DH[1]]]).reshape(3, )
    
mtx4 = np.identity(4)
mtx4[:3, :3] = R
mtx4[:3, 3] = t
trans = np.matmul(T, mtx4)
frame_list[10].transform(trans)

R = np.matrix([[np.cos(tool_disassemble_DH[0]), -np.sin(tool_disassemble_DH[0]) * np.cos(tool_disassemble_DH[3]), np.sin(tool_disassemble_DH[0]) * np.sin(tool_disassemble_DH[3])],
                   [np.sin(tool_disassemble_DH[0]), np.cos(tool_disassemble_DH[0]) * np.cos(tool_disassemble_DH[3]), -np.cos(tool_disassemble_DH[0]) * np.sin(tool_disassemble_DH[3])],
                   [0, np.sin(tool_disassemble_DH[3]), np.cos(tool_disassemble_DH[3])]])
t = np.matrix([[tool_disassemble_DH[2] * np.cos(tool_disassemble_DH[0])],
                   [tool_disassemble_DH[2] * np.sin(tool_disassemble_DH[0])],
                   [tool_disassemble_DH[1]]]).reshape(3, )
    
mtx4 = np.identity(4)
mtx4[:3, :3] = R
mtx4[:3, 3] = t
trans = np.matmul(T, mtx4)
frame_list[11].transform(trans)

o3d.visualization.draw_geometries(frame_list)