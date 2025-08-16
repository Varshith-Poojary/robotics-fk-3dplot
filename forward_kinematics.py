import numpy as np
import matplotlib.pyplot as plt

def fw_kinematics(angles,L):
    #transformation of frame1 wrt to frame0
    T01 = np.array([
    [np.cos(angles[0]), 0, -np.sin(angles[0]), L * np.cos(angles[0])],
    [np.sin(angles[0]), 0,  np.cos(angles[0]), L * np.sin(angles[0])],
    [0,             -1,  0,              0],
    [0,              0,  0,              1]])
    
    #transformation of frame2 wrt to frame1
    T12 = np.array([
    [np.cos(angles[1]), 0, np.sin(angles[1]), L * np.cos(angles[1])],
    [np.sin(angles[1]), 0,  -np.cos(angles[1]), L * np.sin(angles[1])],
    [0,             1,  0,              0],
    [0,              0,  0,              1]])
    
    #transformation of frame3 wrt to frame2
    T23 = np.array([
    [np.cos(angles[2]), 0, -np.sin(angles[2]), L * np.cos(angles[2])],
    [np.sin(angles[2]), 0,  np.cos(angles[2]), L * np.sin(angles[2])],
    [0,             -1,  0,              0],
    [0,              0,  0,              1]])
    
    #transformation of frame4 wrt to frame3
    T34 = np.array([
    [np.cos(angles[3]),  -np.sin(angles[3]), 0, L * np.cos(angles[3])],
    [np.sin(angles[3]),  np.cos(angles[3]), 0, L * np.sin(angles[3])],
    [0,             0,  1,              0],
    [0,              0,  0,              1]])
    
    #transformation of end effector wrt to base
    T04= T01 @ T12 @ T23 @ T34
    
    # Cumulative transformations
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    
    #joint positions
    points = np.array([
        [0, 0, 0],           # Base
        T01[0:3, 3],         # Joint 1
        T02[0:3, 3],         # Joint 2
        T03[0:3, 3],         # Joint 3
        T04[0:3, 3]          # End effector
    ])

    return T04, points


angles_deg=[int(input(f"Enter the angle of joint {i} in deg: ")) for i in range(0,4)]
angles=[np.deg2rad(j) for j in angles_deg]
L = float(input("Length of the links: "))

T04, points = fw_kinematics(angles, L)
print("\nFinal Transformation Matrix T04:\n", T04)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the arm as a line
ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', markersize=6, color='black', linewidth=2)

# Highlight base (red) and end effector (blue)
ax.scatter(points[0, 0], points[0, 1], points[0, 2], color='red', s=80, label='Base')
ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color='blue', s=80, label='End Effector')

# Labels and styling
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Robot Arm in 3D")
ax.set_box_aspect([1, 1, 1])
ax.legend()

plt.show()