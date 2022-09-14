import open3d as o3d
import json
import numpy as np
import pandas as pd

# loading data

pcds_path = 'data/pcd/'

extrinsics = json.load(open('transforms.json'))

intrinsics = json.load(open('transforms.json'))

quarternions = open("images.txt").readlines()
df = pd.DataFrame(columns=['image_id','qw', 'qx', 'qy', 'qz', 'tx', 'ty', 'tz', 'camera_id', 'name'])
line_counter = 1
for quat_line in quarternions:
    if not quat_line.startswith("#"):
        if not line_counter % 2 == 0:
            image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, name = quat_line.rstrip().split(" ")

            df = df.append({'image_id': image_id,
                             'qw': qw,
                             'qx': qx,
                             'qy': qy,
                             'qz': qz,
                             'tx': tx,
                             'ty': ty,
                             'tz': tz,
                             'camera_id': camera_id,
                             'name': name}, ignore_index=True)
        line_counter += 1


ignore_pictures = ['data_18', 'data_19']


import matplotlib.pyplot as plt
import numpy as np

from pytransform3d.rotations import check_quaternion, matrix_from_quaternion, euler_xyz_from_matrix
from pytransform3d.transformations import transform_from

import math

from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R


# visualise camera orientations
def visualise_cameras():
    ax = plt.figure().add_subplot(projection='3d')

    for index, row in df.iterrows():

        quaternion = Quaternion(w=row['qw'], x=row['qx'], y=row['qy'], z=row['qz'])
        quaternion = quaternion.inverse
        quaternion = check_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]], unit=True)

        rotation_matrix = matrix_from_quaternion(quaternion)

        euler = euler_xyz_from_matrix(rotation_matrix) #z==yaw

        yaw = euler[2]
        new_x = math.sin(yaw)
        new_y = math.cos(yaw)

        ax.quiver(float(row['tx']), float(row['ty']), float(row['tz']), new_x, new_y, 0, color='b')
        ax.text(float(row['tx']), float(row['ty']), float(row['tz']), row['name'], bbox=dict(facecolor='red', alpha=0.5))

    plt.show()


# visualise_cameras()


# transform point clouds and visualise
pcd_combined = o3d.geometry.PointCloud()

for index, row in df.iterrows():
    if not any(item == row['name'].split('.')[0] for item in ignore_pictures):

        filename = row['name'].split('.')[0]

        print (filename)

        pcd = o3d.io.read_point_cloud(pcds_path + '/' + filename + '.ply')

        quaternion = Quaternion(w=row['qw'], x=row['qx'], y=row['qy'], z=row['qz'])
        quaternion = quaternion.inverse

        rotation = quaternion.rotation_matrix

        rotation_ = R.from_matrix(rotation)
        euler = rotation_.as_euler('xyz')

        translation = np.array([float(row['tx']), float(row['ty']), float(row['tz'])])

        transformation = transform_from(rotation, translation)

        pcd.transform(transformation)

        pcd_combined += pcd


o3d.visualization.draw_geometries([pcd_combined])