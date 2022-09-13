import open3d as o3d
import json
import numpy as np
import pandas as pd

# loading data

pcds_path = 'data/pcd/'

extrinsics = json.load(open('transforms.json'))

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


### 


wanted_pictures = ['data_4', 'data_33']

from scipy.linalg import polar
import math
from scipy.spatial.transform import Rotation as R
import copy

# trying with euler angles

pcd_combined = o3d.geometry.PointCloud()

for idx, frame in enumerate(extrinsics['frames']):
    if frame['file_path'].split('/')[-1].split('.')[0] in wanted_pictures:
        filename = frame['file_path'].split('/')[-1].split('.')[0]
        transformation = np.array(frame['transform_matrix'])
        print (filename)
        print (transformation)

        pcd = o3d.io.read_point_cloud(pcds_path + '/' + filename + '.ply')

        r = R.from_matrix(transformation[0:3, 0:3])

        deg = r.as_euler('xyz')
        
        R_ = pcd.get_rotation_matrix_from_xyz((-deg[0], -deg[1], -deg[2]))

        pcd.rotate(R_, center=(0, 0, 0))
        
        pcd_combined += pcd


o3d.visualization.draw_geometries([pcd_combined])

# trying with waternions

pcd_combined = o3d.geometry.PointCloud()

for index, row in df.iterrows():
    if row['name'].split('.')[0] in wanted_pictures:

        filename = row['name'].split('.')[0]

        pcd = o3d.io.read_point_cloud(pcds_path + '/' + filename + '.ply')

        #quaternion in scalar-last (x, y, z, w) format
        r = R.from_quat([row['qx'], row['qy'], row['qz'], row['qw']])

        deg = r.as_euler('xyz')

        R_ = pcd.get_rotation_matrix_from_xyz((-deg[0], -deg[1], -deg[2]))

        pcd.rotate(R_, center=(0, 0, 0))

        pcd_combined += pcd


o3d.visualization.draw_geometries([pcd_combined])