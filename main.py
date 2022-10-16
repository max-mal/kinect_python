#!/usr/bin/env python
"""Sweeps throught the depth image showing 100 range at a time"""
import freenect
import cv2
import numpy as np
import time
import open3d as o3d
import frame_convert2
import math
import time


def get_kinect_data():
    depth, _t = freenect.sync_get_depth()
    color, _t = freenect.sync_get_video()
    return (color, depth)


def create_rgbd_image(color, depth):
    depth_image = o3d.geometry.Image((depth).astype(np.uint8))
    color_image = o3d.geometry.Image((color).astype(np.uint8))

    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, 
        depth_image, 
        convert_rgb_to_intensity = False
    )


def _get_camera_intrinsic():
    return o3d.camera.PinholeCameraIntrinsic(640, 480, 594.21, 591.04, 339.5, 242.7)


def uvd_to_xyz_a(x, y, d):
    cx, cy = (339.5, 242.7)
    cz = 0
    fl = 1200

    _x = cx + (x - cx) * d / fl
    _y = cy + (y - cy) * d / fl
    _z = cz + d

    return (_x, _y, float(_z))


def depth_to_meters(depth):
    return 1.0 / (depth * -0.0030711016 + 3.3309495161)


def uvd_to_metric(x, y, d):
    cx_d = 3.3930780975300314e+02
    cy_d = 2.4273913761751615e+02

    fx_d = 5.9421434211923247e+02
    fy_d = 5.9104053696870778e+02

    depth = depth_to_meters(d)

    x = (x - cx_d) * depth / fx_d
    y = (y - cy_d) * depth / fy_d
    z = depth

    return (x, y, z)


def depths_to_gamma_a(depth):

    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)

    return depth


def depths_to_gamma_b(depths):
    values = []
    for i in range(2048):
        k1 = 1.1863
        k2 = 2842.5
        k3 = 0.1236
        depth = k3 * math.tan(i / k2 + k1)
        values.append(depth)

    for x in range(depths.shape[0]):
        for y in range(depths.shape[1]):
            depths[x][y] = values[depths[x][y]]

    return depths


def colors_points_from_data(depths, colors):
    depth_vertices = []
    colors_verticles = []
    for x in range(depths.shape[0]):
        for y in range(depths.shape[1]):
            if depths[x][y] >= 2047:
                continue
            # _x, _y, _z = uvd_to_xyz_a(x, y, depths[x][y])
            _x, _y, _z = uvd_to_metric(x, y, depths[x][y])
            depth_vertices.append([_y, _x, _z])
            colors_verticles.append(
                np.array(colors[x][y]).astype(float) / 255.0
            )

    points = np.asarray(np.array(depth_vertices))
    points_vector = o3d.utility.Vector3dVector(points)

    colors_array = np.asarray(np.array(colors_verticles))
    colors_vector = o3d.utility.Vector3dVector(colors_array)

    return (points_vector, colors_vector)


def pcd_from_data(depths, colors):
    points, colors = colors_points_from_data(depths, colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = points
    pcd.colors = colors

    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    return pcd


def pcd_from_rgbd(rgbd, depths=None):
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, 
        _get_camera_intrinsic()
    )   
    # flip the orientation, so it looks upright, not upside-down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    return pcd


def generate_mesh_bpa(pcd, r=2):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = r * avg_dist

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector([radius, radius * 2])
    )

    return mesh


def generate_mesh_poisson(pcd):
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, 
        depth=12,
        # width=0,
        # scale=1.1,
        # linear_fit=False
    )[0]

    return poisson_mesh


def print_ok():
    print("[OK]")


def set_tilt(tilt):
    ctx = freenect.init()
    dev = freenect.open_device(ctx, 0)
    freenect.set_tilt_degs(dev, tilt)
    freenect.close_device(dev)
    time.sleep(2)


def main():

    tilt = 30
    set_tilt(tilt)

    from_data = True
    print(" - Getting image from kinect")
    kinect_color, kinect_depth = get_kinect_data()

    np.save("color.npy", kinect_color)
    np.save("depth.npy", kinect_depth)

    print_ok()

    print(" - Creating PointCloud")
    if from_data:
        pcd = pcd_from_data(kinect_depth, kinect_color)
    else:
        pcd = pcd_from_rgbd(
            create_rgbd_image(
                kinect_color,
                depths_to_gamma_a(kinect_depth),
            )
        )
    print_ok()

    # print(" - Estimating pcd normals")
    # pcd.estimate_normals()
    # pcd.normalize_normals()
    # print_ok()

    if tilt > 0:
        rotation = (tilt * np.pi / 180)
        R = pcd.get_rotation_matrix_from_xyz((rotation, 0, 0))
        pcd = pcd.rotate(R)
        pcd.translate((0, 1.45, 0))

    freenect.sync_stop()
    set_tilt(0)

    _kinect_color, _kinect_depth = get_kinect_data()
    _pcd = pcd_from_data(_kinect_depth, _kinect_color)

    o3d.visualization.draw_geometries([pcd, _pcd])

    # downpcd = pcd.voxel_down_sample(voxel_size=3)
    # downpcd.estimate_normals()
    # downpcd.normalize_normals()

    # bb_center = [0, -470, -700]

    # R = np.identity(3)
    # extent = np.array([600, 370, 600]) # trying to create a bounding box below 1 unit
    # center = np.array(bb_center)
    # obb = o3d.geometry.OrientedBoundingBox(center,R,extent)
    # o3d.visualization.draw_geometries([pcd, mesh_box])

    # _pcd = pcd.crop(obb)
    # o3d.visualization.draw_geometries([pcd])

    # print(" - Creating poisson mesh")
    # poisson_mesh = generate_mesh_poisson(pcd)
    # print_ok()

    # o3d.visualization.draw_geometries([poisson_mesh])

    # print(" - Creating bpa mesh")
    # bpa_mesh = generate_mesh_bpa(pcd)
    # print_ok()

    # o3d.visualization.draw_geometries([bpa_mesh])

    # input()


main()
freenect.sync_stop()
