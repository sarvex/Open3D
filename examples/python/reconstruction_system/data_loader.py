# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------

import open3d as o3d


def lounge_data_loader():
    print('Loading Stanford Lounge RGB-D Dataset')

    # Get the dataset.
    lounge_rgbd = o3d.data.LoungeRGBDImages()

    return {
        'path_dataset': lounge_rgbd.extract_dir,
        'path_intrinsic': "",
        'depth_max': 3.0,
        'voxel_size': 0.05,
        'depth_diff_max': 0.07,
        'preference_loop_closure_odometry': 0.1,
        'preference_loop_closure_registration': 5.0,
        'tsdf_cubic_size': 3.0,
        'icp_method': "color",
        'global_registration': "ransac",
        'python_multi_threading': True,
    }


def bedroom_data_loader():
    print('Loading Redwood Bedroom RGB-D Dataset')

    # Get the dataset.
    bedroom_rgbd = o3d.data.BedroomRGBDImages()

    return {
        'path_dataset': bedroom_rgbd.extract_dir,
        'path_intrinsic': "",
        'depth_max': 3.0,
        'voxel_size': 0.05,
        'depth_diff_max': 0.07,
        'preference_loop_closure_odometry': 0.1,
        'preference_loop_closure_registration': 5.0,
        'tsdf_cubic_size': 3.0,
        'icp_method': "color",
        'global_registration': "ransac",
        'python_multi_threading': True,
    }


def jackjack_data_loader():
    print('Loading RealSense L515 Jack-Jack RGB-D Bag Dataset')

    # Get the dataset.
    jackjack_bag = o3d.data.JackJackL515Bag()

    return {
        'path_dataset': jackjack_bag.path,
        'path_intrinsic': "",
        'depth_max': 0.85,
        'voxel_size': 0.025,
        'depth_diff_max': 0.03,
        'preference_loop_closure_odometry': 0.1,
        'preference_loop_closure_registration': 5.0,
        'tsdf_cubic_size': 0.75,
        'icp_method': "color",
        'global_registration': "ransac",
        'python_multi_threading': True,
    }
