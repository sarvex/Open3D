# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------

import open3d as o3d
import numpy as np


def f_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = sum(1 for child in node.children if child is not None)
            print(
                f"{'    ' * node_info.depth}{node_info.child_index}: Internal node at depth {node_info.depth} has {n} children and {len(node.indices)} points ({node_info.origin})"
            )

            # We only want to process nodes / spatial regions with enough points.
            early_stop = len(node.indices) < 250
    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            print(
                f"{'    ' * node_info.depth}{node_info.child_index}: Leaf node at depth {node_info.depth} has {len(node.indices)} points with origin {node_info.origin}"
            )
    else:
        raise NotImplementedError('Node type not recognized!')

    # Early stopping: if True, traversal of children of the current node will be skipped.
    return early_stop


if __name__ == "__main__":
    N = 2000
    armadillo_data = o3d.data.ArmadilloMesh()
    pcd = o3d.io.read_triangle_mesh(
        armadillo_data.path).sample_points_poisson_disk(N)
    # Fit to unit cube.
    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
              center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1,
                                                              size=(N, 3)))

    octree = o3d.geometry.Octree(max_depth=4)
    octree.convert_from_point_cloud(pcd, size_expand=0.01)
    print('Displaying input octree ...')
    o3d.visualization.draw([octree])
    print('Traversing octree ...')
    octree.traverse(f_traverse)
