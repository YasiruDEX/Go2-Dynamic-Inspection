#!/usr/bin/env python3
"""
Visualize a .vgh (Visibility Graph) file from FAR Planner.

Binary format (per far_planner.cpp SaveVisibilityGraph):
  - size_t (8B): number of nodes
  For each node:
    - size_t (8B): node ID
    - 3 x float (4B each): x, y, z position
    - 4 x bool (1B each): is_covered, is_frontier, is_navpoint, is_boundary
    - int (4B): free_direct
    - size_t: connect_size, then connect_size x size_t (connection indices)
    - size_t: poly_size, then poly_size x size_t (polygon connection indices)
    - size_t: contour_size, then contour_size x size_t (contour connection indices)
"""

import struct
import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def parse_vgh(filename):
    """Parse a .vgh binary file and return nodes and edges."""
    nodes = []
    connect_edges = []   # navigation edges
    poly_edges = []      # polygon edges
    contour_edges = []   # contour edges

    with open(filename, 'rb') as f:
        # Read graph size
        data = f.read(8)
        graph_size = struct.unpack('<Q', data)[0]
        print(f"Graph contains {graph_size} nodes")

        # First pass: read all nodes and their connection indices
        node_connections = []
        for i in range(graph_size):
            # Node ID (size_t = 8 bytes)
            node_id = struct.unpack('<Q', f.read(8))[0]

            # Position (3 x float = 12 bytes)
            x, y, z = struct.unpack('<3f', f.read(12))

            # Properties (4 x bool + 1 x int = 4 + 4 = 8 bytes)
            is_covered = struct.unpack('<?', f.read(1))[0]
            is_frontier = struct.unpack('<?', f.read(1))[0]
            is_navpoint = struct.unpack('<?', f.read(1))[0]
            is_boundary = struct.unpack('<?', f.read(1))[0]
            free_direct = struct.unpack('<i', f.read(4))[0]

            # Connection indices
            connect_size = struct.unpack('<Q', f.read(8))[0]
            connect_indices = []
            for _ in range(connect_size):
                idx = struct.unpack('<Q', f.read(8))[0]
                connect_indices.append(idx)

            # Polygon connection indices
            poly_size = struct.unpack('<Q', f.read(8))[0]
            poly_indices = []
            for _ in range(poly_size):
                idx = struct.unpack('<Q', f.read(8))[0]
                poly_indices.append(idx)

            # Contour connection indices
            contour_size = struct.unpack('<Q', f.read(8))[0]
            contour_indices = []
            for _ in range(contour_size):
                idx = struct.unpack('<Q', f.read(8))[0]
                contour_indices.append(idx)

            nodes.append({
                'id': node_id,
                'x': x, 'y': y, 'z': z,
                'is_covered': is_covered,
                'is_frontier': is_frontier,
                'is_navpoint': is_navpoint,
                'is_boundary': is_boundary,
                'free_direct': free_direct,
            })
            node_connections.append({
                'connect': connect_indices,
                'poly': poly_indices,
                'contour': contour_indices,
            })

    # Build edge lists (avoid duplicates by only adding i->j where i < j)
    for i, conn in enumerate(node_connections):
        for j in conn['connect']:
            if j < graph_size and i < j:
                connect_edges.append((i, j))
        for j in conn['poly']:
            if j < graph_size and i < j:
                poly_edges.append((i, j))
        for j in conn['contour']:
            if j < graph_size and i < j:
                contour_edges.append((i, j))

    return nodes, connect_edges, poly_edges, contour_edges


def print_stats(nodes, connect_edges, poly_edges, contour_edges):
    """Print graph statistics."""
    xs = [n['x'] for n in nodes]
    ys = [n['y'] for n in nodes]
    zs = [n['z'] for n in nodes]

    print(f"\n=== Visibility Graph Statistics ===")
    print(f"  Nodes:             {len(nodes)}")
    print(f"  Navigation edges:  {len(connect_edges)}")
    print(f"  Polygon edges:     {len(poly_edges)}")
    print(f"  Contour edges:     {len(contour_edges)}")
    print(f"  Frontier nodes:    {sum(1 for n in nodes if n['is_frontier'])}")
    print(f"  Boundary nodes:    {sum(1 for n in nodes if n['is_boundary'])}")
    print(f"  Navpoint nodes:    {sum(1 for n in nodes if n['is_navpoint'])}")
    print(f"  Covered nodes:     {sum(1 for n in nodes if n['is_covered'])}")
    print(f"  X range: [{min(xs):.2f}, {max(xs):.2f}]")
    print(f"  Y range: [{min(ys):.2f}, {max(ys):.2f}]")
    print(f"  Z range: [{min(zs):.2f}, {max(zs):.2f}]")
    print()


def visualize_2d(nodes, connect_edges, poly_edges, contour_edges, title="Visibility Graph"):
    """Create a 2D top-down visualization (X-Y plane)."""
    xs = np.array([n['x'] for n in nodes])
    ys = np.array([n['y'] for n in nodes])

    fig, axes = plt.subplots(1, 2, figsize=(20, 10))
    fig.suptitle(title, fontsize=16)

    # --- Left plot: Full graph with all edge types ---
    ax = axes[0]
    ax.set_title("Full Graph (Top-Down View)")

    # Draw contour edges (green, thin)
    for i, j in contour_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]], 'g-', alpha=0.3, linewidth=0.5)

    # Draw polygon edges (blue, thin)
    for i, j in poly_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]], 'b-', alpha=0.3, linewidth=0.5)

    # Draw navigation edges (gray, thin)
    for i, j in connect_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]], color='gray', alpha=0.2, linewidth=0.3)

    # Draw nodes colored by type
    regular = [i for i, n in enumerate(nodes) if not n['is_frontier'] and not n['is_boundary'] and not n['is_navpoint']]
    frontiers = [i for i, n in enumerate(nodes) if n['is_frontier']]
    boundaries = [i for i, n in enumerate(nodes) if n['is_boundary']]
    navpoints = [i for i, n in enumerate(nodes) if n['is_navpoint']]

    if regular:
        ax.scatter(xs[regular], ys[regular], c='black', s=3, alpha=0.5, label=f'Regular ({len(regular)})', zorder=5)
    if frontiers:
        ax.scatter(xs[frontiers], ys[frontiers], c='red', s=15, alpha=0.8, label=f'Frontier ({len(frontiers)})', zorder=6)
    if boundaries:
        ax.scatter(xs[boundaries], ys[boundaries], c='blue', s=10, alpha=0.8, label=f'Boundary ({len(boundaries)})', zorder=6)
    if navpoints:
        ax.scatter(xs[navpoints], ys[navpoints], c='orange', s=12, alpha=0.8, label=f'Navpoint ({len(navpoints)})', zorder=6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Right plot: Nodes colored by Z height ---
    ax2 = axes[1]
    ax2.set_title("Nodes Colored by Height (Z)")
    zs = np.array([n['z'] for n in nodes])
    scatter = ax2.scatter(xs, ys, c=zs, cmap='viridis', s=5, alpha=0.7)
    plt.colorbar(scatter, ax=ax2, label='Z height (m)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/home/tharushi/Go2-Dynamic-Inspection/university2_2d.png', dpi=150, bbox_inches='tight')
    print("Saved 2D visualization to university2_2d.png")
    plt.show()


def visualize_3d(nodes, connect_edges, poly_edges, contour_edges, title="Visibility Graph 3D"):
    """Create a 3D visualization."""
    xs = np.array([n['x'] for n in nodes])
    ys = np.array([n['y'] for n in nodes])
    zs = np.array([n['z'] for n in nodes])

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)

    # Draw navigation edges
    for i, j in connect_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]], [zs[i], zs[j]], color='gray', alpha=0.15, linewidth=0.3)

    # Draw polygon edges
    for i, j in poly_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]], [zs[i], zs[j]], 'b-', alpha=0.3, linewidth=0.5)

    # Draw contour edges
    for i, j in contour_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]], [zs[i], zs[j]], 'g-', alpha=0.3, linewidth=0.5)

    # Nodes
    frontiers = [i for i, n in enumerate(nodes) if n['is_frontier']]
    boundaries = [i for i, n in enumerate(nodes) if n['is_boundary']]
    others = [i for i, n in enumerate(nodes) if not n['is_frontier'] and not n['is_boundary']]

    if others:
        ax.scatter(xs[others], ys[others], zs[others], c='black', s=2, alpha=0.3, label='Regular')
    if frontiers:
        ax.scatter(xs[frontiers], ys[frontiers], zs[frontiers], c='red', s=15, alpha=0.8, label='Frontier')
    if boundaries:
        ax.scatter(xs[boundaries], ys[boundaries], zs[boundaries], c='blue', s=10, alpha=0.8, label='Boundary')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig('/home/tharushi/Go2-Dynamic-Inspection/university2_3d.png', dpi=150, bbox_inches='tight')
    print("Saved 3D visualization to university2_3d.png")
    plt.show()


if __name__ == '__main__':
    vgh_file = sys.argv[1] if len(sys.argv) > 1 else '/home/tharushi/Documents/university2.vgh'

    print(f"Parsing: {vgh_file}")
    nodes, connect_edges, poly_edges, contour_edges = parse_vgh(vgh_file)
    print_stats(nodes, connect_edges, poly_edges, contour_edges)

    visualize_2d(nodes, connect_edges, poly_edges, contour_edges, title=f"university2.vgh - Visibility Graph")
    visualize_3d(nodes, connect_edges, poly_edges, contour_edges, title=f"university2.vgh - 3D Visibility Graph")
