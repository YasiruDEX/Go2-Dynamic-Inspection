#!/usr/bin/env python3
"""
ROS2 node to parse a .vgh (Visibility Graph) file and publish it as
MarkerArray messages for visualization in RViz2.

Usage:
  # Terminal 1 - run this publisher
  source /opt/ros/humble/setup.bash
  python3 visualize_vgh_rviz.py [path_to_vgh_file]

  # Terminal 2 - launch rviz2
  rviz2

  Then in RViz:
    - Set Fixed Frame to "map"
    - Add display -> By topic -> /vgh_graph/...  (MarkerArray)
"""

import struct
import sys
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def parse_vgh(filename):
    """Parse a .vgh binary file and return nodes and edge lists."""
    nodes = []
    connect_edges = []
    poly_edges = []
    contour_edges = []

    with open(filename, 'rb') as f:
        graph_size = struct.unpack('<Q', f.read(8))[0]
        node_connections = []

        for i in range(graph_size):
            node_id = struct.unpack('<Q', f.read(8))[0]
            x, y, z = struct.unpack('<3f', f.read(12))
            is_covered = struct.unpack('<?', f.read(1))[0]
            is_frontier = struct.unpack('<?', f.read(1))[0]
            is_navpoint = struct.unpack('<?', f.read(1))[0]
            is_boundary = struct.unpack('<?', f.read(1))[0]
            free_direct = struct.unpack('<i', f.read(4))[0]

            connect_size = struct.unpack('<Q', f.read(8))[0]
            connect_indices = [struct.unpack('<Q', f.read(8))[0] for _ in range(connect_size)]

            poly_size = struct.unpack('<Q', f.read(8))[0]
            poly_indices = [struct.unpack('<Q', f.read(8))[0] for _ in range(poly_size)]

            contour_size = struct.unpack('<Q', f.read(8))[0]
            contour_indices = [struct.unpack('<Q', f.read(8))[0] for _ in range(contour_size)]

            nodes.append({
                'id': node_id, 'x': x, 'y': y, 'z': z,
                'is_covered': is_covered, 'is_frontier': is_frontier,
                'is_navpoint': is_navpoint, 'is_boundary': is_boundary,
                'free_direct': free_direct,
            })
            node_connections.append({
                'connect': connect_indices,
                'poly': poly_indices,
                'contour': contour_indices,
            })

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


class VghRvizPublisher(Node):
    def __init__(self, vgh_file):
        super().__init__('vgh_rviz_visualizer')
        self.get_logger().info(f'Parsing VGH file: {vgh_file}')
        self.nodes, self.connect_edges, self.poly_edges, self.contour_edges = parse_vgh(vgh_file)
        self.get_logger().info(
            f'Loaded {len(self.nodes)} nodes, '
            f'{len(self.connect_edges)} nav edges, '
            f'{len(self.poly_edges)} poly edges, '
            f'{len(self.contour_edges)} contour edges'
        )

        # Publishers
        self.node_pub = self.create_publisher(MarkerArray, '/vgh_graph/nodes', 1)
        self.nav_edge_pub = self.create_publisher(MarkerArray, '/vgh_graph/nav_edges', 1)
        self.poly_edge_pub = self.create_publisher(MarkerArray, '/vgh_graph/poly_edges', 1)
        self.contour_edge_pub = self.create_publisher(MarkerArray, '/vgh_graph/contour_edges', 1)
        self.frontier_pub = self.create_publisher(MarkerArray, '/vgh_graph/frontier_nodes', 1)
        self.navpoint_pub = self.create_publisher(MarkerArray, '/vgh_graph/navpoint_nodes', 1)
        self.id_label_pub = self.create_publisher(MarkerArray, '/vgh_graph/node_ids', 1)

        # Publish at 1 Hz (latched-like behavior)
        self.timer = self.create_timer(1.0, self.publish_all)
        self.get_logger().info('Publishing graph markers on /vgh_graph/* topics. Open RViz2 and add MarkerArray displays.')

    def _make_edge_marker(self, marker_id, edges, nodes, r, g, b, a, width, ns):
        """Create a LINE_LIST marker for a set of edges."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width  # line width

        marker.color = ColorRGBA(r=r, g=g, b=b, a=a)

        for i, j in edges:
            p1 = Point(x=float(nodes[i]['x']), y=float(nodes[i]['y']), z=float(nodes[i]['z']))
            p2 = Point(x=float(nodes[j]['x']), y=float(nodes[j]['y']), z=float(nodes[j]['z']))
            marker.points.append(p1)
            marker.points.append(p2)

        return marker

    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        nodes = self.nodes

        # ── 1. ALL NODES as small spheres (SPHERE_LIST) ──
        node_marker = Marker()
        node_marker.header.frame_id = 'map'
        node_marker.header.stamp = stamp
        node_marker.ns = 'all_nodes'
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.pose.orientation.w = 1.0
        node_marker.scale.x = 0.15
        node_marker.scale.y = 0.15
        node_marker.scale.z = 0.15

        for n in nodes:
            node_marker.points.append(
                Point(x=float(n['x']), y=float(n['y']), z=float(n['z'])))
            # Color by type
            if n['is_frontier']:
                node_marker.colors.append(ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9))   # red
            elif n['is_boundary']:
                node_marker.colors.append(ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9))   # blue
            elif n['is_navpoint']:
                node_marker.colors.append(ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.9))  # orange
            elif n['is_covered']:
                node_marker.colors.append(ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.7))   # green
            else:
                node_marker.colors.append(ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.5))   # gray

        node_arr = MarkerArray()
        node_arr.markers.append(node_marker)
        self.node_pub.publish(node_arr)

        # ── 2. FRONTIER NODES (larger, bright red) ──
        frontier_marker = Marker()
        frontier_marker.header.frame_id = 'map'
        frontier_marker.header.stamp = stamp
        frontier_marker.ns = 'frontier_nodes'
        frontier_marker.id = 0
        frontier_marker.type = Marker.SPHERE_LIST
        frontier_marker.action = Marker.ADD
        frontier_marker.pose.orientation.w = 1.0
        frontier_marker.scale.x = 0.25
        frontier_marker.scale.y = 0.25
        frontier_marker.scale.z = 0.25
        frontier_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        for n in nodes:
            if n['is_frontier']:
                frontier_marker.points.append(
                    Point(x=float(n['x']), y=float(n['y']), z=float(n['z'])))

        frontier_arr = MarkerArray()
        frontier_arr.markers.append(frontier_marker)
        self.frontier_pub.publish(frontier_arr)

        # ── 3. NAVPOINT NODES (larger, orange) ──
        navpoint_marker = Marker()
        navpoint_marker.header.frame_id = 'map'
        navpoint_marker.header.stamp = stamp
        navpoint_marker.ns = 'navpoint_nodes'
        navpoint_marker.id = 0
        navpoint_marker.type = Marker.SPHERE_LIST
        navpoint_marker.action = Marker.ADD
        navpoint_marker.pose.orientation.w = 1.0
        navpoint_marker.scale.x = 0.3
        navpoint_marker.scale.y = 0.3
        navpoint_marker.scale.z = 0.3
        navpoint_marker.color = ColorRGBA(r=1.0, g=0.65, b=0.0, a=1.0)
        for n in nodes:
            if n['is_navpoint']:
                navpoint_marker.points.append(
                    Point(x=float(n['x']), y=float(n['y']), z=float(n['z'])))

        navpoint_arr = MarkerArray()
        navpoint_arr.markers.append(navpoint_marker)
        self.navpoint_pub.publish(navpoint_arr)

        # ── 4. NAVIGATION EDGES (white/gray, thin) ──
        nav_edge = self._make_edge_marker(
            0, self.connect_edges, nodes,
            r=0.8, g=0.8, b=0.8, a=0.4, width=0.03, ns='nav_edges')
        nav_arr = MarkerArray()
        nav_arr.markers.append(nav_edge)
        self.nav_edge_pub.publish(nav_arr)

        # ── 5. POLYGON EDGES (cyan, medium) ──
        poly_edge = self._make_edge_marker(
            0, self.poly_edges, nodes,
            r=0.0, g=0.8, b=1.0, a=0.6, width=0.05, ns='poly_edges')
        poly_arr = MarkerArray()
        poly_arr.markers.append(poly_edge)
        self.poly_edge_pub.publish(poly_arr)

        # ── 6. CONTOUR EDGES (green, medium) ──
        contour_edge = self._make_edge_marker(
            0, self.contour_edges, nodes,
            r=0.0, g=1.0, b=0.3, a=0.6, width=0.05, ns='contour_edges')
        contour_arr = MarkerArray()
        contour_arr.markers.append(contour_edge)
        self.contour_edge_pub.publish(contour_arr)


def main():
    rclpy.init()

    vgh_file = sys.argv[1] if len(sys.argv) > 1 else '/home/tharushi/Documents/university2.vgh'
    node = VghRvizPublisher(vgh_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
