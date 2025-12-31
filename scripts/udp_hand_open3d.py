#!/usr/bin/env python3
"""
UDP hand position visualizer using Open3D for 3D visualization.

Reuses PacketReceiver from udp_hand_viz.py to stream wrist + 15 finger joint positions.
Displays skeleton as spheres at joints and cylinders along bones.

Fingers on left hand: cyan
Fingers on right hand: magenta
"""

import sys
import os
import threading
import numpy as np
import open3d as o3d

# Import PacketReceiver from udp_hand_viz
sys.path.insert(0, os.path.dirname(__file__))
from udp_hand_viz import PacketReceiver, BONE_CONNECTIONS

class HandVisualizer3D:
    """Open3D-based hand skeleton visualizer."""
    
    def __init__(self, port=16000):
        self.receiver = PacketReceiver(port)
        self.receiver.start()
        
        # Open3D visualization
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Hand Skeleton Visualizer", width=1280, height=720)
        
        # Geometry containers (will be updated each frame)
        self.geometries = {
            "left_spheres": [],
            "left_cylinders": [],
            "right_spheres": [],
            "right_cylinders": [],
            "axes": self._create_axes()
        }
        
        # Add initial geometries
        self.vis.add_geometry(self.geometries["axes"])
        
        # Camera settings
        ctr = self.vis.get_view_control()
        ctr.set_zoom(0.8)
        
        # State for geometry management
        self.last_left_spheres = []
        self.last_left_cylinders = []
        self.last_right_spheres = []
        self.last_right_cylinders = []
        
        print(f"[Visualizer] Listening on port {port}...")
        print("Press 'Q' in window or Ctrl+C in terminal to exit")
    
    def _create_axes(self, size=0.1):
        """Create a coordinate frame at origin."""
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])
        return mesh_frame
    
    def _create_sphere(self, center, radius=0.01, color=None):
        """Create a sphere at center with given color."""
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.translate(center)
        if color is not None:
            sphere.paint_uniform_color(color)
        return sphere
    
    def _create_cylinder_between(self, p0, p1, radius=0.005, color=None):
        """
        Create a cylinder between two points p0 and p1.
        """
        p0 = np.array(p0)
        p1 = np.array(p1)
        
        direction = p1 - p0
        length = np.linalg.norm(direction)
        
        if length < 1e-6:
            return None
        
        direction = direction / length
        
        # Create cylinder along Z axis
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length)
        
        # Rotate to align with direction
        z_axis = np.array([0, 0, 1])
        
        # Handle case where direction is parallel to z_axis
        if np.abs(np.dot(direction, z_axis)) < 0.9999:
            axis = np.cross(z_axis, direction)
            axis = axis / np.linalg.norm(axis)
            angle = np.arccos(np.dot(z_axis, direction))
            
            # Create rotation matrix
            rotation_matrix = self._rotation_matrix_from_axis_angle(axis, angle)
            cylinder.rotate(rotation_matrix, center=np.array([0, 0, 0]))
        else:
            # Direction is parallel to z-axis (either same or opposite)
            if np.dot(direction, z_axis) < 0:
                rotation_matrix = self._rotation_matrix_from_axis_angle(np.array([1, 0, 0]), np.pi)
                cylinder.rotate(rotation_matrix, center=np.array([0, 0, 0]))
        
        # Position cylinder center at midpoint
        midpoint = (p0 + p1) / 2
        cylinder.translate(midpoint)
        
        if color is not None:
            cylinder.paint_uniform_color(color)
        
        return cylinder
    
    def _rotation_matrix_from_axis_angle(self, axis, angle):
        """Create a 3x3 rotation matrix from axis-angle representation."""
        axis = axis / np.linalg.norm(axis)
        a = np.cos(angle / 2.0)
        b, c, d = -axis * np.sin(angle / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        
        return np.array([
            [aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
            [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
            [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]
        ])
    
    def _build_hand_geometry(self, wrist, positions, color):
        """Build spheres and cylinders for a hand."""
        spheres = []
        cylinders = []
        
        if wrist is None or positions is None:
            return spheres, cylinders
        
        wrist_pos = wrist['position'] * 0.01
        
        # Add sphere at wrist
        wrist_sphere = self._create_sphere(wrist_pos, radius=0.008, color=color)
        if wrist_sphere:
            spheres.append(wrist_sphere)
        
        # Add spheres and cylinders for finger joints
        for parent_idx, child_idx in BONE_CONNECTIONS:
            # Get start point
            if parent_idx is None:
                start_pos = wrist_pos
            else:
                if positions[parent_idx] is None:
                    continue
                start_pos = positions[parent_idx] * 0.01
            
            # Get end point
            if child_idx is None or positions[child_idx] is None:
                continue
            end_pos = positions[child_idx] * 0.01
            
            # Add sphere at child joint
            child_sphere = self._create_sphere(end_pos, radius=0.008, color=color)
            if child_sphere:
                spheres.append(child_sphere)
            
            # Add cylinder between start and end
            cylinder = self._create_cylinder_between(start_pos, end_pos, radius=0.004, color=color)
            if cylinder:
                cylinders.append(cylinder)
        
        return spheres, cylinders
    
    def _update_geometries(self):
        """Update visualization with latest packet data."""
        packet = self.receiver.get_latest()
        
        # Left hand (cyan)
        left_color = np.array([0, 1, 1])
        left_spheres, left_cylinders = self._build_hand_geometry(
            packet["left_wrist"], packet["left_positions"], left_color
        )
        
        # Right hand (magenta)
        right_color = np.array([1, 0, 1])
        right_spheres, right_cylinders = self._build_hand_geometry(
            packet["right_wrist"], packet["right_positions"], right_color
        )
        
        # Remove old geometries
        for geom in self.last_left_spheres + self.last_left_cylinders:
            self.vis.remove_geometry(geom, reset_bounding_box=False)
        for geom in self.last_right_spheres + self.last_right_cylinders:
            self.vis.remove_geometry(geom, reset_bounding_box=False)
        
        # Add new geometries
        for geom in left_spheres + left_cylinders:
            self.vis.add_geometry(geom, reset_bounding_box=False)
        for geom in right_spheres + right_cylinders:
            self.vis.add_geometry(geom, reset_bounding_box=False)
        
        # Store references
        self.last_left_spheres = left_spheres
        self.last_left_cylinders = left_cylinders
        self.last_right_spheres = right_spheres
        self.last_right_cylinders = right_cylinders
        
        # Auto-fit view if we have data
        if left_spheres or right_spheres:
            self.vis.reset_view_point(True)
    
    def run(self):
        """Run the visualization loop."""
        try:
            while self.vis.poll_events():
                self._update_geometries()
                self.vis.update_renderer()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
    
    def stop(self):
        """Stop receiver and close visualization."""
        print("[Visualizer] Shutting down...")
        self.receiver.stop()
        self.vis.destroy_window()

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Open3D hand skeleton visualizer')
    parser.add_argument('--port', type=int, default=16000, help='UDP port to listen on')
    args = parser.parse_args()
    
    viz = HandVisualizer3D(port=args.port)
    viz.run()
