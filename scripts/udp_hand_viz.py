#!/usr/bin/env python3
"""
UDP hand position visualizer matching main.cpp SkeletonViewer approach.

Receives live UDP packets with wrist pose + finger joint world positions,
draws skeleton as connected bones (parent->child joints) in 3D matplotlib.

Packet format (HND1):
  Header (16 bytes): magic(4B) + version(u16) + flags(u16) + timestamp_us(u64)
  DOF info (4 bytes): total_dof(u16) + reserved(u16)
  Per side: wrist(7 floats) + 15 joint positions(45 floats) = 52 floats per side
  Total per frame: 104 floats (52 left + 52 right)
"""

import socket
import struct
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from collections import deque

# Finger joint ordering: 3 joints per finger (MCP, PIP, DIP)
FINGER_NAMES = [
    "Thumb", "Index", "Middle", "Ring", "Pinky"
]
JOINT_NAMES = []
for finger in FINGER_NAMES:
    for i in range(1, 4):
        JOINT_NAMES.append(f"{finger}{i}")
assert len(JOINT_NAMES) == 15

# Bone connections: (parent_idx, child_idx) for each finger
# Parent of first joint (MCP) is wrist at index -1
# MCP (i*3+0) -> PIP (i*3+1) -> DIP (i*3+2)
BONE_CONNECTIONS = []
for finger_idx in range(5):
    mcp_idx = finger_idx * 3 + 0
    pip_idx = finger_idx * 3 + 1
    dip_idx = finger_idx * 3 + 2
    BONE_CONNECTIONS.append((None, mcp_idx))   # wrist -> MCP
    BONE_CONNECTIONS.append((mcp_idx, pip_idx))  # MCP -> PIP
    BONE_CONNECTIONS.append((pip_idx, dip_idx))  # PIP -> DIP

class PacketReceiver:
    """Listen for UDP hand position packets."""
    def __init__(self, port=16000):
        self.port = port
        self.sock = None
        self.running = False
        self.last_packet = {"left_wrist": None, "left_positions": None, "right_wrist": None, "right_positions": None}
        self.lock = threading.Lock()
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
    
    def _listen_loop(self):
        """Receive UDP packets in background thread."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", self.port))
            print(f"[Receiver] Listening on port {self.port}...")
            
            packet_count = 0
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(65536)
                    if len(data) < 20:
                        continue
                    
                    # Check magic
                    if data[:4] != b'HND1':
                        print(f"[Receiver] Invalid magic: {data[:4]}, skipping packet")
                        continue
                    
                    # Parse header (match udp_hand_listen.py format)
                    version, flags = struct.unpack_from('<HH', data, 4)
                    ts_us = struct.unpack_from('<Q', data, 8)[0]
                    dof, reserved = struct.unpack_from('<HH', data, 16)
                    
                    # Payload starts at offset 20
                    payload_offset = 20
                    left_wrist, left_positions = self._parse_side(data, payload_offset)
                    right_wrist, right_positions = self._parse_side(data, payload_offset + 208)  # 52 floats per side = 208 bytes
                    
                    packet_count += 1

                    with self.lock:
                        self.last_packet = {
                            "left_wrist": left_wrist,
                            "left_positions": left_positions,
                            "right_wrist": right_wrist,
                            "right_positions": right_positions,
                            "timestamp": ts_us
                        }
                except Exception as e:
                    print(f"[Receiver] Parse error: {e}")
        except Exception as e:
            print(f"[Receiver] Socket error: {e}")
        finally:
            if self.sock:
                self.sock.close()
    
    def _parse_side(self, data, offset):
        """Extract wrist pose (7 floats) + 15 joint positions (45 floats) from packet."""
        try:
            if offset + 28 > len(data):
                return None, None
                
            # Wrist: px, py, pz, qx, qy, qz, qw (7 floats = 28 bytes)
            wrist_data = struct.unpack_from('<7f', data, offset)
            wrist = {
                'position': np.array(wrist_data[:3]),
                'quaternion': np.array(wrist_data[3:7])
            }
            
            # 15 joint positions (3 floats each = 45 floats = 180 bytes)
            positions = []
            joint_offset = offset + 28
            for i in range(15):
                if joint_offset + 12 > len(data):
                    positions.append(None)
                    continue
                    
                x, y, z = struct.unpack_from('<fff', data, joint_offset)
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    positions.append(np.array([x, y, z]))
                else:
                    positions.append(None)
                joint_offset += 12
            
            return wrist, positions
        except Exception as e:
            print(f"[Receiver] Parse error at offset {offset}: {e}")
            import traceback
            traceback.print_exc()
            return None, None
    
    def get_latest(self):
        """Get most recent packet data."""
        with self.lock:
            return dict(self.last_packet)
    
    def stop(self):
        self.running = False

def build_hand_lines(wrist, positions):
    """
    Build line segments connecting finger joints.
    wrist: dict with 'position' (np.array [x,y,z])
    positions: list of np.array or None for each joint
    Returns: list of (start_point, end_point) tuples for each bone
    """
    lines = []
    wrist_pos = wrist['position'] * 0.01  # scale like main.cpp viewer
    
    for parent_idx, child_idx in BONE_CONNECTIONS:
        if parent_idx is None:
            # Connect wrist to first joint
            start = wrist_pos
        else:
            if positions[parent_idx] is None:
                continue
            start = positions[parent_idx] * 0.01
        
        if child_idx is None or positions[child_idx] is None:
            continue
        end = positions[child_idx] * 0.01
        
        lines.append((start, end))
    
    return lines

class HandVisualizer:
    def __init__(self, port=16000):
        self.receiver = PacketReceiver(port)
        self.receiver.start()
        
        # Create figure and 3D axes
        self.fig = plt.figure(figsize=(12, 6))
        self.ax_left = self.fig.add_subplot(121, projection='3d')
        self.ax_right = self.fig.add_subplot(122, projection='3d')
        
        # Set axis labels
        for ax, title in [(self.ax_left, "Left Hand"), (self.ax_right, "Right Hand")]:
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(title)
        
        # Line containers for animation
        self.left_lines = []
        self.right_lines = []
        self.left_scatter = None
        self.right_scatter = None
        
        # Status text
        self.status_left = self.fig.text(0.25, 0.02, "No data", ha='center')
        self.status_right = self.fig.text(0.75, 0.02, "No data", ha='center')
        
        # Add initial line segments (max 15 lines per hand for 5 fingers × 3 bones)
        for _ in range(15):
            l, = self.ax_left.plot([], [], 'b-', linewidth=2)
            self.left_lines.append(l)
            l, = self.ax_right.plot([], [], 'r-', linewidth=2)
            self.right_lines.append(l)
        
        self.anim = FuncAnimation(self.fig, self._update, interval=50, blit=False, cache_frame_data=False)
        
    def _update(self, frame):
        """Update animation frame with latest packet data."""
        packet = self.receiver.get_latest()
        all_points = []  # Collect all valid points for auto-scaling
        
        # Left hand
        if packet["left_wrist"] and packet["left_positions"]:
            left_lines_data = build_hand_lines(packet["left_wrist"], packet["left_positions"])
            # Only update lines that have data
            for i in range(len(left_lines_data)):
                if i < len(self.left_lines):
                    start, end = left_lines_data[i]
                    self.left_lines[i].set_data([start[0], end[0]], [start[1], end[1]])
                    self.left_lines[i].set_3d_properties([start[2], end[2]])
            
            # Joint scatter points - remove old scatter and create new
            left_points = np.array([packet["left_wrist"]['position'] * 0.01] + 
                                  [p * 0.01 if p is not None else np.array([np.nan, np.nan, np.nan]) 
                                   for p in packet["left_positions"]])
            valid = ~np.isnan(left_points).any(axis=1)
            valid_points = left_points[valid]
            
            # Filter out outliers and inf values
            if valid_points.shape[0] > 0:
                # Remove inf values
                valid_points = valid_points[~np.isinf(valid_points).any(axis=1)]
                
                # Remove points that are too far from the median (outliers)
                if valid_points.shape[0] > 3:
                    median_pos = np.median(valid_points, axis=0)
                    distances = np.linalg.norm(valid_points - median_pos, axis=1)
                    median_dist = np.median(distances)
                    # Keep points within 5x median distance
                    outlier_threshold = median_dist * 5
                    valid_points = valid_points[distances <= outlier_threshold]
            
            if valid_points.shape[0] > 0:
                all_points.append(valid_points)
                # Remove old scatter if it exists
                if self.left_scatter is not None:
                    self.left_scatter.remove()
                self.left_scatter = self.ax_left.scatter(valid_points[:, 0], valid_points[:, 1], valid_points[:, 2], 
                                                        c='cyan', s=20)
            
            self.status_left.set_text(f"Left: {len(left_lines_data)} bones, ts={packet.get('timestamp', 0)}")
        else:
            self.status_left.set_text("Left: Waiting...")
        
        # Right hand
        if packet["right_wrist"] and packet["right_positions"]:
            right_lines_data = build_hand_lines(packet["right_wrist"], packet["right_positions"])
            # Only update lines that have data
            for i in range(len(right_lines_data)):
                if i < len(self.right_lines):
                    start, end = right_lines_data[i]
                    self.right_lines[i].set_data([start[0], end[0]], [start[1], end[1]])
                    self.right_lines[i].set_3d_properties([start[2], end[2]])
            
            # Joint scatter points - remove old scatter and create new
            right_points = np.array([packet["right_wrist"]['position'] * 0.01] + 
                                   [p * 0.01 if p is not None else np.array([np.nan, np.nan, np.nan]) 
                                    for p in packet["right_positions"]])
            valid = ~np.isnan(right_points).any(axis=1)
            valid_points = right_points[valid]
            
            # Filter out outliers and inf values
            if valid_points.shape[0] > 0:
                # Remove inf values
                valid_points = valid_points[~np.isinf(valid_points).any(axis=1)]
                
                # Remove points that are too far from the median (outliers)
                if valid_points.shape[0] > 3:
                    median_pos = np.median(valid_points, axis=0)
                    distances = np.linalg.norm(valid_points - median_pos, axis=1)
                    median_dist = np.median(distances)
                    # Keep points within 5x median distance
                    outlier_threshold = median_dist * 5
                    valid_points = valid_points[distances <= outlier_threshold]
            
            if valid_points.shape[0] > 0:
                all_points.append(valid_points)
                # Remove old scatter if it exists
                if self.right_scatter is not None:
                    self.right_scatter.remove()
                self.right_scatter = self.ax_right.scatter(valid_points[:, 0], valid_points[:, 1], valid_points[:, 2], 
                                                         c='magenta', s=20)
            
            self.status_right.set_text(f"Right: {len(right_lines_data)} bones, ts={packet.get('timestamp', 0)}")
        else:
            self.status_right.set_text("Right: Waiting...")
        
        # Auto-scale axes based on all collected points
        if len(all_points) > 0:
            combined_points = np.vstack(all_points)
            padding = 0.05  # 5% padding around bounds
            
            x_min, x_max = combined_points[:, 0].min(), combined_points[:, 0].max()
            y_min, y_max = combined_points[:, 1].min(), combined_points[:, 1].max()
            z_min, z_max = combined_points[:, 2].min(), combined_points[:, 2].max()
            
            x_range = max(x_max - x_min, 0.01)
            y_range = max(y_max - y_min, 0.01)
            z_range = max(z_max - z_min, 0.01)
            
            for ax in [self.ax_left, self.ax_right]:
                ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
                ax.set_ylim(y_min - padding * y_range, y_max + padding * y_range)
                ax.set_zlim(z_min - padding * z_range, z_max + padding * z_range)
        
        return self.left_lines + self.right_lines + [self.status_left, self.status_right]
    
    def show(self):
        """Display the visualization."""
        plt.tight_layout()
        plt.show()
    
    def stop(self):
        self.receiver.stop()

if __name__ == '__main__':
    import sys
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 16000
    
    print(f"Starting hand visualizer on port {port}...")
    print("Packet format: HND1 magic + wrist pose (7 floats) + 15 finger positions (45 floats) per side")
    print("Press Ctrl+C to exit")
    
    viz = HandVisualizer(port)
    try:
        viz.show()
    finally:
        viz.stop()

