import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("amr_hmi_cmd_vel_publisher")
        self.publisher_ = self.create_publisher(Twist, "/diff_cont/cmd_vel_unstamped", 10)
        

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher_.publish(msg)


class AMRHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("AMR HMI - Mapping Control")
        self.root.geometry("820x520")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ROS
        rclpy.init(args=None)
        self.ros_node = CmdVelPublisher()
        self.ros_running = True
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Launch process handle
        self.mapping_process = None
        self.exploration_process = None

        # Continuous drive state
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.drive_active = False
        self.drive_thread = None

        # Default values
        self.linear_speed = tk.DoubleVar(value=0.20)
        self.angular_speed = tk.DoubleVar(value=0.80)
        self.map_name = tk.StringVar(value="my_map")

        self.build_ui()

    def build_ui(self):
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.pack(fill="both", expand=True)

        left_frame = ttk.LabelFrame(main_frame, text="Launch / Mapping", padding=10)
        left_frame.pack(side="left", fill="y", padx=(0, 10))

        right_frame = ttk.LabelFrame(main_frame, text="Manual Drive", padding=10)
        right_frame.pack(side="left", fill="both", expand=True)

        bottom_frame = ttk.LabelFrame(self.root, text="Status", padding=10)
        bottom_frame.pack(fill="both", expand=False, padx=12, pady=(0, 12))

        # Left frame widgets
        ttk.Button(
            left_frame,
            text="Start Mapping Mode",
            command=self.start_mapping_mode,
            width=24
        ).pack(pady=6)

        ttk.Button(
            left_frame,
            text="Stop Mapping Mode",
            command=self.stop_mapping_mode,
            width=24
        ).pack(pady=6)

        ttk.Label(left_frame, text="Map name:").pack(anchor="w", pady=(16, 4))
        ttk.Entry(left_frame, textvariable=self.map_name, width=24).pack(pady=4)

        ttk.Button(
            left_frame,
            text="Save Map",
            command=self.save_map,
            width=24
        ).pack(pady=6)

        ttk.Separator(left_frame, orient="horizontal").pack(fill="x", pady=14)

        ttk.Button(
            left_frame,
            text="Shutdown All",
            command=self.shutdown_all,
            width=24
        ).pack(pady=6)

        ttk.Button(
            left_frame,
            text="Start Exploration",
            command=self.start_exploration_mode,
            width=24
        ).pack(pady=6)

        ttk.Button(
            left_frame,
            text="Stop Exploration",
            command=self.stop_exploration_mode,
            width=24
        ).pack(pady=6)

        # Speed controls
        ttk.Label(left_frame, text="Linear speed (m/s)").pack(anchor="w", pady=(16, 4))
        ttk.Scale(
            left_frame,
            from_=0.05,
            to=0.60,
            variable=self.linear_speed,
            orient="horizontal",
            length=180
        ).pack()

        self.linear_label = ttk.Label(left_frame, text=f"{self.linear_speed.get():.2f}")
        self.linear_label.pack(anchor="e", pady=(2, 8))

        ttk.Label(left_frame, text="Angular speed (rad/s)").pack(anchor="w", pady=(8, 4))
        ttk.Scale(
            left_frame,
            from_=0.20,
            to=2.00,
            variable=self.angular_speed,
            orient="horizontal",
            length=180
        ).pack()

        self.angular_label = ttk.Label(left_frame, text=f"{self.angular_speed.get():.2f}")
        self.angular_label.pack(anchor="e", pady=(2, 8))

        self.linear_speed.trace_add("write", self.update_speed_labels)
        self.angular_speed.trace_add("write", self.update_speed_labels)

        # Right frame widgets
        drive_pad = ttk.Frame(right_frame)
        drive_pad.pack(expand=True)

        btn_forward = ttk.Button(drive_pad, text="Forward", width=14)
        btn_left = ttk.Button(drive_pad, text="Left", width=14)
        btn_stop = ttk.Button(drive_pad, text="Stop", width=14, command=self.stop_motion)
        btn_right = ttk.Button(drive_pad, text="Right", width=14)
        btn_backward = ttk.Button(drive_pad, text="Backward", width=14)

        btn_forward.grid(row=0, column=1, padx=8, pady=8)
        btn_left.grid(row=1, column=0, padx=8, pady=8)
        btn_stop.grid(row=1, column=1, padx=8, pady=8)
        btn_right.grid(row=1, column=2, padx=8, pady=8)
        btn_backward.grid(row=2, column=1, padx=8, pady=8)

        self.bind_hold_button(
            btn_forward,
            linear_getter=lambda: self.linear_speed.get(),
            angular_getter=lambda: 0.0
        )
        self.bind_hold_button(
            btn_backward,
            linear_getter=lambda: -self.linear_speed.get(),
            angular_getter=lambda: 0.0
        )
        self.bind_hold_button(
            btn_left,
            linear_getter=lambda: 0.0,
            angular_getter=lambda: self.angular_speed.get()
        )
        self.bind_hold_button(
            btn_right,
            linear_getter=lambda: 0.0,
            angular_getter=lambda: -self.angular_speed.get()
        )

        ttk.Label(
            right_frame,
            text="Hold a direction button to move. Release to stop.",
        ).pack(pady=(8, 0))

        # Bottom status box
        self.status_text = tk.Text(bottom_frame, height=10, wrap="word", state="disabled")
        self.status_text.pack(fill="both", expand=True)

        self.log("HMI started.")
        self.log("Ready to launch mapping mode.")

    def update_speed_labels(self, *args):
        self.linear_label.config(text=f"{self.linear_speed.get():.2f}")
        self.angular_label.config(text=f"{self.angular_speed.get():.2f}")

    def log(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        self.status_text.config(state="normal")
        self.status_text.insert("end", f"[{timestamp}] {message}\n")
        self.status_text.see("end")
        self.status_text.config(state="disabled")

    def spin_ros(self):
        while self.ros_running and rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def bind_hold_button(self, button, linear_getter, angular_getter):
        button.bind(
            "<ButtonPress-1>",
            lambda event: self.start_motion(linear_getter(), angular_getter())
        )
        button.bind("<ButtonRelease-1>", lambda event: self.stop_motion())

    def motion_loop(self):
        while self.drive_active:
            self.ros_node.publish_cmd(self.current_linear, self.current_angular)
            time.sleep(0.1)

    def start_motion(self, linear_x: float, angular_z: float):
        self.current_linear = linear_x
        self.current_angular = angular_z

        if not self.drive_active:
            self.drive_active = True
            self.drive_thread = threading.Thread(target=self.motion_loop, daemon=True)
            self.drive_thread.start()

        self.log(f"Driving: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}")

    def stop_motion(self):
        self.drive_active = False
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.ros_node.publish_cmd(0.0, 0.0)
        self.log("Motion stopped.")

    def start_mapping_mode(self):

        if self.mapping_process is not None and self.mapping_process.poll() is None:
            self.log("Mapping mode is already running.")
            return
        
        if self.exploration_process is not None and self.exploration_process.poll() is None:
            self.terminate_process_group(self.exploration_process, "exploration mode")
            self.exploration_process = None
        # Clean up any stale processes before starting a fresh mapping session
        for pattern in ["gzserver", "gzclient", "spawn_entity.py", "slam_toolbox"]:
            subprocess.run(
                ["pkill", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False
            )
        try:
            cmd = [
                "bash",
                "-c",
                "source /opt/ros/humble/setup.bash && "
                "source ~/amr_ws/install/setup.bash && "
                "ros2 launch amr_bringup mapping.launch.py"
            ]

            self.mapping_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            threading.Thread(
                target=self.read_process_output,
                args=(self.mapping_process, "MAPPING"),
                daemon=True
            ).start()

            self.log("Started mapping mode.")
        except Exception as e:
            self.log(f"Failed to start mapping mode: {e}")
            messagebox.showerror("Error", f"Could not start mapping mode.\n\n{e}")

    def stop_mapping_mode(self):
        if self.exploration_process is not None and self.exploration_process.poll() is None:
            self.terminate_process_group(self.exploration_process, "exploration mode")
            self.exploration_process = None

        if self.mapping_process is None:
            self.log("No active mapping process to stop.")
            return

        self.terminate_process_group(self.mapping_process, "mapping mode")
        self.mapping_process = None

    def read_process_output(self, process, tag):
        try:
            for line in process.stdout:
                clean = line.strip()
                if clean:
                    self.log(f"{tag}: {clean}")
        except Exception as e:
            self.log(f"Error reading {tag} output: {e}")

    def save_map(self):
        map_name = self.map_name.get().strip()
        if not map_name:
            messagebox.showwarning("Missing map name", "Please enter a map name first.")
            return

        save_dir = os.path.expanduser("~/amr_ws/maps")
        os.makedirs(save_dir, exist_ok=True)
        full_path = os.path.join(save_dir, map_name)

        self.log(f"Saving map to: {full_path}")

        try:
            cmd = [
                "bash",
                "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"source ~/amr_ws/install/setup.bash && "
                f"ros2 run nav2_map_server map_saver_cli -f {full_path}"
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60
            )

            if result.returncode == 0:
                self.log(f"Map saved successfully: {full_path}.yaml / {full_path}.pgm")
                messagebox.showinfo("Map Saved", f"Map saved successfully:\n{full_path}")
            else:
                self.log("Map saving failed.")
                self.log(result.stdout)
                self.log(result.stderr)
                messagebox.showerror(
                    "Save Failed",
                    f"Map save failed.\n\nSTDOUT:\n{result.stdout}\n\nSTDERR:\n{result.stderr}"
                )
        except Exception as e:
            self.log(f"Error saving map: {e}")
            messagebox.showerror("Error", f"Could not save map.\n\n{e}")


    def terminate_process_group(self, process, name="process"):
        if process is None:
            return

        try:
            if process.poll() is None:
                self.log(f"Stopping {name}...")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                self.log(f"{name} stopped cleanly.")
            else:
                self.log(f"{name} already stopped.")
        except subprocess.TimeoutExpired:
            self.log(f"{name} did not stop in time. Force killing...")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                self.log(f"{name} force-killed.")
            except Exception as e:
                self.log(f"Failed to force-kill {name}: {e}")
        except ProcessLookupError:
            self.log(f"{name} process group no longer exists.")
        except Exception as e:
            self.log(f"Failed to stop {name}: {e}")


    def shutdown_all(self):
        self.log("Shutdown requested.")

        # Stop robot motion first
        self.stop_motion()

        # Stop mapping launch and all children
        if self.mapping_process is not None:
            self.terminate_process_group(self.mapping_process, "mapping mode")
            self.mapping_process = None

        if self.exploration_process is not None:
            self.terminate_process_group(self.exploration_process, "exploration mode")
            self.exploration_process = None

        # Extra cleanup for stale ROS/Gazebo processes that may survive
        cleanup_patterns = [
            "gzserver",
            "gzclient",
            "spawn_entity.py",
            "slam_toolbox",
            "robot_state_publisher",
            "frontier_detector",
            "frontier_explorer",
            "controller_manager",
            "spawner",
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",
            "lifecycle_manager",
            "amr_bringup mapping.launch.py",
            "amr_bringup exploration.launch.py",
        ]

        for pattern in cleanup_patterns:
            try:
                subprocess.run(
                    ["pkill", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False
                )
            except Exception as e:
                self.log(f"Cleanup warning for '{pattern}': {e}")

        self.log("Shutdown complete.")

    def on_close(self):
        self.shutdown_all()

        self.ros_running = False
        try:
            if rclpy.ok():
                self.ros_node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass

        self.root.destroy()


    def start_exploration_mode(self):
        if self.exploration_process is not None and self.exploration_process.poll() is None:
            self.log("Exploration mode is already running.")
            return

        if self.mapping_process is None or self.mapping_process.poll() is not None:
            self.log("Cannot start exploration: mapping mode is not running.")
            messagebox.showwarning(
                "Mapping Not Running",
                "Start Mapping Mode first, then start exploration."
            )
            return

        try:
            cmd = [
                "bash",
                "-c",
                "source /opt/ros/humble/setup.bash && "
                "source ~/amr_ws/install/setup.bash && "
                "ros2 launch amr_bringup exploration.launch.py"
            ]

            self.exploration_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            threading.Thread(
                target=self.read_process_output,
                args=(self.exploration_process, "EXPLORATION"),
                daemon=True
            ).start()

            self.log("Started exploration mode.")
        except Exception as e:
            self.log(f"Failed to start exploration mode: {e}")
            messagebox.showerror("Error", f"Could not start exploration mode.\n\n{e}")


    def stop_exploration_mode(self):
        if self.exploration_process is None:
            self.log("No active exploration process to stop.")
            return

        self.terminate_process_group(self.exploration_process, "exploration mode")
        self.exploration_process = None


def main():
    root = tk.Tk()
    style = ttk.Style()
    try:
        style.theme_use("clam")
    except Exception:
        pass

    app = AMRHMI(root)
    root.mainloop()


if __name__ == "__main__":
    main()