#!/usr/bin/env python3
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class UpperBodyPanel(Node):
    def __init__(self):
        super().__init__('upper_body_panel')
        self.declare_parameter('min_pos', -0.30)   # bottom (500 mm stroke)
        self.declare_parameter('max_pos', 0.065)    # top
        self.declare_parameter('step', 0.01)
        self.declare_parameter('speed', 0.05)      # m/s ramp speed
        self.declare_parameter('topic', '/upper_body/cmd_pos')

        self.min_pos = self.get_parameter('min_pos').value
        self.max_pos = self.get_parameter('max_pos').value
        self.step = self.get_parameter('step').value
        self.speed = self.get_parameter('speed').value

        self.pub = self.create_publisher(
            Float64, self.get_parameter('topic').value, 10)
        self.target = 0.0
        self.current = 0.0
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)

        self.root = tk.Tk()
        self.root.title('Upper Body Control')
        self.root.attributes('-topmost', True)

        self.slider = tk.Scale(
            self.root, from_=self.max_pos, to=self.min_pos,
            resolution=0.001, orient=tk.VERTICAL, length=250,
            label='Height (m)', command=self.on_slider)
        self.slider.pack(side=tk.LEFT, padx=10, pady=10)
        self.slider.set(0.0)

        frame = tk.Frame(self.root)
        frame.pack(side=tk.LEFT, padx=10, pady=10)
        tk.Button(frame, text='▲ UP', width=10, height=2,
                  command=lambda: self.nudge(+self.step)).pack(pady=5)
        tk.Button(frame, text='HOME', width=10,
                  command=lambda: self.set_target(0.0)).pack(pady=5)
        tk.Button(frame, text='▼ DOWN', width=10, height=2,
                  command=lambda: self.nudge(-self.step)).pack(pady=5)
        self.label = tk.Label(frame, text='0.000 m')
        self.label.pack(pady=5)

    def on_slider(self, v):
        self.target = float(v)

    def set_target(self, v):
        self.target = min(max(v, self.min_pos), self.max_pos)
        self.slider.set(self.target)

    def nudge(self, d):
        self.set_target(self.target + d)

    def tick(self):
        max_step = self.speed * self.dt
        err = self.target - self.current
        self.current += max(-max_step, min(max_step, err))
        self.pub.publish(Float64(data=self.current))
        self.label.config(text=f'{self.current:.3f} m')

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                self.root.update()
            except tk.TclError:
                break


def main():
    rclpy.init()
    node = UpperBodyPanel()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()