#!/usr/bin/env python3
"""Robot control panel: slider for the upper body, rotary knobs for shoulders/arms."""
import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ---------------- theme ----------------
BG = '#161b26'
CARD = '#222a3a'
CARD_HI = '#2c3649'
TEXT = '#e8edf7'
SUBTEXT = '#8f9bb3'
ACCENT = '#4da3ff'
ACCENT_BODY = '#ffb84d'
TRACK = '#3a455c'
LED_ON = '#2ee66b'
LED_OFF = '#1f3a2a'
FONT = 'Helvetica'
SCALE = 1.2   # overall panel size (1.0 = original, 1.2 = +20%)


def S(x):
    return int(round(x * SCALE))


class Axis:
    """One ramped position channel (publishes a smoothed setpoint)."""

    def __init__(self, pub, lo, hi, speed, unit):
        self.pub, self.lo, self.hi, self.speed, self.unit = pub, lo, hi, speed, unit
        self.target = 0.0
        self.current = 0.0
        self.set_widget = None   # callable(v): move the on-screen control
        self.label = None

    def clamp(self, v):
        return min(max(v, self.lo), self.hi)

    def tick(self, dt):
        max_step = self.speed * dt
        err = self.target - self.current
        self.current += max(-max_step, min(max_step, err))
        self.pub.publish(Float64(data=self.current))
        if self.label is not None:
            if self.unit == 'm':
                self.label.config(text=f'{self.current:+.3f} m')
            else:
                self.label.config(
                    text=f'{self.current:+.2f} rad  |  {math.degrees(self.current):+.0f}°')


class Knob(tk.Canvas):
    """Rotary knob: drag with mouse, scroll wheel, double-click = home."""
    START = 135.0   # screen angle (deg, clockwise from +x) at minimum
    SWEEP = 270.0

    def __init__(self, parent, lo, hi, home, step, on_change, size=124,
                 accent=ACCENT, bg=CARD):
        self.k = S(size) / 124.0          # drawing scale factor
        size = S(size)
        super().__init__(parent, width=size, height=size, bg=bg,
                         highlightthickness=0, cursor='hand2')
        self.lo, self.hi, self.home, self.step = lo, hi, home, step
        self.on_change, self.size, self.accent = on_change, size, accent
        self.value = home
        self.bind('<Button-1>', self._drag)
        self.bind('<B1-Motion>', self._drag)
        self.bind('<Double-Button-1>', lambda e: self._emit(self.home))
        self.bind('<MouseWheel>', self._wheel)
        self.bind('<Button-4>', lambda e: self._emit(self.value + self.step))
        self.bind('<Button-5>', lambda e: self._emit(self.value - self.step))
        self._draw()

    def _frac(self, v):
        return (v - self.lo) / (self.hi - self.lo)

    def _emit(self, v):
        v = min(max(v, self.lo), self.hi)
        self.set(v)
        self.on_change(v)

    def _wheel(self, e):
        self._emit(self.value + (self.step if e.delta > 0 else -self.step))

    def _drag(self, e):
        c = self.size / 2
        ang = math.degrees(math.atan2(e.y - c, e.x - c)) % 360
        rel = (ang - self.START) % 360
        if rel > self.SWEEP:                       # in the dead zone at the bottom
            rel = 0.0 if rel > self.SWEEP + (360 - self.SWEEP) / 2 else self.SWEEP
        self._emit(self.lo + rel / self.SWEEP * (self.hi - self.lo))

    def set(self, v):
        self.value = v
        self._draw()

    def _draw(self):
        self.delete('all')
        k = self.k
        c = self.size / 2
        r = c - 10 * k
        box = (c - r, c - r, c + r, c + r)
        # track + value arc (tk arcs are counter-clockwise, ours are clockwise)
        self.create_arc(*box, start=-self.START, extent=-self.SWEEP,
                        style='arc', outline=TRACK, width=8 * k)
        frac = self._frac(self.value)
        if frac > 0.002:
            self.create_arc(*box, start=-self.START, extent=-self.SWEEP * frac,
                            style='arc', outline=self.accent, width=8 * k)
        # home tick
        ha = math.radians(self.START + self.SWEEP * self._frac(self.home))
        self.create_line(c + (r - 12 * k) * math.cos(ha), c + (r - 12 * k) * math.sin(ha),
                         c + (r + 5 * k) * math.cos(ha), c + (r + 5 * k) * math.sin(ha),
                         fill=SUBTEXT, width=2 * k)
        # knob body
        kr = r - 16 * k
        self.create_oval(c - kr, c - kr, c + kr, c + kr, fill=CARD_HI,
                         outline='#3d4960', width=2 * k)
        # pointer
        a = math.radians(self.START + self.SWEEP * frac)
        self.create_line(c + kr * 0.25 * math.cos(a), c + kr * 0.25 * math.sin(a),
                         c + kr * 0.88 * math.cos(a), c + kr * 0.88 * math.sin(a),
                         fill=TEXT, width=4 * k, capstyle='round')


class LedButton(tk.Canvas):
    """Button with a green LED. toggle=True keeps state; otherwise the LED just flashes."""

    def __init__(self, parent, text, toggle=True, command=None,
                 width=205, height=46, state=False, bg=CARD):
        super().__init__(parent, width=S(width), height=S(height), bg=bg,
                         highlightthickness=0, cursor='hand2')
        self.text, self.toggle, self.command = text, toggle, command
        self.w, self.h = S(width), S(height)
        self.state = state
        self.hover = False
        self.bind('<Button-1>', self._click)
        self.bind('<Enter>', lambda e: self._hover(True))
        self.bind('<Leave>', lambda e: self._hover(False))
        self.bind('<Configure>', self._resize)
        self._draw()

    def get(self):
        return self.state

    def _resize(self, e):
        self.w, self.h = e.width, e.height
        self._draw()

    def _hover(self, h):
        self.hover = h
        self._draw()

    def _click(self, _e):
        if self.toggle:
            self.state = not self.state
        else:
            self.state = True
            self.after(350, self._off)
        self._draw()
        if self.command:
            self.command()

    def _off(self):
        self.state = False
        self._draw()

    def _draw(self):
        self.delete('all')
        fill = '#36425a' if self.hover else CARD_HI
        self._round(2, 2, self.w - 2, self.h - 2, S(12), fill)
        cy = self.h / 2
        if self.state:
            self.create_oval(S(10), cy - S(12), S(34), cy + S(12), fill='#14532d', outline='')
        self.create_oval(S(15), cy - S(7), S(29), cy + S(7),
                         fill=LED_ON if self.state else LED_OFF,
                         outline='#0e2a1a', width=1)
        self.create_text(S(46), cy, text=self.text, anchor='w', fill=TEXT,
                         font=(FONT, 11, 'bold'))

    def _round(self, x1, y1, x2, y2, r, fill):
        pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
               x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
        self.create_polygon(pts, smooth=True, fill=fill, outline='')


class UpperBodyPanel(Node):
    PAIRS = {
        'l_sh': ('r_sh', 'shoulder'), 'r_sh': ('l_sh', 'shoulder'),
        'l_arm': ('r_arm', 'arm'),    'r_arm': ('l_arm', 'arm'),
    }

    def __init__(self):
        super().__init__('upper_body_panel')
        d = self.declare_parameter
        d('min_pos', -0.30); d('max_pos', 0.065)
        d('step', 0.01);     d('speed', 0.05)
        d('topic', '/upper_body/cmd_pos')
        d('shoulder_min', -3.14); d('shoulder_max', 1.57)
        d('shoulder_step', 0.1);  d('shoulder_speed', 0.8)
        d('arm_min', -1.57); d('arm_max', 1.57)
        d('arm_step', 0.1);  d('arm_speed', 0.8)
        d('left_shoulder_topic', '/left_shoulder/cmd_pos')
        d('right_shoulder_topic', '/right_shoulder/cmd_pos')
        d('left_arm_topic', '/left_arm/cmd_pos')
        d('right_arm_topic', '/right_arm/cmd_pos')

        g = lambda n: self.get_parameter(n).value
        mk = lambda topic: self.create_publisher(Float64, topic, 10)
        self.dt = 0.05

        self.axes = {
            'body':  Axis(mk(g('topic')), g('min_pos'), g('max_pos'), g('speed'), 'm'),
            'l_sh':  Axis(mk(g('left_shoulder_topic')), g('shoulder_min'), g('shoulder_max'), g('shoulder_speed'), 'rad'),
            'r_sh':  Axis(mk(g('right_shoulder_topic')), g('shoulder_min'), g('shoulder_max'), g('shoulder_speed'), 'rad'),
            'l_arm': Axis(mk(g('left_arm_topic')), g('arm_min'), g('arm_max'), g('arm_speed'), 'rad'),
            'r_arm': Axis(mk(g('right_arm_topic')), g('arm_min'), g('arm_max'), g('arm_speed'), 'rad'),
        }
        self.steps = {'body': g('step'), 'l_sh': g('shoulder_step'), 'r_sh': g('shoulder_step'),
                      'l_arm': g('arm_step'), 'r_arm': g('arm_step')}
        self._busy = False
        self.create_timer(self.dt, self.tick)

        self.root = tk.Tk()
        self.root.tk.call('tk', 'scaling',
                          float(self.root.tk.call('tk', 'scaling')) * SCALE)
        self.root.title('Robot Control')
        self.root.configure(bg=BG)
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)
        self._build_ui()

    # ---------------- UI ----------------
    def _card(self, parent, title, accent):
        card = tk.Frame(parent, bg=CARD, padx=16, pady=12)
        tk.Label(card, text=title, bg=CARD, fg=accent,
                 font=(FONT, 12, 'bold')).pack(anchor='w')
        tk.Frame(card, bg=accent, height=2).pack(fill='x', pady=(4, 10))
        return card

    def _build_ui(self):
        tk.Label(self.root, text='ROBOT CONTROL', bg=BG, fg=TEXT,
                 font=(FONT, 15, 'bold')).pack(pady=(12, 2))
        tk.Label(self.root, text='drag a knob  |  scroll wheel to fine-tune  |  double-click = home',
                 bg=BG, fg=SUBTEXT, font=(FONT, 9)).pack(pady=(0, 8))

        main = tk.Frame(self.root, bg=BG)
        main.pack(padx=14)

        # ----- left: upper body slider -----
        body = self._card(main, 'UPPER BODY', ACCENT_BODY)
        body.grid(row=0, column=0, sticky='ns', padx=(0, 10))
        tk.Label(body, text='Height (m)', bg=CARD, fg=SUBTEXT,
                 font=(FONT, 10)).pack()
        row = tk.Frame(body, bg=CARD)
        row.pack(pady=6)
        ax = self.axes['body']
        scale = tk.Scale(row, from_=ax.hi, to=ax.lo, resolution=0.001,
                         orient=tk.VERTICAL, length=S(300), width=S(34), sliderlength=S(46),
                         showvalue=False, bg=CARD, fg=TEXT, troughcolor=TRACK,
                         activebackground=ACCENT_BODY, highlightthickness=0, bd=0,
                         command=lambda v: self.on_slider('body', v))
        scale.pack(side=tk.LEFT, padx=(0, 12))
        scale.set(0.0)
        ax.set_widget = scale.set
        btns = tk.Frame(row, bg=CARD)
        btns.pack(side=tk.LEFT)
        for text, cmd in (('▲  UP', lambda: self.nudge('body', +self.steps['body'])),
                          ('HOME', lambda: self.set_target('body', 0.0)),
                          ('▼  DOWN', lambda: self.nudge('body', -self.steps['body']))):
            tk.Button(btns, text=text, command=cmd, width=8, height=2, bg=CARD_HI, fg=TEXT,
                      activebackground=ACCENT_BODY, activeforeground='#000', bd=0,
                      font=(FONT, 10, 'bold'), cursor='hand2').pack(pady=6)
        ax.label = tk.Label(body, text='+0.000 m', bg=CARD, fg=TEXT, font=(FONT, 11, 'bold'))
        ax.label.pack(pady=(4, 0))

        # ----- right: knobs -----
        arms = self._card(main, 'SHOULDERS & ARMS', ACCENT)
        arms.grid(row=0, column=1, sticky='ns')
        grid = tk.Frame(arms, bg=CARD)
        grid.pack()
        for col, side in enumerate(('LEFT', 'RIGHT')):
            tk.Label(grid, text=side, bg=CARD, fg=TEXT,
                     font=(FONT, 11, 'bold')).grid(row=0, column=col + 1, pady=(0, 4))
        for r, (label, names) in enumerate((('Shoulder\n(Y axis)', ('l_sh', 'r_sh')),
                                            ('Arm\n(Z axis)', ('l_arm', 'r_arm')))):
            tk.Label(grid, text=label, bg=CARD, fg=SUBTEXT, font=(FONT, 10),
                     justify='right').grid(row=r + 1, column=0, padx=(0, 10))
            for c, name in enumerate(names):
                self._build_knob(grid, name).grid(row=r + 1, column=c + 1, padx=8, pady=6)

        # ----- bottom: toggles -----
        bar = tk.Frame(self.root, bg=CARD, padx=12, pady=10)
        bar.pack(fill='x', padx=14, pady=12)
        self.sync = {
            'shoulder': LedButton(bar, 'Sync shoulders', toggle=True, state=False),
            'arm': LedButton(bar, 'Sync arms', toggle=True, state=False),
        }
        self.mirror_arm = LedButton(bar, 'Mirror arms', toggle=True, state=True)
        home_btn = LedButton(bar, 'ALL HOME', toggle=False, command=self.home_all)
        for i, b in enumerate((self.sync['shoulder'], self.sync['arm'],
                               self.mirror_arm, home_btn)):
            bar.grid_columnconfigure(i, weight=1, uniform='btn')
            b.grid(row=0, column=i, padx=6, sticky='ew')

    def _build_knob(self, parent, name):
        ax = self.axes[name]
        cell = tk.Frame(parent, bg=CARD)
        knob = Knob(cell, ax.lo, ax.hi, 0.0, self.steps[name],
                    on_change=lambda v, n=name: self.set_target(n, v))
        knob.pack()
        ax.set_widget = knob.set
        ax.label = tk.Label(cell, text='+0.00 rad  |  +0°', bg=CARD, fg=TEXT,
                            font=(FONT, 10, 'bold'))
        ax.label.pack(pady=(2, 2))
        row = tk.Frame(cell, bg=CARD)
        row.pack()
        for text, fn in (('◀', lambda n=name: self.nudge(n, -self.steps[n])),
                         ('HOME', lambda n=name: self.set_target(n, 0.0)),
                         ('▶', lambda n=name: self.nudge(n, +self.steps[n]))):
            tk.Button(row, text=text, command=fn, width=4, bg=CARD_HI, fg=TEXT,
                      activebackground=ACCENT, activeforeground='#000', bd=0,
                      font=(FONT, 9, 'bold'), cursor='hand2').pack(side=tk.LEFT, padx=2)
        return cell

    # ---------------- logic ----------------
    def on_slider(self, name, v):
        if self._busy:
            return
        self.set_target(name, float(v))

    def set_target(self, name, v):
        ax = self.axes[name]
        v = ax.clamp(v)
        self._busy = True
        try:
            ax.target = v
            ax.set_widget(v)
            if name in self.PAIRS:
                partner, group = self.PAIRS[name]
                if self.sync[group].get():
                    pv = -v if (group == 'arm' and self.mirror_arm.get()) else v
                    other = self.axes[partner]
                    other.target = other.clamp(pv)
                    other.set_widget(other.target)
        finally:
            self._busy = False

    def nudge(self, name, d):
        self.set_target(name, self.axes[name].target + d)

    def home_all(self):
        for n in self.axes:
            self.set_target(n, 0.0)

    def tick(self):
        for ax in self.axes.values():
            ax.tick(self.dt)

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