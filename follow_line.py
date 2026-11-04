import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

L1 = 1.0           # link lengths in metres
L2 = 1.0
DT = 0.05          # seconds per frame
FRAMES = 240       # 12 seconds of driving
SPEED = 1.0        # truck speed in m/s


def line_y(x):
    """The line we want to paint: a gentle curve at about 1.2 m from the truck."""
    return 1.2 + 0.25 * math.sin(0.8 * x)


def truck_position(t):
    """Truck drives forward and wobbles sideways by up to 8 cm."""
    return SPEED * t, 0.08 * math.sin(2.5 * t)


def hand_position(t1, t2):
    return (L1 * math.cos(t1) + L2 * math.cos(t1 + t2),
            L1 * math.sin(t1) + L2 * math.sin(t1 + t2))


def nudge(t1, t2, xt, yt, gain=0.2, iterations=10):
    """A few small iterative steps toward the target (same idea as step 3)."""
    for _ in range(iterations):
        x, y = hand_position(t1, t2)
        ex, ey = xt - x, yt - y
        b = t1 + t2
        j11 = -L1 * math.sin(t1) - L2 * math.sin(b)
        j12 = -L2 * math.sin(b)
        j21 = L1 * math.cos(t1) + L2 * math.cos(b)
        j22 = L2 * math.cos(b)
        t1 += gain * (j11 * ex + j21 * ey)
        t2 += gain * (j12 * ex + j22 * ey)
    return t1, t2


# ---- 1. Simulate the whole drive first ----
t1, t2 = math.radians(24), math.radians(103)   # a sensible starting pose
frames = []
errors = []

for i in range(FRAMES):
    t = i * DT
    bx, by = truck_position(t)                 # arm base moves with the truck

    # target point on the line, 0.3 m ahead of the truck (world coordinates)
    tx = bx + 0.3
    ty = line_y(tx)

    # the arm works in its own frame: target relative to the base
    t1, t2 = nudge(t1, t2, tx - bx, ty - by)

    hx, hy = hand_position(t1, t2)
    nozzle = (bx + hx, by + hy)
    elbow = (bx + L1 * math.cos(t1), by + L1 * math.sin(t1))

    errors.append(math.hypot(nozzle[0] - tx, nozzle[1] - ty))
    frames.append((bx, by, elbow, nozzle, (tx, ty)))

print(f"max error: {max(errors) * 100:.1f} cm")
print(f"average error: {sum(errors) / len(errors) * 100:.1f} cm")

# ---- 2. Animate it ----
fig, ax = plt.subplots(figsize=(10, 4))
ax.set_aspect("equal")
ax.set_ylim(-1.5, 2.2)

xs = [i * 0.05 for i in range(300)]
ax.plot(xs, [line_y(x) for x in xs], "--", color="gray", label="line to paint")
paint, = ax.plot([], [], "-", color="tab:orange", linewidth=4, label="paint")
arm, = ax.plot([], [], "o-", color="tab:blue", linewidth=4)
target_dot, = ax.plot([], [], "rx", markersize=10, label="target")
truck = Rectangle((0, 0), 1.6, 0.9, color="gray", alpha=0.6)
ax.add_patch(truck)
ax.legend(loc="lower right")

painted_x, painted_y = [], []


def update(i):
    bx, by, elbow, nozzle, target = frames[i]
    painted_x.append(nozzle[0])
    painted_y.append(nozzle[1])
    arm.set_data([bx, elbow[0], nozzle[0]], [by, elbow[1], nozzle[1]])
    paint.set_data(painted_x, painted_y)
    target_dot.set_data([target[0]], [target[1]])
    truck.set_xy((bx - 0.8, by - 0.45))
    ax.set_xlim(bx - 2, bx + 6)
    return arm, paint, target_dot, truck


anim = FuncAnimation(fig, update, frames=FRAMES, interval=40, repeat=False)
anim.save("demo.gif", writer="pillow", fps=20)
print("saved demo.gif")
plt.show()