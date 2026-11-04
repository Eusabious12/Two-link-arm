import math

L1 = 1.0  # length of stick 1
L2 = 1.0  # length of stick 2


def forward(theta1_deg, theta2_deg):
    """Angles in degrees in, hand position (x, y) out."""
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


import matplotlib.pyplot as plt


def plot_arm(theta1_deg, theta2_deg, ax, label):
    t1 = math.radians(theta1_deg)
    elbow = (L1 * math.cos(t1), L1 * math.sin(t1))
    hand = forward(theta1_deg, theta2_deg)
    ax.plot([0, elbow[0], hand[0]], [0, elbow[1], hand[1]],
            "o-", linewidth=4, label=label)

def inverse(x, y, elbow_up=True):
    """Target (x, y) in, angles in degrees out. Returns None if unreachable."""
    d_squared = x * x + y * y

    # How bent the elbow must be (law of cosines)
    c = (d_squared - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if c < -1 or c > 1:
        return None  # too far or too close, no solution

    t2 = math.acos(c)
    if not elbow_up:
        t2 = -t2

    # Shoulder angle: direction to target, minus the correction from the elbow
    t1 = math.atan2(y, x) - math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    return math.degrees(t1), math.degrees(t2)


# Check: go to a target, then run forward kinematics to see where we landed
for target in [(1, 1), (0.5, 1.2), (1.5, 0.5), (3, 3)]:
    for up in (True, False):
        angles = inverse(*target, elbow_up=up)
        if angles is None:
            print(target, "unreachable")
        else:
            landed = forward(*angles)
            print(target, "elbow_up" if up else "elbow_down",
                  "angles:", [round(a, 1) for a in angles],
                  "landed at:", [round(v, 2) for v in landed])
            
def show_both(x, y):
    fig, ax = plt.subplots()
    for bend_plus, color in [(True, "tab:blue"), (False, "tab:orange")]:
        angles = inverse(x, y, elbow_up=bend_plus)
        if angles is None:
            print("unreachable")
            return
        t1 = math.radians(angles[0])
        elbow = (L1 * math.cos(t1), L1 * math.sin(t1))
        hand = forward(*angles)
        name = "bend +" if bend_plus else "bend -"
        ax.plot([0, elbow[0], hand[0]], [0, elbow[1], hand[1]],
                "o-", linewidth=4, color=color,
                label=f"{name}: {angles[0]:.0f}, {angles[1]:.0f}")
    ax.plot(x, y, "rx", markersize=14, label="target")
    ax.axhline(0, color="black")
    ax.set_aspect("equal")
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-1.5, 2.5)
    ax.legend(title="theta1, theta2")
    plt.show()


show_both(1.5, 0.5)

def forward_rad(t1, t2):
    return (L1 * math.cos(t1) + L2 * math.cos(t1 + t2),
            L1 * math.sin(t1) + L2 * math.sin(t1 + t2))


def iterative(xt, yt, t1=math.radians(80), t2=math.radians(40),
              gain=0.1, max_steps=300):
    """Start from a pose and nudge the joints toward the target."""
    history = []
    for _ in range(max_steps):
        x, y = forward_rad(t1, t2)
        history.append((t1, t2, x, y))

        ex, ey = xt - x, yt - y          # error: how far the hand is from target
        if ex * ex + ey * ey < 1e-8:     # close enough, stop
            break

        # Jacobian: how the hand moves when each joint turns a tiny bit
        b = t1 + t2
        j11 = -L1 * math.sin(t1) - L2 * math.sin(b)
        j12 = -L2 * math.sin(b)
        j21 = L1 * math.cos(t1) + L2 * math.cos(b)
        j22 = L2 * math.cos(b)

        # Nudge each joint in the direction that reduces the error
        t1 += gain * (j11 * ex + j21 * ey)
        t2 += gain * (j12 * ex + j22 * ey)
    return history


def show_iterative(xt, yt):
    history = iterative(xt, yt)
    fig, ax = plt.subplots()

    trail_x = [h[2] for h in history]
    trail_y = [h[3] for h in history]
    ax.plot(trail_x, trail_y, ".", color="gray", label="hand path")

    # draw the arm at the start, middle and end of the journey
    picks = [0, len(history) // 2, len(history) - 1]
    for idx, color in zip(picks, ["tab:orange", "tab:green", "tab:blue"]):
        t1, t2, x, y = history[idx]
        elbow = (L1 * math.cos(t1), L1 * math.sin(t1))
        ax.plot([0, elbow[0], x], [0, elbow[1], y], "o-",
                linewidth=3, color=color, label=f"step {idx}")

    ax.plot(xt, yt, "rx", markersize=14, label="target")
    ax.axhline(0, color="black")
    ax.set_aspect("equal")
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-1.5, 2.5)
    ax.legend()
    final = history[-1]
    print("steps used:", len(history), "final hand:",
          round(final[2], 3), round(final[3], 3))
    plt.show()


show_iterative(1.5, 0.5)