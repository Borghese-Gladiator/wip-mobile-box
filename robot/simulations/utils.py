"""Shared visualization helpers for all simulation scripts."""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def make_fig(title: str):
    """Create a standard dark 8x8 axes. Returns (fig, ax)."""
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=13)
    return fig, ax


def make_robot_artists(ax, *, with_ekf: bool = False):
    """
    Add the standard robot artists to ax.

    Returns a dict with keys:
      robot_body, heading_line, phone_dot
      robot_est_dot  (only when with_ekf=True)
    """
    import matplotlib.pyplot as plt  # needed for Circle

    robot_body = plt.Circle((-5, 0), 0.4, color="orange", zorder=5)
    ax.add_patch(robot_body)

    heading_line, = ax.plot([], [], color="orange", linewidth=2, zorder=6)
    phone_dot,    = ax.plot([], [], "o", color="yellow", markersize=10,
                             zorder=7, label="Phone")

    artists = dict(robot_body=robot_body, heading_line=heading_line,
                   phone_dot=phone_dot)

    if with_ekf:
        robot_est_dot, = ax.plot([], [], "o", color="cyan", markersize=6,
                                  zorder=7, label="EKF estimate")
        artists["robot_est_dot"] = robot_est_dot

    ax.legend(loc="upper right", fontsize=9)
    return artists


def draw_fading_trail(ax, trail, color):
    """Scatter a trail list of [x, y, ...] arrays with increasing alpha."""
    n = len(trail)
    for i, p in enumerate(trail):
        ax.scatter(p[0], p[1], s=4, color=color, alpha=i / max(n, 1))


def clear_collections(ax):
    """Remove all scatter collections from ax (call before redrawing trails)."""
    for coll in ax.collections[:]:
        coll.remove()


def run(animate_fn, dt: float):
    """Wire up FuncAnimation and block on plt.show()."""
    fig = plt.gcf()
    FuncAnimation(fig, animate_fn, interval=int(dt * 1000),
                  blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
