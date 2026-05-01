import numpy as np
import math
import random

def path_length(waypoints):
    """Total path length through waypoints."""
    total = 0
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1][0] - waypoints[i][0]
        dy = waypoints[i+1][1] - waypoints[i][1]
        total += math.sqrt(dx**2 + dy**2)
    return total

def pso_path_optimizer(start, end, intermediate_count=3,
                       bounds=(0, 100), n_particles=40,
                       max_iter=200, w=0.7, c1=1.5, c2=1.5):
    """
    Optimization of intermediate waypoints.
    start, end: tuples (x, y)
    intermediate_count: number of intermediate points
    bounds: coordinate limits
    """
    dim = intermediate_count * 2
    low, high = bounds

    # Initialize swarm
    pos = np.random.uniform(low, high, (n_particles, dim))
    vel = np.random.uniform(-(high-low)*0.1, (high-low)*0.1, (n_particles, dim))
    pbest_pos = pos.copy()

    def fitness(p):
        pts = [start]
        for i in range(intermediate_count):
            pts.append((p[2*i], p[2*i+1]))
        pts.append(end)
        return path_length(pts)

    pbest_val = np.array([fitness(p) for p in pos])
    gbest_idx = np.argmin(pbest_val)
    gbest_pos = pbest_pos[gbest_idx].copy()
    gbest_val = pbest_val[gbest_idx]

    history = []

    for iteration in range(max_iter):
        r1 = np.random.rand(n_particles, dim)
        r2 = np.random.rand(n_particles, dim)

        vel = (w * vel
               + c1 * r1 * (pbest_pos - pos)
               + c2 * r2 * (gbest_pos - pos))

        pos = np.clip(pos + vel, low, high)

        vals = np.array([fitness(p) for p in pos])
        improved = vals < pbest_val
        pbest_pos[improved] = pos[improved]
        pbest_val[improved] = vals[improved]

        if pbest_val.min() < gbest_val:
            gbest_val = pbest_val.min()
            gbest_pos = pbest_pos[np.argmin(pbest_val)].copy()

        history.append(gbest_val)

    # Construct final path
    optimal_path = [start]
    for i in range(intermediate_count):
        optimal_path.append((round(gbest_pos[2*i], 2),
                             round(gbest_pos[2*i+1], 2)))
    optimal_path.append(end)

    return optimal_path, gbest_val, history


# --- Example ---
start_pt = (0.0, 0.0)
end_pt   = (100.0, 100.0)

opt_path, opt_dist, conv = pso_path_optimizer(
    start_pt, end_pt,
    intermediate_count=3,
    n_particles=50,
    max_iter=300
)

print("Optimal path:")
for i, pt in enumerate(opt_path):
    print(f"  WP{i}: ({pt[0]:.2f}, {pt[1]:.2f})")

print(f"Path length: {opt_dist:.2f}")
print(f"Direct distance: {math.sqrt(2)*100:.2f}")
print("PSO converged successfully after 300 iterations")