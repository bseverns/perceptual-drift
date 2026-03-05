import time
import math
import itertools
from software.swarm.virtual_swarm import VirtualDrone, minimum_pairwise_distance, pairwise_distances

def minimum_pairwise_distance_original(drones):
    fleet = list(drones)
    pairs = []
    for i in range(len(fleet)):
        for j in range(i + 1, len(fleet)):
            dx = fleet[j].x - fleet[i].x
            dy = fleet[j].y - fleet[i].y
            dz = fleet[j].z - fleet[i].z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            pairs.append((i, j, dist))
    if not pairs:
        return math.inf
    return min(dist for _, _, dist in pairs)

drones = [VirtualDrone(drone_id=i, x=i*0.1, y=i*0.2, z=i*0.3) for i in range(100)]

def benchmark(func, drones, iterations=1000):
    start = time.perf_counter()
    for _ in range(iterations):
        func(drones)
    end = time.perf_counter()
    return end - start

baseline = benchmark(minimum_pairwise_distance_original, drones)
new = benchmark(minimum_pairwise_distance, drones)
print(f"Original: {baseline:.4f}s")
print(f"Optimized: {new:.4f}s")
print(f"Improvement: {(baseline - new) / baseline * 100:.2f}%")
