import time
import math
import itertools
from software.swarm.virtual_swarm import VirtualDrone, pairwise_distances

def minimum_pairwise_distance_original(drones):
    pairs = pairwise_distances(drones)
    if not pairs:
        return math.inf
    return min(dist for _, _, dist in pairs)

def pairwise_distances_generator(drones):
    fleet = list(drones)
    for i in range(len(fleet)):
        for j in range(i + 1, len(fleet)):
            dx = fleet[j].x - fleet[i].x
            dy = fleet[j].y - fleet[i].y
            dz = fleet[j].z - fleet[i].z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            yield (i, j, dist)

def minimum_pairwise_distance_generator(drones):
    return min((dist for _, _, dist in pairwise_distances_generator(drones)), default=math.inf)

def pairwise_distances_comb(drones):
    fleet = list(drones)
    for (i, a), (j, b) in itertools.combinations(enumerate(fleet), 2):
        dx = b.x - a.x
        dy = b.y - a.y
        dz = b.z - a.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        yield (i, j, dist)

def minimum_pairwise_distance_comb(drones):
    return min((dist for _, _, dist in pairwise_distances_comb(drones)), default=math.inf)

def pairwise_distances_comb_direct(drones):
    fleet = list(drones)
    for a, b in itertools.combinations(fleet, 2):
        dx = b.x - a.x
        dy = b.y - a.y
        dz = b.z - a.z
        yield math.sqrt(dx * dx + dy * dy + dz * dz)

def minimum_pairwise_distance_comb_direct(drones):
    return min(pairwise_distances_comb_direct(drones), default=math.inf)

drones = [VirtualDrone(drone_id=i, x=i*0.1, y=i*0.2, z=i*0.3) for i in range(100)]

def benchmark(func, drones, iterations=1000):
    start = time.perf_counter()
    for _ in range(iterations):
        func(drones)
    end = time.perf_counter()
    return end - start

print(f"Original: {benchmark(minimum_pairwise_distance_original, drones):.4f}s")
print(f"Generator: {benchmark(minimum_pairwise_distance_generator, drones):.4f}s")
print(f"Combinations: {benchmark(minimum_pairwise_distance_comb, drones):.4f}s")
print(f"Combinations direct: {benchmark(minimum_pairwise_distance_comb_direct, drones):.4f}s")
