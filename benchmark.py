import time
import math
import itertools
from software.swarm.virtual_swarm import VirtualDrone, pairwise_distances

def benchmark(func, drones, iterations=1000):
    start = time.perf_counter()
    for _ in range(iterations):
        list(func(drones))
    end = time.perf_counter()
    return end - start

def pairwise_distances_generator(drones):
    fleet = list(drones)
    for i in range(len(fleet)):
        for j in range(i + 1, len(fleet)):
            dx = fleet[j].x - fleet[i].x
            dy = fleet[j].y - fleet[i].y
            dz = fleet[j].z - fleet[i].z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            yield (i, j, dist)

def pairwise_distances_combinations(drones):
    fleet = list(drones)
    for (i, d1), (j, d2) in itertools.combinations(enumerate(fleet), 2):
        dx = d2.x - d1.x
        dy = d2.y - d1.y
        dz = d2.z - d1.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        yield (i, j, dist)

drones = [VirtualDrone(drone_id=i, x=i*0.1, y=i*0.2, z=i*0.3) for i in range(100)]

time_original = benchmark(pairwise_distances, drones)
print(f"Original: {time_original:.4f}s")

time_gen = benchmark(pairwise_distances_generator, drones)
print(f"Generator: {time_gen:.4f}s")

time_comb = benchmark(pairwise_distances_combinations, drones)
print(f"Combinations: {time_comb:.4f}s")
