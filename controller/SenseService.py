import SharedData
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


gaps_dist = 1.5
total_point = 0
mean_scale = 20
range_max = 5
resolution = 0.05  # 5 cm per cella
map_size = 20.0  # 10 x 10 metri
grid_size = int(map_size / resolution)
origin = grid_size // 2
grid = np.zeros((grid_size, grid_size))
theta_r = 0
x_r = 0
y_r = 0

angle_max = 2 * np.pi
angle_increment = 0
angles = 0

prev_vector = []

def get_lidar_data(sim,function,script_handle):
    return sim.callScriptFunction(function, script_handle)

def get_total_point(sim,function,script_handle):
    global total_point, angle_increment, angles, prev_vector
    prev_vector = get_lidar_data(sim,function,script_handle)
    while len(prev_vector) == 0:
        prev_vector = get_lidar_data(sim, function, script_handle)
    total_point = len(prev_vector)
    angle_increment = angle_max / total_point
    angles = np.arange(total_point) * angle_increment

def sense_loop():
    global x_r,y_r,theta_r
    client = RemoteAPIClient()
    sim = client.require('sim')
    shared = SharedData.get_shared()

    function = 'scanCallBack'
    lidar = ':/fastHokuyo'
    base_link = sim.getObject(':/base_link_respondable')

    while sim.getSimulationState() == 0:
        time.sleep(0.1)

    script_handle = sim.getScript(sim.scripttype_simulation, lidar)

    get_total_point(sim,function,script_handle)

    m = sim.getObjectMatrix(base_link, -1)
    x_r, y_r = m[3], m[7]
    theta_r = math.atan2(-m[6], -m[2])
    shared["position"] = (x_r, y_r, theta_r)

    shared["setup"] += [True]
    print("[SenseService] avviato")
    while True:
        #print("sensing")
        global prev_vector
        distance_vector = get_lidar_data(sim,function,script_handle)
        m = sim.getObjectMatrix(base_link, -1)
        x_r, y_r = m[3], m[7]
        theta_r = math.atan2(-m[6], -m[2])
        shared["position"] = (x_r, y_r, theta_r)
        #fill_map(pos, prev_vector, distance_vector)
        gaps, closest = find_gaps(distance_vector)
        gap_len, gap_best_dir, gap_distance = get_best_dir(gaps,distance_vector)
        prev_vector = distance_vector
        shared["map"] = grid
        shared["sensor_data"] = {
            "total_point": total_point,
            "gap_len": gap_len,
            "best_dir": gap_best_dir,
            "gap_distance": gap_distance,
            "closest": closest
        }
        if shared["objective"] != None and len(shared["objective"]) > 0:
            desire = get_desire_dir(shared["objective"][0], shared["position"])
            desire = adjust_desire(desire,distance_vector)
            if shared["desire"] != None:
                delta_t = abs(shared["desire"][0]-desire[0])
                delta_d = abs(shared["desire"][1]-desire[1])
                if delta_t < 50 or delta_d > 1:
                    shared["desire"] = desire
            else:
                shared["desire"] = desire
        time.sleep(0.1)

def adjust_desire(desire,distance_vector):
    dt, dist = desire
    vector_check = int(((dt+math.pi)/(2*math.pi))*total_point)
    max_dist = distance_vector[vector_check]
    max_n = vector_check
    if max_dist >= dist:
        return max_n, dist
    for i in range(int(total_point/3)):
        check_right = (vector_check+i)%total_point
        check_left = (vector_check-i)%total_point
        if max_dist < distance_vector[check_right]:
            max_dist = distance_vector[check_right]
            max_n = check_right
        if max_dist < distance_vector[check_left]:
            max_dist = distance_vector[check_left]
            max_n = check_left
        if max_dist >= dist:
            break
    return max_n, dist


def get_desire_dir(block, pos):
    x_c, y_c = block[1], block[2]
    x_r, y_r, t_r = pos
    dx = x_c - x_r
    dy = y_c - y_r
    dist = (dx ** 2 + dy ** 2) ** (1 / 2)
    t_target = math.atan2(dy, dx)
    delta_t = t_target - t_r
    delta_t = (delta_t + math.pi) % (2 * math.pi) - math.pi
    return delta_t, dist

def find_gaps(lidar_vector):
    gaps = []
    gaps_count = -1
    rays_count = 0
    closest = [gaps_dist,0]
    gap = False
    for dist in lidar_vector:
        if total_point / 8 < rays_count < total_point / 8 * 7:
            if dist < closest[0]:
                closest[0] = dist
                closest[1] = rays_count
        if total_point / 4 < rays_count < total_point / 4 * 3:
            if dist > gaps_dist:
                if not gap:
                    gap = True
                    gaps.append([])
                    gaps_count = gaps_count + 1
                gaps[gaps_count].append(rays_count)
            elif gap:
                gap = False
        rays_count = rays_count + 1
    return gaps, closest

def get_best_dir(gaps, distance_vector):
    gap_len = []
    gap_best_dir = []
    gap_distance = []
    for i in range(len(gaps)):
        gap_len.append(len(gaps[i]))
        gap_info = get_best_dir_x_gap(gaps[i],distance_vector)
        gap_best_dir.append(gap_info[0])
        gap_distance.append(gap_info[1])
    return gap_len, gap_best_dir, gap_distance


def get_best_dir_x_gap(gap, distance_vector):
    sum_vector = [0]
    mean_vector = [0]
    max_mean = 0
    best_dir = 0
    i = 0
    for ray in gap:
        if i == 0:
            sum_vector[i] = distance_vector[ray]
        else:
            sum_vector.append(sum_vector[i-1] + distance_vector[ray])
        i = i + 1
    for i in range(len(gap)):
        if i < mean_scale and i < len(gap) - mean_scale:
            mean_vector.append(sum_vector[i + mean_scale] - sum_vector[0])
        elif len(gap) - mean_scale < i and mean_scale < i:
            mean_vector.append(sum_vector[len(sum_vector)-1] - sum_vector[i - mean_scale - 1])
        elif mean_scale < i < len(gap) - mean_scale:
            mean_vector.append(sum_vector[i + mean_scale] - sum_vector[i - mean_scale - 1])
        else:
            mean_vector.append(0)
        if max_mean < mean_vector[i]:
            max_mean = mean_vector[i]
            best_dir = gap[i]
    safe_dir = gap[int(len(gap)/2)]
    return best_dir, max_mean, safe_dir

def fill_map(pos, prev_vector, curr_vector):
    global grid, theta_r, x_r, y_r

    # 1. Estrae punti locali da entrambe le scansioni
    p_prev = ranges_to_points(prev_vector, angle_max)
    p_curr = ranges_to_points(curr_vector, angle_max)

    dx, dy, dtheta = pos
    #dx, dy, dtheta = scan_match(p_prev, p_curr)

    # 3. Aggiorna posa stimata globale
    x_r += dx * np.cos(theta_r) - dy * np.sin(theta_r)
    y_r += dx * np.sin(theta_r) + dy * np.cos(theta_r)
    theta_r += dtheta
    theta_r = (theta_r + np.pi) % (2 * np.pi) - np.pi

    # 4. Trasforma i punti della scansione corrente in coordinate globali
    angles = np.linspace(0, angle_max, len(curr_vector), endpoint=False)
    r = np.array(curr_vector)

    # Filtra i raggi max
    valid = r < range_max
    r = r[valid]
    angles = angles[valid]

    x_local = r * np.cos(angles)
    y_local = r * np.sin(angles)

    # Trasformazione in globale
    x_global = x_r + x_local * np.cos(theta_r) - y_local * np.sin(theta_r)
    y_global = y_r + x_local * np.sin(theta_r) + y_local * np.cos(theta_r)

    # 5. Aggiorna la mappa
    for xg, yg, r in zip(x_global, y_global, curr_vector):
        if r >= range_max:
            continue
        gx = int(origin + xg / resolution)
        gy = int(origin + yg / resolution)
        if 0 <= gx < grid_size and 0 <= gy < grid_size:
            grid[gy, gx] = 1


def ranges_to_points(ranges, angle_max, angle_min=0, max_range=None):
    angles = np.linspace(angle_min, angle_max, len(ranges), endpoint=False)
    r = np.array(ranges)

    # Filtra i punti troppo lontani (range massimo)
    if max_range is not None:
        mask = r < max_range
        r = r[mask]
        angles = angles[mask]

    x = r * np.cos(angles)
    y = r * np.sin(angles)
    points = np.vstack((x, y)).T   # shape (N, 2)

    return points


def scan_match(P_prev, P_curr, max_iters=40, tolerance=1e-4):
    """
    Allinea P_curr a P_prev; restituisce trasformazione (R,t) e posa incrementale (dx,dy,dtheta)
    """
    N_prev = compute_normals(P_prev)
    T_total = np.eye(3)
    P = P_curr.copy()
    angles = np.arctan2(P[:, 1], P[:, 0])
    order = np.argsort(angles)
    P = P[order]
    prev_error = float('inf')

    for it in range(max_iters):
        # trova corrispondenze più vicine
        tree = cKDTree(P_prev)
        dists, idxs = tree.query(P)
        Q = P_prev[idxs]
        N = N_prev[idxs]

        # costruisci sistema lineare per incremento (dx, dy, dtheta)
        A = []
        b = []
        for p, q, n in zip(P, Q, N):
            r = p - q
            # Jacobiano: [n^T, n^T * J * p]
            # con J = [[0, -1], [1, 0]]
            J_theta = np.array([n[1] * p[0] - n[0] * p[1]])
            A.append([n[0], n[1], J_theta[0]])
            b.append(-np.dot(n, r))

        A = np.array(A)
        b = np.array(b)

        # risolvi least squares
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
        dx, dy, dtheta = x

        # aggiorna trasformazione
        c = np.cos(dtheta)
        s = np.sin(dtheta)
        R_inc = np.array([[c, -s], [s, c]])
        t_inc = np.array([dx, dy])

        P = (R_inc @ P.T).T + t_inc

        # accumula trasformazione globale
        T_inc = np.eye(3)
        T_inc[:2, :2] = R_inc
        T_inc[:2, 2] = t_inc
        T_total = T_inc @ T_total

        mean_error = np.mean(np.abs(b))
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # estrai parametri finali
    dx = T_total[0, 2]
    dy = T_total[1, 2]
    dtheta = np.arctan2(T_total[1, 0], T_total[0, 0])

    return dx, dy, dtheta

def compute_normals(points):
    """Stima normali 2D tramite differenze finite."""
    n = len(points)
    normals = np.zeros_like(points)
    for i in range(n):
        p_prev = points[(i - 1) % n]
        p_next = points[(i + 1) % n]
        tangent = p_next - p_prev
        normal = np.array([-tangent[1], tangent[0]])  # rotazione 90°
        normal /= np.linalg.norm(normal) + 1e-8
        normals[i] = normal
    return normals

def show_map():
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.title('Mappa 2D da LIDAR 360°')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.show()



