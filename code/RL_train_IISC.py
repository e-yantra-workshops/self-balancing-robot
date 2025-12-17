import numpy as np
import pickle
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict
import time
import random 

# --- SETTINGS / HYPERPARAMS ---
def default_q_value():
    return 0.0   # set to 0.0 for neutral init, >0 for optimistic init

NUM_TILINGS = 8
actions = [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]

n_episodes = 1000
max_steps = 5000

batch_id = 1
batch_name = []
batch_name.append(f"batch{batch_id}")
D = {}                     # batch memory
batch_size = 35            # episodes per batch

target_alpha = 0.09    # base step-size (will be scaled by 1/NUM_TILINGS)
gamma = 0.99
epsilon = 1.0

# state continuous ranges
pos_space = np.linspace(-0.1, 0.1, 21)
pos_dot_space = np.linspace(-0.2, 0.2, 21)
tilt_space = np.linspace(-0.3, 0.3, 21)
tilt_dot_space = np.linspace(-0.6, 0.6, 21)

pos_prev, tilt_prev = 0.0, 0.0
prev_pos_cont, prev_tilt_cont = 0.0, 0.0

# --- GLOBALS ---
weights = defaultdict(float)           # w[(tile_id, action)] -> float
best_weights = None
elig_traces = defaultdict(float)       # e[(tile_id, action)]
tilecoder = None

# CoppeliaSim objects and variables (populated in sysCall_init)
sim = None
cart = None
wheel_l = None
wheel_r = None

episode = 0
step_count = 0
reward_sum = 0.0
training_complete = False
total_rewards = None
max_reward = -np.inf

# --- TileCoder class ---
class TileCoder:
    def __init__(self, dims_ranges, tiles_per_dim, num_tilings=8):
        self.dims = len(dims_ranges)
        self.ranges = dims_ranges
        self.tiles = np.array(tiles_per_dim, dtype=int)
        self.num_tilings = int(num_tilings)
        # widths (real units per tile interval)
        self.tile_width = (np.array([r[1] - r[0] for r in dims_ranges]) / self.tiles)
        # offsets for each tiling
        self.offsets = np.zeros((self.num_tilings, self.dims))
        for t in range(self.num_tilings):
            self.offsets[t] = (t / float(self.num_tilings)) * self.tile_width


    def get_tiles(self, state):
        s = np.array(state, dtype=float)
        ids = []
        for t in range(self.num_tilings):
            shifted = s + self.offsets[t] - np.array([r[0] for r in self.ranges])
            coords = np.floor(shifted / self.tile_width).astype(int)
            coords = np.minimum(np.maximum(coords, 0), self.tiles - 1)
            # include tiling index to avoid collisions
            tile_id = (t, int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3]))
            ids.append(tile_id)
        return ids

# continuous state reading
def get_state_continuous():
    global cart, pos_space, tilt_space, pos_dot_space, tilt_dot_space, pos_prev, tilt_prev
    pos = sim.getObjectPosition(cart, -1)[1]
    tilt = sim.getObjectOrientation(cart, -1)[0]
    pos_dot = pos - pos_prev
    tilt_dot = tilt - tilt_prev
    
    pos_prev = pos
    tilt_prev = tilt
    # clip to coder ranges
    pos = float(np.clip(pos, pos_space[0], pos_space[-1]))
    pos_dot = float(np.clip(pos_dot, pos_dot_space[0], pos_dot_space[-1]))
    tilt = float(np.clip(tilt, tilt_space[0], tilt_space[-1]))
    tilt_dot = float(np.clip(tilt_dot, tilt_dot_space[0], tilt_dot_space[-1]))
    return pos, tilt, pos_dot, tilt_dot

def get_active_features_from_state():
    pos, tilt, pos_dot, tilt_dot = get_state_continuous()
    return tilecoder.get_tiles([pos, tilt, pos_dot, tilt_dot])

# Q estimate for (tile set, action)
def q_from_tiles(tile_ids, action):
    # default per-tile weight used when no stored weight
    default_per_tile = default_q_value() / float(tilecoder.num_tilings)
    s = 0.0
    for t in tile_ids:
        s += weights.get((t, float(action)), default_per_tile)
    return s

# epsilon-greedy over actions given active tile ids
def choose_action(state_tiles, epsilon):
    global visit_bonus, visit_counts, beta, actions
    if np.random.rand() < epsilon:
        return np.random.choice(actions)

    q_values = [sum(weights[t, float(a)] for t in state_tiles) for a in actions]
    return actions[np.argmax(np.array(q_values))]

# apply motor action
def apply_action(action):
    a = float(action)
    sim.setJointTargetVelocity(wheel_l, a)
    sim.setJointTargetVelocity(wheel_r, a)

def reset_environment():
    sim.setObjectPosition(cart, -1, [0, 0, 0.0535])
    sim.setObjectOrientation(cart, -1, [0, 0, 0])
    sim.resetDynamicObject(cart)
    sim.setJointTargetVelocity(wheel_l, 0)
    sim.setJointTargetVelocity(wheel_r, 0)

# --- sysCall_init ---
def sysCall_init():
    global sim, cart, wheel_l, wheel_r, tilecoder, episode, step_count, reward_sum, total_rewards, weights, best_weights, elig_traces, epsilon

    sim = require('sim')
    globals()['sim'] = sim

    cart = sim.getObject('/body')
    wheel_r = sim.getObject('/right_joint')
    wheel_l = sim.getObject('/left_joint')
    D[batch_name[-1]] = []
    total_rewards = np.zeros(n_episodes)

    # instantiate tilecoder: use tiles_per_dim = intervals count (len-1)
    tilecoder = TileCoder(
        dims_ranges=[(pos_space[0], pos_space[-1]), (tilt_space[0], tilt_space[-1]), (pos_dot_space[0], pos_dot_space[-1]), (tilt_dot_space[0], tilt_dot_space[-1])],
        tiles_per_dim=[len(pos_space)-1, len(tilt_space)-1, len(pos_dot_space)-1, len(tilt_dot_space)-1],
        num_tilings=NUM_TILINGS
    )

    # init episode vars
    episode = 0
    step_count = 0
    reward_sum = 0.0
    elig_traces.clear()

    # initial state and action
    tiles = get_active_features_from_state()
    globals()['state_tiles'] = tiles
    globals()['action'] = choose_action(tiles, epsilon)

# --- sysCall_actuation ---
def sysCall_actuation():
    global action, step_count, training_complete
    if training_complete:
        return
    apply_action(action)
    step_count += 1

# --- sysCall_sensing ---
def sysCall_sensing():
    global D, weights, episode, step_count, reward_sum, total_rewards, max_reward, best_weights, training_complete
    global batch_size, state_tiles, action, prev_pos_cont, prev_tilt_cont, epsilon, visit_counts, n_episodes
    global batch_name, batch_id
    
    if training_complete:
        return

    pos_cont, tilt_cont = sim.getObjectPosition(cart, -1)[1], sim.getObjectOrientation(cart, -1)[0]
    pos_dot_cont = pos_cont - prev_pos_cont
    tilt_dot_cont = tilt_cont - prev_tilt_cont
    prev_pos_cont = pos_cont
    prev_tilt_cont = tilt_cont

    reward = (2.02 - (1.0*abs(tilt_cont)/0.3 + 1.0*abs(pos_cont)/0.1 + 0.01*abs(tilt_dot_cont)/0.6 + 0.01*abs(pos_dot_cont)/0.2))

    done = abs(pos_cont) > 0.1 or abs(tilt_cont) > 0.3 or step_count >= max_steps

    next_tiles = get_active_features_from_state()
    next_action = choose_action(next_tiles, epsilon)

    D[batch_name[-1]].append((state_tiles, action, reward, next_tiles, next_action, done))
    
    reward_sum += reward
    state_tiles = next_tiles
    action = next_action

    if done:
        total_rewards[episode] = reward_sum
        print(f"Episode {episode} reward: {reward_sum:.2f}")

        if total_rewards[episode] > max_reward:
            max_reward = total_rewards[episode]
            best_weights = weights.copy()

        episode += 1
        step_count = 0
        reward_sum = 0.0
        
        reset_environment()

        
        state_tiles = get_active_features_from_state()
        action = choose_action(state_tiles, epsilon)

        # Batch update every few episodes
        if episode % batch_size == 0 and len(D) > 0:
            batch_update()
            save_results()
            epsilon = max(0.05, epsilon*0.5)
            

        if episode >= n_episodes:
            training_complete = True
            save_results()


# --- save_results ---
def save_results():
    global weights, best_weights, total_rewards, sim
    try:
        path = sim.getStringParameter(sim.stringparam_scene_path)
    except Exception:
        path = os.getcwd()

    wpath = os.path.join(path, "q__weights.pkl")
    best_wpath = os.path.join(path, "q_best_weights.pkl")
    graph_path = os.path.join(path, "q_reward_graph.png")

    try:
        with open(wpath, "wb") as f:
            pickle.dump(dict(weights), f)
        print("Saved weights to", wpath)
        if best_weights is not None:
            with open(best_wpath, "wb") as f:
                pickle.dump(dict(best_weights), f)
            print("Saved best weights to", best_wpath)
    except Exception as e:
        print("Error saving weights:", e)

    try:
        avg_reward = np.convolve(total_rewards, np.ones(20)/20, mode='valid')
        plt.figure(figsize=(8,4))
        plt.plot(avg_reward)
        plt.title("Rewards (moving avg)")
        plt.xlabel("Episodes")
        plt.ylabel("Avg Reward (window=20)")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(graph_path)
        plt.close()
        print("Saved reward graph to", graph_path)
    except Exception as e:
        print("Error plotting/saving rewards:", e)
        
def batch_update():
    global weights, D, tilecoder, target_alpha, gamma, actions, batch_id
    
    if len(D) == 0:
        return

    k_replay = 500000   # total replay per batch
    per_tile_alpha = target_alpha / float(tilecoder.num_tilings)
    
    # -------- Perform replay --------
    for name in batch_name:
        batch = D[name]

        if len(batch) == 0:
            continue

        num_iters = k_replay
        print(per_tile_alpha)
        print(num_iters)
        
        for _ in range(num_iters):
            tiles, action, reward, next_tiles, _, done = random.choice(batch)
            Q_sa = q_from_tiles(tiles, action)

            if done:
                Q_next = 0.0
            else:
                Q_next = max(q_from_tiles(next_tiles, a_next) for a_next in actions)

            td_error = reward + gamma * Q_next - Q_sa

            for t in tiles:
                key = (t, float(action))
                weights[key] += per_tile_alpha * td_error

    print(f"[Batch Update] Completed prioritized replay over {len(D)} transitions.")

    # create new batch
    batch_id += 1
    batch_name.append(f"batch{batch_id}")
    D[batch_name[-1]] = []

# --- sysCall_cleanup ---
def sysCall_cleanup():
    print("script ended.")
