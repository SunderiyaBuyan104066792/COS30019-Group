import os
import math
import heapq
import random
import xlrd
import numpy as np
import pandas as pd
import tensorflow as tf
from collections import deque
from sklearn.preprocessing import MinMaxScaler
from keras.models import load_model as _lm

from boroondara_graph import load_scats_data, build_graph
from travel_time import travel_time_seconds


class TrafficPredictor:
    def __init__(self, model_dir='model', data_file='data/Scats_Data_Oct_2006.xls', lag=12):
        self.model_dir = model_dir
        self.data_file = data_file
        self.lag = lag
        self._lstm_cache = {}
        self._gru_cache = {}
        self._dff_model = None
        self._dff_norm = None
        self._raw_data = None
    
    # data helpers
    
    def _load_raw(self):
        if self._raw_data is not None:
            return 
        wb = xlrd.open_workbook(self.data_file)
        sh = wb.sheet_by_name('Data')
        headers = sh.row_values(1)
        rows = [sh.row_values(i) for i in range(2, sh.nrows) if sh.row_values(i)[0]]
        self._raw_data = pd.DataFrame(rows, columns=headers)
        self._raw_data['SCATS Number'] = self._raw_data['SCATS Number'].apply(
            lambda x: int(float(str(x).strip()))
        )
        
    def _get_flow_series(self, scats_id):
        # Return the full flattened flow series for one site (all days, all slots)
        self._load_raw()
        df = self._raw_data[self._raw_data['SCATS Number'] == scats_id]
        cols = ['V%02d' % i for i in range(96)]
        
        return df[cols].values.flatten().astype(float)
        
    def _get_lag_window(self, scats_id, time_slot):
        # Return the 'lag' most-recent flow values ending at time_slot
        flow = self._get_flow_series(scats_id)
        end_idx = time_slot if time_slot > 0 else len(flow)
        window = flow[max(0, end_idx - self.lag):end_idx]
        if len(window) < self.lag:
            window = np.concatenate([np.zeros(self.lag - len(window)), window])
        return window.astype(float)
    
    def _fallback(self, scats_id):
        # Historical average flow, used when no model file is found
        flow = self._get_flow_series(scats_id)
        return float(np.mean(flow)) if len(flow) > 0 else 50.0
    
    
    # -- LSTM
    
    def _load_lstm(self, scats_id):
        if scats_id in self._lstm_cache:
            return self._lstm_cache[scats_id]
        path = os.path.join(self.model_dir, f'lstm_{scats_id}.h5')
        if not os.path.exists(path):
            return None
        model = _lm(path, compile=False)
        scaler = MinMaxScaler()
        scaler.fit(self._get_flow_series(scats_id).reshape(-1, 1))
        self._lstm_cache[scats_id]  = (model, scaler)
        return self. _lstm_cache[scats_id]
    
    def predict_lstm(self, scats_id: int, time_slot: int):
        # Predict traffic flow (veh/15min) using the LSTM model
        
        result = self._load_lstm(scats_id)
        if result is None:
            return self._fallback(scats_id)
        model, scaler = result
        window = self._get_lag_window(scats_id, time_slot)
        scaled = scaler.transform(window.reshape(-1, 1)).flatten()
        y_scaled = model.predict(scaled.reshape(1, self.lag, 1), verbose=0)
        return max(0.0, float(scaler.inverse_transform(y_scaled.reshape(-1, 1)).flatten()[0]))
    
    
    # -- GRU
    
    def _load_gru(self, scats_id):
        if scats_id in self._gru_cache:
            return self._gru_cache[scats_id]
        path = os.path.join(self.model_dir, f'gru_{scats_id}.h5')
        if not os.path.exists(path):
            return None
        model = _lm(path, compile=False)
        scaler = MinMaxScaler()
        scaler.fit(self._get_flow_series(scats_id).reshape(-1, 1))
        self._gru_cache[scats_id] = (model, scaler)
        return self._gru_cache[scats_id]
    
    def predict_gru(self, scats_id: int, time_slot: int):
        # Predict traffic flow (veh/15min) using the GRU model
        result = self._load_gru(scats_id)
        if result is None:
            return self._fallback(scats_id)
        model, scaler = result
        window = self._get_lag_window(scats_id, time_slot)
        scaled = scaler.transform(window.reshape(-1, 1)).flatten()
        y_scaled = model.predict(scaled.reshape(1, self.lag, 1), verbose=0)
        return max(0.0, float(scaler.inverse_transform(y_scaled.reshape(-1, 1)).flatten()[0]))
    
    
    # -- DFF
    
    def _load_dff(self):
        if self._dff_model is not None:
            return True
        model_path = os.path.join(self.model_dir, 'model.keras')
        norm_path = os.path.join(self.model_dir, 'norm.npz')
        if not os.path.exists(model_path) or not os.path.exists(norm_path):
            return False
        self._dff_model = tf.keras.models.load_model(model_path)
        self._dff_norm = np.load(norm_path)
        return True
    
    def predict_dff(self, scats_id: int, time_slot: int, day_of_week: int = 0):
        """
        Predict traffic flow (veh/15min) using the DFF model.

        - Parameters
        scats_id    : SCATS site number
        time_slot   : 0-95 (15-min intervals from midnight)
        day_of_week : 0=Monday ... 6=Sunday
        """
        
        if not self._load_dff():
            return self._fallback(scats_id)
        norm = self._dff_norm
        x_raw = np.array([[scats_id, day_of_week, time_slot]], dtype=np.float32)
        x_n = (x_raw - norm['x_mean']) / norm['x_std']
        window = self._get_lag_window(scats_id, time_slot).astype(np.float32)
        lags_n = ((window - norm['y_mean']) / norm['y_std']).reshape(1, self.lag, 1)
        y_n = self._dff_model.predict([x_n, lags_n], verbose=0)
        return max(0.0, float(y_n[0, 0]) * norm['y_std'] + norm['y_mean'])
    
    # Unified interface
    
    def predict(self, scats_id: int, time_slot: int, model_type: str = 'lstm', day_of_week: int = 0):
        """
        Predict traffic flow (veh/15min) for a SCATS site.

        - Parameters
        scats_id    : SCATS site number
        time_slot   : 0-95  (slot 0 = 00:00, slot 32 = 08:00)
        model_type  : 'lstm' | 'gru' | 'dff'
        day_of_week : 0=Monday ... 6=Sunday  (DFF only)
        """
        
        mt = model_type.lower()
        if mt == 'lstm':
            return self.predict_lstm(scats_id, time_slot)
        elif mt == 'gru':
            return self.predict_gru(scats_id, time_slot)
        elif mt == 'dff':
            return self.predict_dff(scats_id, time_slot, day_of_week)
        else: 
            raise ValueError(f"Unknown model_type '{model_type}'. Use 'lstm', 'gru', or 'dff'.")


# Dynamic Edge Costs

def build_dynamic_edges(edges, predictor: TrafficPredictor, model_type: str = 'lstm', 
                        time_slot: int = 32, day_of_week: int = 0):
    """
    Replace km distances with travel-time costs (seconds) using ML predictions.

    - Parameters
    edges       : {from_id: [(to_id, dist_km), ...]}  from build_graph()
    predictor   : TrafficPredictor instance
    model_type  : 'lstm' | 'gru' | 'dff'
    time_slot   : 0-95  which 15-min slot to predict for
    day_of_week : 0-6   (used by DFF only)

    - Returns
    dynamic_edges : {from_id: [(to_id, travel_time_seconds), ...]}
    """
    
    dynamic_edges = {}
    for from_id, neighbors in edges.items():
        dynamic_edges[from_id] = []
        for to_id, dist_km in neighbors:
            try: 
                flow = predictor.predict(from_id, time_slot, model_type, day_of_week)
            except Exception:
                flow = 50.0
            cost_s = travel_time_seconds(dist_km, flow, n_intersections=1)
            dynamic_edges[from_id].append((to_id, cost_s))
    return dynamic_edges


# Search node + helpers

class Node: 
    def __init__(self, state, parent=None, path_cost=0.0, depth=0, created_order=0):
        self.state = state
        self.parent = parent
        self.path_cost = path_cost
        self.depth = depth
        self.created_order = created_order

def build_path(goal_node):
    path, cur = [], goal_node
    while cur: 
        path.append(cur.state)
        cur = cur.parent
    path.reverse()
    return path

def straight_line_distance(a, b):
    return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)

def heuristic_distance(state, nodes, destinations):
    p = nodes[state]
    return min(straight_line_distance(p, nodes[g]) for g in destinations)


# Search Algorithms

def bfs(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    nc = 0
    root = Node(origin, None, 0.0, 0, nc)
    nc += 1
    frontier = deque([root])
    visited = {origin}
    while frontier:
        cur = frontier.popleft()
        if cur.state in destinations:
            return cur, nc
        for nxt, _ in edges.get(cur.state, []):
            if nxt not in visited:
                visited.add(nxt)
                frontier.append(Node(nxt, cur, 0.0, cur.depth+1, nc))
                nc += 1
    return None, nc

def dfs(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    nc = 0
    root = Node(origin, None, 0.0, 0, nc)
    nc += 1
    frontier = [root]
    visited = set()
    while frontier:
        cur = frontier.pop()
        if cur.state in visited:
            continue
        visited.add(cur.state)
        if cur.state in destinations:
            return cur, nc
        for nxt, _ in reversed(edges.get(cur.state, [])):
            if nxt not in visited:
                frontier.append(Node(nxt, cur, 0.0, cur.depth+1, nc))
                nc += 1
    return None, nc

def gbfs(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    nc = 0
    root = Node(origin, None, 0.0, 0, nc)
    nc += 1
    if root.state in destinations:
        return root, nc
    frontier = []
    visited = set()
    heapq.heappush(frontier, (heuristic_distance(origin, nodes, destinations), 
                            origin, root.created_order, root))
    while frontier:
        cur = heapq.heappop(frontier)[3]
        if cur.state in visited:
            continue
        visited.add(cur.state)
        if cur.state in destinations:
            return cur, nc
        for nxt, cost in edges.get(cur.state, []):
            if nxt not in visited:
                child = Node(nxt, cur, cur.path_cost+cost, cur.depth+1, nc)
                nc += 1
                heapq.heappush(frontier, (heuristic_distance(nxt, nodes, destinations),
                                        nxt, child.created_order, child))
    return None, nc

def astar(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    nc, ctr = 0, 0
    root = Node(origin, None, 0.0, 0, nc)
    nc += 1
    best_g = {origin: 0.0}
    frontier = [(heuristic_distance(origin, nodes, destinations), origin, ctr, root)]
    while frontier:
        _, _, _, cur = heapq.heappop(frontier)
        if cur.path_cost > best_g.get(cur.state, float('inf')):
            continue
        if cur.state in destinations:
            return cur, nc
        for nxt, cost in edges.get(cur.state, []):
            ng = cur.path_cost + cost
            if ng < best_g.get(nxt, float('inf')):
                best_g[nxt] = ng
                ctr += 1
                child = Node(nxt, cur, ng, cur.depth+1, ctr)
                nc += 1
                heapq.heappush(frontier, (ng + heuristic_distance(nxt, nodes, destinations),
                                        nxt, ctr, child))
    return None, nc

def dls(nodes, edges, origin, destinations, limit):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    CUTOFF = object()
    nc = 0
    root = Node(origin, None, 0.0, 0, nc)
    nc += 1
    if root.state in destinations:
        return root, nc
    
    def rdls(cur, visited, depth):
        nonlocal nc
        if cur.state in destinations:
            return cur
        if depth == 0:
            return CUTOFF
        cutoff_hit = False
        for nxt, _ in edges.get(cur.state, []):
            if nxt in visited:
                continue
            child = Node(nxt, cur, 0.0, cur.depth+1, nc)
            nc += 1
            visited.add(nxt)
            res = rdls(child, visited, depth-1)
            if res is CUTOFF:
                cutoff_hit = True
            elif res is not None:
                return res
            visited.remove(nxt)
        return CUTOFF if cutoff_hit else None
    
    res = rdls(root, {origin}, limit)
    if res is CUTOFF:
        return "CUTOFF", nc
    return res, nc

def _dijkstra_landmark(edges, landmark):
    ctr = 0
    root = Node(landmark, None, 0.0, 0, ctr)
    frontier = [(0.0, landmark, ctr, root)]
    best_g = {landmark: 0.0}
    dist = {}
    while frontier: 
        _, _, _, cur = heapq.heappop(frontier)
        if cur.path_cost > best_g.get(cur.state, float('inf')):
            continue
        dist[cur.state] = cur.path_cost
        for nxt, cost in edges.get(cur.state, []):
            ng = cur.path_cost + cost
            if ng < best_g.get(nxt, float('inf')):
                best_g[nxt] = ng
                ctr += 1
                heapq.heappush(frontier, (ng, nxt, ctr, Node(nxt, cur, ng, cur.depth+1, ctr)))
    return dist

def create_landmark_table(landmarks, edges):
    return {L: _dijkstra_landmark(edges, L) for L in landmarks}

def landmark_heuristic(state, destinations, landmarks, LM_Table):
    goals = []
    for goal in destinations:
        best = float('-inf')
        for L in landmarks:
            if state not in LM_Table[L] or goal not in LM_Table[L]:
                continue
            best = max(best, abs(LM_Table[L][state] - LM_Table[L][goal]))
        if best != float('-inf'):
            goals.append(best)
    return min(goals) if goals else 0

def a_landmark_triangle_inequality_search(nodes, edges, origin, destinations, landmarks=None):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    nc, ctr = 0, 0
    root = Node(origin, None, 0.0, 0, nc)
    nc += 1
    if not landmarks:
        k = min(16, max(2, int(len(nodes)**0.5)))
        landmarks = random.sample(list(nodes.keys()), min(k, len(nodes)))
    if root.state in destinations:
        return root, nc
    LM_Table = create_landmark_table(landmarks, edges)
    best_g = {origin: 0.0}
    frontier = [(landmark_heuristic(origin, destinations, landmarks, LM_Table),
                origin, ctr, root)]
    while frontier:
        _, _, _, cur = heapq.heappop(frontier)
        if cur.path_cost > best_g.get(cur.state, float('inf')):
            continue
        if cur.state in destinations:
            return cur, nc
        for nxt, cost in edges.get(cur.state, []):
            ng = cur.path_cost + cost
            if ng < best_g.get(nxt, float('inf')):
                best_g[nxt] = ng
                ctr += 1
                child = Node(nxt, cur, ng, cur.depth+1, ctr)
                nc += 1
                h = landmark_heuristic(nxt, destinations, landmarks, LM_Table)
                heapq.heappush(frontier, (ng+h, nxt, ctr, child))
    return None, nc

# Top-k Paths (k-shortest loopless paths)

def _dijkstra(edges, origin, destinations):
    best_g = {origin: 0.0}
    frontier = [(0.0, origin, [origin])]
    while frontier:
        cost, u, path = heapq.heappop(frontier)
        if cost > best_g.get(u, float('inf')):
            continue
        if u == destinations:
            return cost, path
        for v, w in edges.get(u, []):
            nc = cost + w
            if nc < best_g.get(v, float('inf')):
                best_g[v] = nc
                heapq.heappush(frontier, (nc, v, path+[v]))
    return float('inf'), []

def top_k_paths(nodes, edges, origin, destination, k=5):
    """
    Returns list of (total_cost_seconds, path_list) sorted cheapest first.
    May return fewer than k entries if the graph has fewer distinct paths.
    """
    
    cost, path = _dijkstra(edges, origin, destination)
    if not path:
        return []
    
    A = [(cost, path)]
    B = []
    seen_B = set()
    
    for _ in range(1, k):
        _, prev_path = A[-1]
        
        for i in range(len(prev_path) - 1):
            spur_node = prev_path[i]
            root_path = prev_path[:i+1]
            
            root_cost = 0.0
            for j in range(len(root_path)-1):
                u, v = root_path[j], root_path[j+1]
                for nb, w in edges.get(u, []):
                    if nb == v:
                        root_cost += w
                        break
            
            blocked_edges = {}
            for _, path_a in A:
                if path_a[:i+1] == root_path and len(path_a) > i+1:
                    blocked_edges.setdefault(path_a[i], set()).add(path_a[i+1])
            blocked_nodes = set(root_path[:-1])
            
            best_spur = {spur_node: 0.0}
            heap = [(0.0, spur_node, [spur_node])]
            while heap:
                sc, u, sp = heapq.heappop(heap)
                if sc > best_spur.get(u, float('inf')):
                    continue
                if u == destination:
                    full_path = root_path[:-1] + sp
                    full_cost = root_cost + sc
                    key = tuple(full_path)
                    if key not in seen_B and not any(p == full_path for _, p in A):
                        seen_B.add(key)
                        heapq.heappush(B, (full_cost, full_path))
                    break
                for v, w in edges.get(u, []):
                    if v in blocked_nodes:
                        continue
                    if v in blocked_edges.get(u, set()):
                        continue
                    nc = sc + w
                    if nc < best_spur.get(v, float('inf')):
                        best_spur[v] = nc
                        heapq.heappush(heap, (nc, v, sp+[v]))
        
        if not B:
            break
        best_cost, best_path = heapq.heappop(B)
        A.append((best_cost, best_path))
    
    return A

def save_path_to_txt(paths, scats_sites=None, output_file='result.txt'):
    lines = format_paths(paths, scats_sites)
    with open(output_file, 'w') as f:
        for line in lines:
            f.write(line + '\n')
    print(f"Result saved to {output_file}")


# Display Helpers

def format_paths(paths, scats_sites=None):
    lines = []
    for rank, (cost_s, path) in enumerate(paths, 1):
        mins = int(cost_s // 60)
        secs = int(cost_s % 60)
        if scats_sites:
            stops = ' -> '.join(
                f"{sid} ({scats_sites[sid][2].split()[0]})" for sid in path
            )
        else:
            stops = ' -> '.join(str(sid) for sid in path)
        lines.append(f"Route {rank}: {mins}m {secs:02d}s  |  {stops}")
    return lines

def time_to_slot(hour: int, minute: int = 0):
    # Convert a clock time to a 0-95 time slot index
    return (hour * 60 + minute) // 15


# Smoke Test

if __name__ == '__main__':
    DATA = 'data/Scats_Data_Oct_2006.xls'
    
    sites = load_scats_data(DATA)
    nodes, edges = build_graph(sites)
    predictor = TrafficPredictor(model_dir='model', data_file=DATA)
    
    hour, minute = 5, 30
    origin, destination = 3180, 4262
    slot = time_to_slot(hour, minute)
    
    for model_type in ['lstm', 'gru', 'dff']:
        print(f"\n{model_type.upper()} (slot {slot} = {hour}:{minute:02d})")
        dyn = build_dynamic_edges(edges, predictor, model_type=model_type, time_slot=slot)
        paths = top_k_paths(nodes, dyn, origin, destination, k=5)
        save_path_to_txt(paths, sites, output_file=f'result_{model_type}.txt')
        if not paths:
            print("  No path found.")
        else: 
            for line in format_paths(paths, sites):
                print(' ', line)