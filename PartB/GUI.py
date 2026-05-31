import os
import sys
import tkinter as tk
from tkinter import ttk
import tkintermapview
import pandas as pd
import json
import requests
import threading
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ROUTE_COLORS = ['#e05252', '#5299e0', '#52e08a', '#e0b452', '#b452e0', '#52d4e0']
ALGORITHMS = ['AS', 'BFS', 'DFS', 'GBFS', 'DLS', 'ALT']
MODELS = ['lstm', 'gru', 'custom']


class PathfinderApp:
    def __init__(self, root):
        self.root = root
        root.title('SCATS Pathfinder')
        root.geometry('1300x800')

        self.edit_mode = None
        self.first_node = None
        self.connections = {}
        self.path_lines = {}
        self.route_lines = []
        self._drawn_routes = []
        self.node_coordinates = {}
        self.node_roads = {}

        self._build_sidebar(root)
        self._build_map(root)
        self.load_data()

    # ── Sidebar ────────────────────────────────────────────────────

    def _build_sidebar(self, root):
        outer = tk.Frame(root, width=280, bg='#2b2b2b')
        outer.pack(side='left', fill='y')
        outer.pack_propagate(False)

        canvas = tk.Canvas(outer, bg='#2b2b2b', highlightthickness=0)
        scrollbar = tk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)

        sidebar = tk.Frame(canvas, bg='#2b2b2b')
        canvas.create_window((0, 0), window=sidebar, anchor='nw')
        sidebar.bind('<Configure>', lambda _: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<MouseWheel>', lambda e: canvas.yview_scroll(-1 if e.delta > 0 else 1, 'units'))

        tk.Label(sidebar, text='SCATS Pathfinder', font=('Helvetica', 13, 'bold'),
                 bg='#2b2b2b', fg='white').pack(anchor='w', padx=12, pady=10)

        self._build_search(sidebar)
        self._divider(sidebar)
        self._build_pathfinder(sidebar)
        self._divider(sidebar)
        self._build_connections(sidebar)
        self._divider(sidebar)

        self.status_label = tk.Label(sidebar, text='', bg='#2b2b2b', fg='#ffaa00',
                                     font=('Helvetica', 9), wraplength=240, justify='left')
        self.status_label.pack(anchor='w', padx=12, pady=4)

    def _divider(self, parent):
        tk.Frame(parent, bg='#444444', height=1).pack(fill='x', padx=12, pady=6)

    def _build_search(self, sidebar):
        tk.Label(sidebar, text='Search nodes', bg='#2b2b2b', fg='#aaaaaa').pack(
            anchor='w', padx=12, pady=(4, 2))
        self.search_var = tk.StringVar()
        self.search_var.trace_add('write', self.on_search_typed)
        tk.Entry(sidebar, textvariable=self.search_var, width=28).pack(padx=12, pady=2)
        self.search_results = tk.Listbox(sidebar, height=4, width=28)
        self.search_results.pack(padx=12, pady=2)
        self.search_results.bind('<<ListboxSelect>>', self.on_result_selected)

    def _build_pathfinder(self, sidebar):
        tk.Label(sidebar, text='Find Routes', bg='#2b2b2b', fg='#aaaaaa').pack(
            anchor='w', padx=12, pady=(4, 2))

        def combo(label, attr, options, default):
            f = tk.Frame(sidebar, bg='#2b2b2b')
            f.pack(fill='x', padx=12, pady=1)
            tk.Label(f, text=label, bg='#2b2b2b', fg='white', width=12, anchor='w').pack(side='left')
            var = tk.StringVar(value=default)
            setattr(self, attr, var)
            cb = ttk.Combobox(f, textvariable=var, values=options, state='readonly', width=10)
            cb.pack(side='left')
            return cb

        def spin(label, attr, lo, hi, default):
            f = tk.Frame(sidebar, bg='#2b2b2b')
            f.pack(fill='x', padx=12, pady=1)
            tk.Label(f, text=label, bg='#2b2b2b', fg='white', width=12, anchor='w').pack(side='left')
            var = tk.StringVar(value=default)
            setattr(self, attr, var)
            tk.Spinbox(f, from_=lo, to=hi, textvariable=var, width=6).pack(side='left')

        self._origin_cb = combo('Origin', 'pf_origin_var', [], '')
        self._dest_cb = combo('Destination', 'pf_dest_var', [], '')
        combo('Algorithm', 'pf_method_var', ALGORITHMS, 'AS')
        combo('Model', 'pf_model_var', MODELS, 'lstm')
        spin('Day (1–30)', 'pf_day_var', 1, 30, '4')
        spin('Hour (0–23)', 'pf_hour_var', 0, 23, '8')
        spin('Minute', 'pf_min_var', 0, 59, '0')
        spin('Routes (k)', 'pf_k_var', 1, 6, '3')

        self.find_btn = tk.Button(sidebar, text='Find Routes', width=22,
                                  command=self.on_find_routes, state='disabled')
        self.find_btn.pack(padx=12, pady=(6, 2))

        tk.Button(sidebar, text='Clear Routes', width=22,
                  command=self.clear_route_lines).pack(padx=12, pady=2)

        self.route_list = tk.Listbox(sidebar, height=6, width=28, font=('Courier', 9))
        self.route_list.pack(padx=12, pady=4)
        self.route_list.bind('<<ListboxSelect>>', self.on_route_selected)

    def _build_connections(self, sidebar):
        tk.Label(sidebar, text='Edit connections', bg='#2b2b2b', fg='#aaaaaa').pack(
            anchor='w', padx=12, pady=(4, 2))
        tk.Button(sidebar, text='Connect Nodes', width=22,
                  command=self.start_connect_mode).pack(padx=12, pady=2)
        tk.Button(sidebar, text='Remove Connection', width=22,
                  command=self.start_remove_mode).pack(padx=12, pady=2)
        tk.Button(sidebar, text='Clear All Connections', width=22, fg='#ff6666',
                  command=self.clear_all_connections).pack(padx=12, pady=2)

    def _build_map(self, root):
        self.map_widget = tkintermapview.TkinterMapView(root)
        self.map_widget.pack(side='right', fill='both', expand=True)
        self.map_widget.set_position(-37.8136, 144.9631)
        self.map_widget.set_zoom(11)

    # ── Data loading ───────────────────────────────────────────────

    def load_data(self):
        script_folder = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_folder, 'graph', 'site_road_data', 'road_data.csv')
        df = pd.read_csv(csv_path)

        scats_ids = []
        for _, row in df.iterrows():
            node_id = int(row['scats_num'])
            self.node_coordinates[node_id] = (float(row['latitude']), float(row['longitude']))
            self.node_roads[node_id] = str(row['roads']).replace('_', ' ')
            scats_ids.append(str(node_id))

        for node_id, (lat, lon) in self.node_coordinates.items():
            def on_click(_, clicked_id=node_id):
                self.on_node_clicked(clicked_id)
            self.map_widget.set_marker(lat, lon, text=str(node_id), command=on_click)

        self._origin_cb.config(values=scats_ids)
        self._dest_cb.config(values=scats_ids)
        self.pf_origin_var.set(scats_ids[0] if scats_ids else '')
        self.pf_dest_var.set(scats_ids[-1] if scats_ids else '')

        self.find_btn.config(state='normal')
        self.status_label.config(text=f'Loaded {len(self.node_coordinates)} nodes.')

    # ── OSRM ──────────────────────────────────────────────────────

    def get_road_route(self, lat1, lon1, lat2, lon2):
        url = (
            f'http://router.project-osrm.org/route/v1/driving/'
            f'{lon1},{lat1};{lon2},{lat2}'
            f'?overview=full&geometries=geojson'
        )
        try:
            response = requests.get(url, timeout=10)
            data = response.json()
            route_data = data['routes'][0]
            distance_km = route_data['distance'] / 1000
            route_points = [(lat, lon) for lon, lat in route_data['geometry']['coordinates']]
            return distance_km, route_points
        except Exception:
            return None, [(lat1, lon1), (lat2, lon2)]

    # ── Pathfinder ────────────────────────────────────────────────

    def on_find_routes(self):
        try:
            origin = int(self.pf_origin_var.get())
            dest = int(self.pf_dest_var.get())
            depart = datetime(2006, 11, int(self.pf_day_var.get()),
                              int(self.pf_hour_var.get()), int(self.pf_min_var.get()))
            k = int(self.pf_k_var.get())
        except ValueError as e:
            self.status_label.config(text=f'Input error: {e}')
            return

        model = self.pf_model_var.get()
        method = self.pf_method_var.get()

        self.find_btn.config(state='disabled')
        self.clear_route_lines()
        self.route_list.delete(0, 'end')
        self.status_label.config(text='Searching…')

        def worker():
            try:
                from pathFinder import find_routes
                routes = find_routes(origin, dest, depart,
                                     model_name=model, method=method, k=k)
                drawn = []
                for route in routes:
                    all_points = []
                    path = route['path']
                    for i in range(len(path) - 1):
                        la1, lo1 = self.node_coordinates[path[i]]
                        la2, lo2 = self.node_coordinates[path[i + 1]]
                        _, pts = self.get_road_route(la1, lo1, la2, lo2)
                        all_points.extend(pts)
                    drawn.append({'route': route, 'points': all_points})
                self.root.after(0, lambda: self._on_routes_found(drawn))
            except Exception as e:
                self.root.after(0, lambda: self.status_label.config(text=f'Error: {e}'))
                self.root.after(0, lambda: self.find_btn.config(state='normal'))

        threading.Thread(target=worker, daemon=True).start()

    def _on_routes_found(self, drawn):
        self.find_btn.config(state='normal')
        self._drawn_routes = drawn

        if not drawn:
            self.status_label.config(text='No routes found.')
            return

        for i, d in enumerate(drawn):
            color = ROUTE_COLORS[i % len(ROUTE_COLORS)]
            line = self.map_widget.set_path(d['points'], color=color, width=4)
            self.route_lines.append(line)
            m, sec = divmod(int(d['route']['cost_seconds']), 60)
            self.route_list.insert('end', f'  {i+1}.  {m} min {sec} sec')
            self.route_list.itemconfig(i, foreground=color)

        self.route_list.selection_set(0)
        self.status_label.config(text=f'{len(drawn)} route(s) found.')

    def on_route_selected(self, *_):
        sel = self.route_list.curselection()
        if not sel or not self._drawn_routes:
            return
        idx = sel[0]
        for line in self.route_lines:
            line.delete()
        self.route_lines.clear()
        for i, d in enumerate(self._drawn_routes):
            color = ROUTE_COLORS[i % len(ROUTE_COLORS)]
            width = 5 if i == idx else 2
            draw_color = color if i == idx else '#444466'
            line = self.map_widget.set_path(d['points'], color=draw_color, width=width)
            self.route_lines.append(line)

    def clear_route_lines(self):
        for line in self.route_lines:
            line.delete()
        self.route_lines.clear()
        self._drawn_routes = []
        self.route_list.delete(0, 'end')

    # ── Connection editing ────────────────────────────────────────

    def save_connections(self):
        script_folder = os.path.dirname(os.path.abspath(__file__))
        save_path = os.path.join(script_folder, 'graph', 'site_road_data', 'connections.json')
        seen = set()
        records = []
        for node_a, neighbours in self.connections.items():
            for node_b, distance_km, route_points in neighbours:
                pair = (min(node_a, node_b), max(node_a, node_b))
                if pair not in seen:
                    seen.add(pair)
                    records.append({'nodes': list(pair), 'distance_km': distance_km, 'route': route_points})
        with open(save_path, 'w') as f:
            json.dump(records, f)

    def on_search_typed(self, *_):
        typed = self.search_var.get().lower()
        self.search_results.delete(0, 'end')
        if not typed:
            return
        for node_id, roads in self.node_roads.items():
            if typed in roads.lower() or typed in str(node_id):
                self.search_results.insert('end', f'{node_id} - {roads}')

    def on_result_selected(self, *_):
        selected = self.search_results.curselection()
        if not selected:
            return
        node_id = int(self.search_results.get(selected[0]).split(' - ', 1)[0])
        lat, lon = self.node_coordinates[node_id]
        self.map_widget.set_position(lat, lon)
        self.map_widget.set_zoom(15)
        self.status_label.config(text=f'Jumped to node {node_id}')

    def on_node_clicked(self, node_id):
        if self.edit_mode == 'connect':
            self.handle_connect(node_id)
        elif self.edit_mode == 'remove':
            self.handle_remove(node_id)
        else:
            lat, lon = self.node_coordinates[node_id]
            self.status_label.config(text=f'Node {node_id}  ({lat:.4f}, {lon:.4f})')

    def start_connect_mode(self):
        self.edit_mode = 'connect'
        self.first_node = None
        self.status_label.config(text='Connect: click the first node.')

    def handle_connect(self, node_id):
        if self.first_node is None:
            self.first_node = node_id
            self.status_label.config(text=f'Connect: {node_id} selected — now click the second node.')
            return

        first, second = self.first_node, node_id
        self.edit_mode = None
        self.first_node = None

        if first == second:
            self.status_label.config(text='Same node clicked twice — nothing added.')
            return

        pair = (min(first, second), max(first, second))
        if pair in self.path_lines:
            self.status_label.config(text=f'{first} ↔ {second} already connected.')
            return

        lat1, lon1 = self.node_coordinates[first]
        lat2, lon2 = self.node_coordinates[second]
        distance_km, route_points = self.get_road_route(lat1, lon1, lat2, lon2)

        self.connections.setdefault(first, []).append((second, distance_km, route_points))
        self.connections.setdefault(second, []).append((first, distance_km, route_points))

        line = self.map_widget.set_path(route_points, color='#888888', width=6)
        self.path_lines[pair] = line

        self.save_connections()
        self.status_label.config(text=f'Connected {first} ↔ {second}  ({distance_km:.2f} km) — saved.')

    def start_remove_mode(self):
        self.edit_mode = 'remove'
        self.first_node = None
        self.status_label.config(text='Remove: click the first node.')

    def handle_remove(self, node_id):
        if self.first_node is None:
            self.first_node = node_id
            self.status_label.config(text=f'Remove: {node_id} selected — now click the second node.')
            return

        first, second = self.first_node, node_id
        self.edit_mode = None
        self.first_node = None

        pair = (min(first, second), max(first, second))
        if pair not in self.path_lines:
            self.status_label.config(text=f'No connection between {first} and {second}.')
            return

        self.path_lines[pair].delete()
        del self.path_lines[pair]

        self.connections[first] = [(n, d, r) for n, d, r in self.connections.get(first, []) if n != second]
        self.connections[second] = [(n, d, r) for n, d, r in self.connections.get(second, []) if n != first]

        self.save_connections()
        self.status_label.config(text=f'Removed connection {first} ↔ {second} — saved.')

    def clear_all_connections(self):
        for line in self.path_lines.values():
            line.delete()
        self.path_lines.clear()
        self.connections.clear()
        self.save_connections()
        self.status_label.config(text='All connections cleared.')


root = tk.Tk()
app = PathfinderApp(root)
root.mainloop()
