import os
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import tkintermapview
from datetime import datetime
from pathFinder import find_routes
import pandas as pd
import json
import requests


# colours for each route - the selected route uses its colour, others are drawn dimmed grey
route_colours = [
    '#e05252', '#5299e0', '#52e08a', '#e0b452',
    '#b452e0', '#52d4e0', '#e07a52',
]


class PathfinderApp:

    def __init__(self, root):
        self.root = root
        root.title("SCATS Pathfinder")
        root.geometry("1200x900")

        self._init_state()
        self._build_sidebar()
        self._build_map()
        self.load_data()

    # ------------------------------------------------------------------ #
    #____________________Initialisation helpers__________________________#


    def _init_state(self):
        # cache for OSRM road segments - maps (lat1,lon1,lat2,lon2) -> (distance_km, points)
        self._osrm_cache = {}
        # list of path objects currently drawn on the map so we can delete them when redrawing
        self._map_paths = []
        # list of gathered road point lists, one per found route
        self._route_points = []
        # the last set of routes returned by find_routes
        self._routes = []

        # tracks what mode the user is in and which node they clicked first
        self.edit_mode = None   # 'connect', 'remove', or None
        self.first_node = None  # first node clicked, waiting for second
        self.connections = {}   # node_id -> [(neighbour_id, distance_km, route_points), ..]
        self.node_coordinates = {}
        self.node_roads = {}

        # using StringVar() so widgets and trace callbacks share the same value
        # https://www.askpython.com/python-modules/tkinter/stringvar-with-examples
        self.search_var = tk.StringVar()
        self.start_search_var = tk.StringVar()
        self.end_search_var = tk.StringVar()
        self.chosen_model_var = tk.StringVar()
        self.chosen_algorithmn_var = tk.StringVar()
        self.day_var = tk.StringVar(value='1')
        self.hour_var = tk.StringVar(value='8')
        self.min_var = tk.StringVar(value='0')
        self.k_var = tk.StringVar(value='3')

        # we use trace to track variables
        # https://www.geeksforgeeks.org/python/tracing-tkinter-variables-in-python/
        self.search_var.trace_add('write', self.search_changed)
        self.start_search_var.trace_add('write', self.start_changed)
        self.end_search_var.trace_add('write', self.end_changed)

    def _build_sidebar(self):
        sidebar = tk.Frame(self.root, width=280, bg='#2b2b2b')
        sidebar.pack(side='left', fill='y')
        sidebar.pack_propagate(False)

        tk.Label(
            sidebar,
            text='SCATS Pathfinder',
            font=('Helvetica', 14, 'bold'),
            bg='#2b2b2b',
            fg='white',
        ).pack(anchor='w', padx=12, pady=10)

        # Status label - shows messages to the user
        self.status_label = tk.Label(
            sidebar,
            text='',
            bg='#2b2b2b',
            fg='#ffaa00',
            font=('Helvetica', 9),
            wraplength=250,
            justify='left',
        )
        self.status_label.pack(anchor='w', padx=12, pady=4)

        tk.Frame(sidebar, bg='#444444', height=1).pack(fill='x', padx=12, pady=4)

        # Start location search
        self._section_label(sidebar, 'Starting Location')
        tk.Entry(sidebar, textvariable=self.start_search_var, width=28).pack(padx=12, pady=2)
        self.start_search_results = tk.Listbox(
            sidebar, exportselection=False, height=4, width=28,
            bg='#1e1e1e', fg='white', selectbackground='#3a5a8a',
            relief='flat', borderwidth=0,
        )
        self.start_search_results.pack(padx=12, pady=2)
        self.start_search_results.bind('<<ListboxSelect>>', self.on_result_selected)

        # End location search
        self._section_label(sidebar, 'End Location')
        tk.Entry(sidebar, textvariable=self.end_search_var, width=28).pack(padx=12, pady=2)
        self.end_search_results = tk.Listbox(
            sidebar, exportselection=False, height=4, width=28,
            bg='#1e1e1e', fg='white', selectbackground='#3a5a8a',
            relief='flat', borderwidth=0,
        )
        self.end_search_results.pack(padx=12, pady=2)
        self.end_search_results.bind('<<ListboxSelect>>', self.on_result_selected)

        # Model and algorithm dropdowns
        self._section_label(sidebar, 'Select Model')
        self.chosen_model = ttk.Combobox(sidebar, width=27, textvariable=self.chosen_model_var, state='readonly')
        self.chosen_model['values'] = ('lstm', 'gru', 'custom')
        self.chosen_model.pack(padx=12, pady=2)

        self._section_label(sidebar, 'Select Search Algorithmn')
        self.chosen_algorithmn = ttk.Combobox(sidebar, width=27, textvariable=self.chosen_algorithmn_var, state='readonly')
        self.chosen_algorithmn['values'] = ('BFS', 'DFS', 'GBFS', 'AS', 'DLS', 'ALT')
        self.chosen_algorithmn.pack(padx=12, pady=2)

        # using Spinbox instead of Entry for day/hour/min/paths so the widget
        # enforces the allowed range and we don't need manual validation methods
        self._section_label(sidebar, 'Select Day (1-30)')
        ttk.Spinbox(sidebar, from_=1, to=30, textvariable=self.day_var, width=27).pack(padx=12, pady=2)

        self._section_label(sidebar, 'Select Hour (0-23)')
        ttk.Spinbox(sidebar, from_=0, to=23, textvariable=self.hour_var, width=27).pack(padx=12, pady=2)

        self._section_label(sidebar, 'Select Minute (0-59)')
        ttk.Spinbox(sidebar, from_=0, to=59, textvariable=self.min_var, width=27).pack(padx=12, pady=2)

        self._section_label(sidebar, 'Select amount of paths')
        ttk.Spinbox(sidebar, from_=1, to=10, textvariable=self.k_var, width=27).pack(padx=12, pady=2)

        # store as instance variable so submit() can disable it while a search is running
        self.find_btn = tk.Button(sidebar, text='Find Path', width=22, fg='#228B22', command=self.submit)
        self.find_btn.pack(padx=12, pady=(15, 6))

        # Route results list - shows each found route with its travel time coloured to match
        # its line on the map; clicking a row highlights that route and dims all others
        tk.Frame(sidebar, bg='#444444', height=1).pack(fill='x', padx=12, pady=4)
        tk.Label(sidebar, text='Routes  (click to highlight)', bg='#2b2b2b', fg='#aaaaaa').pack(anchor='w', padx=12, pady=(4, 2))
        self.route_list = tk.Listbox(
            sidebar, height=5, width=28,
            bg='#1a1a1a', fg='white', selectbackground='#2a2a2a',
            relief='flat', borderwidth=0, font=('Courier', 9),
        )
        self.route_list.pack(padx=12, pady=2, fill='x')
        self.route_list.bind('<<ListboxSelect>>', self._on_route_select)

    def _build_map(self):
        self.map_widget = tkintermapview.TkinterMapView(self.root)
        self.map_widget.pack(side='right', fill='both', expand=True)
        self.map_widget.set_position(-37.8136, 144.9631)
        self.map_widget.set_zoom(11)

    def _section_label(self, parent, text):
        tk.Label(parent, text=text, bg='#2b2b2b', fg='#aaaaaa').pack(anchor='w', padx=12, pady=(4, 2))

    # ------------------------------------------------------------------ #
    #_______________________Data Loading ________________________________#

    def load_data(self):
        script_folder = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_folder, 'graph', 'site_road_data', 'road_data.csv')
        df = pd.read_csv(csv_path)

        for _, row in df.iterrows():
            node_id = int(row['scats_num'])
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            roads = str(row['roads']).replace('_', ' ')
            self.node_coordinates[node_id] = (lat, lon)
            self.node_roads[node_id] = roads

        for node_id, (lat, lon) in self.node_coordinates.items():
            # clicked_id=node_id captures this node's id for each marker
            # without it every marker would report the same (last) node_id
            def on_click(marker, clicked_id=node_id):
                self.on_node_clicked(clicked_id)
            self.map_widget.set_marker(lat, lon, text=str(node_id), command=on_click)

        self.status_label.config(text=f'Loaded {len(self.node_coordinates)} nodes.')

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
                    records.append({
                        'nodes': list(pair),
                        'distance_km': distance_km,
                        'route': route_points,
                    })

        with open(save_path, 'w') as f:
            json.dump(records, f)


    # ------------------------------------------------------------------ #
    #____________________ OSRM road routing __________________________#

    def get_road_route(self, lat1, lon1, lat2, lon2):
        # cache OSRM results so the same road segment is never fetched twice -
        # this makes repeated or overlapping route draws much faster
        key = (round(lat1, 5), round(lon1, 5), round(lat2, 5), round(lon2, 5))
        if key in self._osrm_cache:
            return self._osrm_cache[key]

        # OSRM needs lon, lat order (opposite to how we store in road_data.csv)
        url = (
            f'http://router.project-osrm.org/route/v1/driving/'
            f'{lon1},{lat1};{lon2},{lat2}'
            f'?overview=full&geometries=geojson'
        )
        response = requests.get(url, timeout=10)
        data = response.json()

        route_data = data['routes'][0]
        distance_km = route_data['distance'] / 1000
        # flip each point from [lon, lat] to (lat, lon) for tkintermapview
        route_points = [(lat, lon) for lon, lat in route_data['geometry']['coordinates']]

        result = (distance_km, route_points)
        self._osrm_cache[key] = result
        return result


    # ------------------------------------------------------------------ #
    #____________________Search box - filtering__________________________#

    def on_search_typed(self, *args, search_type):
        match search_type:
            case "start":
                typed = self.start_search_var.get().lower()
                self.start_search_results.delete(0, 'end')
                if not typed:
                    return
                for node_id, roads in self.node_roads.items():
                    if typed in roads.lower() or typed in str(node_id):
                        self.start_search_results.insert('end', f'{node_id} - {roads}')
            case "end":
                typed = self.end_search_var.get().lower()
                self.end_search_results.delete(0, 'end')
                if not typed:
                    return
                for node_id, roads in self.node_roads.items():
                    if typed in roads.lower() or typed in str(node_id):
                        self.end_search_results.insert('end', f'{node_id} - {roads}')

    def search_changed(self, *args):
        self.on_search_typed(*args, search_type="search")

    def start_changed(self, *args):
        self.on_search_typed(*args, search_type="start")

    def end_changed(self, *args):
        self.on_search_typed(*args, search_type="end")

    # ------------------------------------------------------ #
    #____________Route finding - for selected values_________#

    def get_selected(self, lb):
        selected = lb.curselection()
        if selected:
            return lb.get(selected[0])
        return None

    def submit(self, *args):
        Loc_Start = self.get_selected(self.start_search_results)
        Loc_End = self.get_selected(self.end_search_results)
        selected_model = self.chosen_model.get().strip()
        selected_algo = self.chosen_algorithmn.get().strip()

        if not all([Loc_Start, Loc_End, selected_model, selected_algo]):
            self.show_error("Missing or invalid Start, End, Model or Algorithmn")
            return

        # spinboxes enforce the range so we just need to read the values
        try:
            selected_day = int(self.day_var.get())
            selected_hour = int(self.hour_var.get())
            selected_min = int(self.min_var.get())
            selected_no_path = int(self.k_var.get())
        except ValueError:
            self.show_error("Invalid time or path count value")
            return

        origin = int(Loc_Start.split(" - ")[0])
        destination = int(Loc_End.split(" - ")[0])
        departure_time = datetime(2006, 11, selected_day, selected_hour, selected_min, 0)

        self.find_btn.config(state='disabled')
        self.status_label.config(text='Searching...')
        self._clear_map_routes()

        # run find_routes in a background thread so the UI stays responsive
        # self.root.after() schedules UI updates back on the main thread -
        # tkinter is not thread-safe so we cannot call UI methods directly from the thread
        def run():
            try:
                routes = find_routes(origin, destination, departure_time, selected_model, selected_algo, selected_no_path)
                self.root.after(0, lambda: self._on_routes_found(routes))
            except Exception as e:
                self.root.after(0, lambda: self._on_error(str(e)))

        threading.Thread(target=run, daemon=True).start()

    def _on_routes_found(self, routes):
        # called back on the main thread once find_routes finishes
        self.find_btn.config(state='normal')
        self._routes = routes
        self.route_list.delete(0, 'end')

        if not routes:
            self.status_label.config(text='No routes found.')
            return

        # populate the route list - each route gets its own colour matching its line on the map
        for i, route in enumerate(routes):
            color = route_colours[i % len(route_colours)]
            cost = route.get('cost_seconds', 0)
            m, s = divmod(int(cost), 60)
            self.route_list.insert('end', f'  {i + 1}.  {m} min {s} sec')
            self.route_list.itemconfig(i, foreground=color)

        self.status_label.config(text=f'{len(routes)} route(s) found. Drawing roads...')

        # OSRM calls can be slow (one HTTP request per road segment between nodes),
        # so gather all road points in another background thread to keep the UI unblocked
        def gather():
            collected = [self._gather_route_points(r['path']) for r in routes]
            self.root.after(0, lambda: self._on_points_ready(collected))

        threading.Thread(target=gather, daemon=True).start()


    def _gather_route_points(self, path):
        # build the full list of road-following coordinates for one route
        # by stitching together the OSRM segment for each consecutive pair of nodes
        full_route = []
        for j in range(len(path) - 1):
            n1, n2 = path[j], path[j + 1]
            lat1, lon1 = self.node_coordinates[n1]
            lat2, lon2 = self.node_coordinates[n2]
            _, segment = self.get_road_route(lat1, lon1, lat2, lon2)
            if j > 0:
                segment = segment[1:]  # avoid duplicates
            full_route.extend(segment)
        return full_route


    def _on_points_ready(self, collected):
        # called back on the main thread once all OSRM points are gathered
        self._route_points = collected
        self._redraw_routes(selected=0)
        self.route_list.selection_set(0)
        self.status_label.config(text=f'{len(self._routes)} route(s) found.')

    def _redraw_routes(self, selected=0):
        for path_obj in self._map_paths:
            try:
                path_obj.delete()
            except Exception:
                pass
        self._map_paths.clear()

        # draw non-selected routes first (dimmed grey) so they sit underneath the selected one
        for i, pts in enumerate(self._route_points):
            if i == selected or not pts:
                continue
            path_obj = self.map_widget.set_path(pts, color='#555555', width=2)
            self._map_paths.append(path_obj)

        # draw the selected route on top in its colour with a thicker line
        if selected < len(self._route_points) and self._route_points[selected]:
            color = route_colours[selected % len(route_colours)]
            path_obj = self.map_widget.set_path(self._route_points[selected], color=color, width=5)
            self._map_paths.append(path_obj)

    def _on_route_select(self, event):
        # called when the user clicks a route in the route results list
        sel = self.route_list.curselection()
        if not sel or not self._route_points:
            return
        self._redraw_routes(selected=sel[0])

    def _clear_map_routes(self):
        # remove all drawn routes from the map and reset route state before a new search
        for path_obj in self._map_paths:
            try:
                path_obj.delete()
            except Exception:
                pass
        self._map_paths.clear()
        self._route_points.clear()
        self._routes.clear()
        self.route_list.delete(0, 'end')

    # ------------------------------------------------------------------ #
    #_________________Node selection and connections_________________#

    def on_result_selected(self, event):
        listbox = event.widget
        selected = listbox.curselection()
        if not selected:
            return
        # split on ' - ' and take the first part to get the node ID
        # maxsplit=1 means we only split on the first ' - ' in case road names contain hyphens
        node_id = int(listbox.get(selected[0]).split(' - ', 1)[0])
        lat, lon = self.node_coordinates[node_id]
        self.map_widget.set_position(lat, lon)
        self.map_widget.set_zoom(15)
        self.status_label.config(text=f'Jumped to node {node_id}')

    def on_node_clicked(self, node_id):
        # called whenever a marker is clicked on the map
        if self.edit_mode == 'connect':
            self.handle_connect(node_id)
        else:
            lat, lon = self.node_coordinates[node_id]
            self.status_label.config(text=f'Node {node_id}  ({lat:.4f}, {lon:.4f})')

    def handle_connect(self, node_id):
        # first click - remember this node and wait for the second
        if self.first_node is None:
            self.first_node = node_id
            self.status_label.config(text=f'Connect: {node_id} selected — now click the SECOND node.')
            return

        # second click - we have both nodes
        first = self.first_node
        second = node_id
        self.edit_mode = None
        self.first_node = None

        if first == second:
            self.status_label.config(text='Same node clicked twice — nothing added.')
            return

        lat1, lon1 = self.node_coordinates[first]
        lat2, lon2 = self.node_coordinates[second]
        distance_km, route_points = self.get_road_route(lat1, lon1, lat2, lon2)

        self.connections.setdefault(first, []).append((second, distance_km, route_points))
        self.connections.setdefault(second, []).append((first, distance_km, route_points))
        self.map_widget.set_path(route_points, color='#888888', width=6)
        self.save_connections()
        self.status_label.config(text=f'Connected {first} ↔ {second}  ({distance_km:.2f} km) — saved.')

    def start_connect_mode(self):
        self.edit_mode = 'connect'
        self.first_node = None
        self.status_label.config(text='Connect: click the FIRST node.')

    def start_remove_mode(self):
        self.status_label.config(text='Remove: click the FIRST node.')

    def clear_all_connections(self):
        self.status_label.config(text='All connections cleared.')



    # ------------------------------------------------------------------ #
    #____________________Error handling __________________________#

    def _on_error(self, msg):
        # called back on the main thread if find_routes raises an exception
        self.find_btn.config(state='normal')
        self.status_label.config(text='Error.')
        messagebox.showerror('Error', msg)

    def show_error(self, error_msg):
        messagebox.showerror('Error', error_msg)


if __name__ == '__main__':
    root = tk.Tk()
    app = PathfinderApp(root)
    root.mainloop()
