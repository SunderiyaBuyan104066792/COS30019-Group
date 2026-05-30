
"""
For widgets:

widget = tk.widget(parent, option=value, ...)
widget.pack()

create, then place with pack


anchor='w' means align to the west (left)

Something important for implementation:
We need to not use straight lines when creating the paths between nodes,
for this I am using OSRM which is a routing engine. 

You send two coordinates and it returns the actual road path and distance.
- this is important for Part A implementation as we should also compare the actual
distance between nodes

What happens:
- we send a URL of the coordinates to the orsm website using APIs
- we get a JSON that contains the road distance in m, and geometry.coordinates

geometry.coordinates is a list of [lon, lat] that trace the actual road. 





"""

import os
import tkinter as tk
from tkinter import ttk
import tkintermapview
from datetime import datetime
from tkinter import messagebox
from pathFinder import find_routes
import pandas as pd
import json
import requests


class PathfinderApp:
    # we are going to retake in the coordinates as 
    # it is what is expected of tkintermapview and we 
    # take in location for model training
    def load_data(self):
        # build the path to the file again 
        # we are taking in road_data.csv that exists after running parse_road_data.py
        script_folder = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_folder, 'graph', 'site_road_data', 'road_data.csv')

        #read the file into a table
        df = pd.read_csv(csv_path)

        # loop over every row and store the coordinates
        for _, row in df.iterrows():
            node_id = int(row['scats_num'])
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            roads = str(row['roads']).replace('_', ' ') 
            # so you can search by that exact location - HIGH STREET RD etc
            self.node_coordinates[node_id] = (lat, lon)
            self.node_roads[node_id] = roads

        #place a marker on the map for each node:
        for node_id, (lat, lon) in self.node_coordinates.items():
            # clicked_id=node_id captures this node's id for each marker
            # without it every marker would report the same (last) node_id
            def on_click(marker, clicked_id=node_id):
                self.on_node_clicked(clicked_id)

            self.map_widget.set_marker(lat, lon, text=str(node_id), command=on_click)

        self.status_label.config(text=f'Loaded {len(self.node_coordinates)} nodes.')



    def get_road_route(self, lat1, lon1, lat2, lon2):
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

        return distance_km, route_points



    def save_connections(self):
        script_folder = os.path.dirname(os.path.abspath(__file__))
        save_path = os.path.join(script_folder, 'graph', 'site_road_data', 'connections.json')

        seen = set()
        records = []
        for node_a, neighbours in self.connections.items():
            for node_b, distance_km, route_points in neighbours:  # unpack all three
                pair = (min(node_a, node_b), max(node_a, node_b))
                if pair not in seen:
                    seen.add(pair)
                    records.append({
                        'nodes': list(pair),
                        'distance_km': distance_km,
                        'route': route_points
                    })

        with open(save_path, 'w') as f:
            json.dump(records, f)





    def __init__(self, root):
        self.root = root
        root.title("SCATS Pathfinder")
        root.geometry("1200x900")

        # --- Sidebar ---
        sidebar = tk.Frame(root, width=260, bg='#2b2b2b')
        sidebar.pack(side='left', fill='y')
        sidebar.pack_propagate(False)

        # labels:

        # title label
        title_label = tk.Label(
            sidebar,
            text='SCATS Pathfinder',
            font=('Helvetica', 14, 'bold'),
            bg="#2b2b2b",
            fg='white'
        )
        title_label.pack(anchor="w", padx=12, pady=10)

        #--- Search Box: ---#
        # tk.Label(sidebar, text='Search nodes', bg='#2b2b2b', fg='#aaaaaa').pack(anchor='w', padx=12, pady=(10, 2))

        #using StringVar() 
        # https://www.askpython.com/python-modules/tkinter/stringvar-with-examples
        self.search_var = tk.StringVar()
        self.start_search_var = tk.StringVar()
        self.end_search_var = tk.StringVar()
        self.chosen_model_var = tk.StringVar()
        self.chosen_algorithmn_var = tk.StringVar()

        # we use trace to track variables
        # https://www.geeksforgeeks.org/python/tracing-tkinter-variables-in-python/
        self.search_var.trace_add('write', self.search_changed)
        self.start_search_var.trace_add('write', self.start_changed)
        self.end_search_var.trace_add('write', self.end_changed)
        

        # tk.Entry(sidebar, textvariable=self.search_var, width=28).pack(padx=12, pady=2)

        # # show the matching results for search - use height 5 to show 5 results:
        # self.search_results = tk.Listbox(sidebar, height=5, width=28)
        # self.search_results.pack(padx=12, pady=2)

        # # bind the selected result when clicked
        # self.search_results.bind('<<ListboxSelect>>', self.on_result_selected)
        


        # divider line 
        # divider = tk.Frame(sidebar, bg='#444444', height=1)
        # divider.pack(fill='x', padx=12, pady=4)

        # #Section label
        # self.section_label = tk.Label(
        #     sidebar,
        #     text='Edit connections',
        #     bg="#2b2b2b",
        #     fg='#aaaaaa',
        # )
        # self.section_label.pack(anchor="w", padx=12, pady=(4, 2))

        # #buttons:
        # self.connect_button = tk.Button(sidebar, text='Connect Nodes', width=22,
        #                                 command=self.start_connect_mode)
        # self.connect_button.pack(padx=12, pady=2)

        # self.remove_button = tk.Button(sidebar, text='Remove Connection', width=22,
        #                                command=self.start_remove_mode)
        # self.remove_button.pack(padx=12, pady=2)

        # self.clear_button = tk.Button(sidebar, text='Clear All Connections', width=22, fg='#ff6666',
        #                               command=self.clear_all_connections)
        # self.clear_button.pack(padx=12, pady=2)


        # # Another divider
        # divider2 = tk.Frame(sidebar, bg='#444444', height=1)
        # divider2.pack(fill='x', padx=12, pady=4)


        # Status label - shows messages to the user
        self.status_label = tk.Label(
            sidebar,
            text='',
            bg='#2b2b2b',
            fg='#ffaa00',
            font=('Helvetica', 9),
            wraplength=230,
            justify='left'
        )
        self.status_label.pack(anchor='w', padx=12, pady=4)

        # --- Map ---
        self.map_widget = tkintermapview.TkinterMapView(root)
        self.map_widget.pack(side='right', fill='both', expand=True)
        self.map_widget.set_position(-37.8136, 144.9631)
        self.map_widget.set_zoom(11)

        # tracks what mode the user is in and which node they clicked first
        self.edit_mode = None # 'connect', 'remove', or None
        self.first_node = None # first node clicked, waiting for second
        self.connections = {}  # node_id -> [(neighbour_id, distance_km, route_points), ..]

        self.node_coordinates = {}
        self.node_roads = {}
        self.load_data()

        divider = tk.Frame(sidebar, bg='#444444', height=1)
        divider.pack(fill='x', padx=12, pady=4)

        self.section_label = tk.Label(
            sidebar,
            text='Starting Location',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))
        tk.Entry(sidebar, textvariable=self.start_search_var, width=28).pack(padx=12, pady=2)

        # show the matching results for search - use height 5 to show 5 results:
        self.start_search_results = tk.Listbox(sidebar, exportselection=False,height=5, width=28)
        self.start_search_results.pack(padx=12, pady=2)

        # bind the selected result when clicked
        self.start_search_results.bind('<<ListboxSelect>>', self.on_result_selected)

        self.section_label = tk.Label(
            sidebar,
            text='End Location',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))

        tk.Entry(sidebar, textvariable=self.end_search_var, width=28).pack(padx=12, pady=2)

        # show the matching results for search - use height 5 to show 5 results:
        self.end_search_results = tk.Listbox(sidebar, exportselection=False,height=5, width=28)
        self.end_search_results.pack(padx=12, pady=2)

        # bind the selected result when clicked
        self.end_search_results.bind('<<ListboxSelect>>', self.on_result_selected)
        self.section_label = tk.Label(
            sidebar,
            text=' Select Model',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))

        self.chosen_model = ttk.Combobox(sidebar, width = 27, textvariable = self.chosen_model_var)
        self.chosen_model['values'] = (' lstm', 
                          ' gru',
                          ' custom')
        self.chosen_model.pack(padx=12, pady=2)

        self.section_label = tk.Label(
            sidebar,
            text='Select Search Algorithmn',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))
        self.chosen_algorithmn = ttk.Combobox(sidebar, width = 27, textvariable = self.chosen_algorithmn_var)
        self.chosen_algorithmn['values'] = ('BFS',
                                'DFS',
                                'GBFS',
                                'AS',
                                'DLS',
                                'ALT')
        
        self.chosen_algorithmn.pack(padx=12, pady=2)
        self.section_label = tk.Label(
            sidebar,
            text='Select Day',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))

        self.day_entry = tk.Entry(sidebar, width=28)
        self.day_entry.pack(padx=12, pady=2)
        
        self.section_label = tk.Label(
            sidebar,
            text='Select Hour',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))
        self.hour_entry = tk.Entry(sidebar, width=28)
        self.hour_entry.pack(padx=12, pady=2)

        self.section_label = tk.Label(
            sidebar,
            text='Select Minute',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))
        self.min_entry = tk.Entry(sidebar, width=28)
        self.min_entry.pack(padx=12, pady=2)

        self.section_label = tk.Label(
            sidebar,
            text='Select amount of paths',
            bg="#2b2b2b",
            fg='#aaaaaa',
        )
        self.section_label.pack(anchor="w", padx=12, pady=(4, 2))
        self.no_path_entry = tk.Entry(sidebar, width=28)
        self.no_path_entry.pack(padx=12, pady=2)
        
        btn = tk.Button(sidebar, text="Find Path", width=22, fg="#228B22", command=self.submit)
        btn.pack(padx=12, pady=(15, 10))
        





    # --- Button actions ---
    # These methods are called when a button is clicked.
    # self.status_label.config() updates the label's text.

    def on_search_typed(self, *args, search_type):
        # call this each time the search box changes
        match search_type:
            case search_type if search_type == "search":
                typed = self.search_var.get().lower()
                self.search_results.delete(0, 'end')

                if not typed:
                    return

                for node_id, roads in self.node_roads.items():
                    # match the text to a loaded location or ID
                    if typed in roads.lower() or typed in str(node_id):
                        self.search_results.insert('end', f'{node_id} - {roads}')
            case search_type if search_type == "start":
                typed = self.start_search_var.get().lower()
                self.start_search_results.delete(0, 'end')

                if not typed:
                    return

                for node_id, roads in self.node_roads.items():
                    # match the text to a loaded location or ID
                    if typed in roads.lower() or typed in str(node_id):
                        self.start_search_results.insert('end', f'{node_id} - {roads}')
            case search_type if search_type == "end":
                typed = self.end_search_var.get().lower()
                self.end_search_results.delete(0, 'end')

                if not typed:
                    return

                for node_id, roads in self.node_roads.items():
                    # match the text to a loaded location or ID
                    if typed in roads.lower() or typed in str(node_id):
                        self.end_search_results.insert('end', f'{node_id} - {roads}')


    def search_changed(self, *args):
        self.on_search_typed(*args, search_type="search")

    def start_changed(self, *args):
        self.on_search_typed(*args, search_type="start")

    def end_changed(self, *args):
        self.on_search_typed(*args, search_type="end")
    
    
    
    def get_selected(self, lb):
        selected = lb.curselection()
        if selected:
            index = selected[0]
            return lb.get(index)
        return None
    
    def submit(self, *args):
        Loc_Start = self.get_selected(self.start_search_results)
        Loc_End = self.get_selected(self.end_search_results)
        selected_model = self.chosen_model.get()
        selected_algo = self.chosen_algorithmn.get()

        selected_day = self.day_entry.get()
        selected_hour = self.hour_entry.get()
        selected_min = self.min_entry.get()
        selected_no_path = self.no_path_entry.get()

        if not all([Loc_Start, Loc_End, selected_model, selected_algo]):
            self.show_error("Missing or invalid Start, End, Model or Algorithmn")
            return

        if not self.validate_day(selected_day):
            self.show_error("invalid day")
            return

        if not self.validate_hour(selected_hour):
            self.show_error("invalid hour")
            return

        if not self.validate_minute(selected_min):
            self.show_error("invalid minute")
            return
        
        if not self.validate_No_path(selected_no_path):
            self.show_error("invalid minute")
            return
        
        
        origin = int(Loc_Start.split(" - ")[0])
        destination = int(Loc_End.split(" - ")[0])
        departure_time = datetime(2006, 11, int(selected_day), int(selected_hour), int(selected_min), 0)
        routes = []
        routes = find_routes(origin, destination, departure_time, selected_model, selected_algo, int(selected_no_path) )
        if len(routes) < 1:
            self.show_error("No valid Path")

        self.draw_routes(routes)
        self.draw_best_route(routes)
    
    def validate_day(self, day):
        if day.isdigit() and 1 <= int(day) <= 31:
            return True
        return False


    def validate_hour(self, hour):
        if hour.isdigit() and 0 <= int(hour) <= 23:
            return True
        return False


    def validate_minute(self, minute):
        if minute.isdigit() and 0 <= int(minute) <= 59:
            return True
        return False
    
    def validate_No_path(self, minute):
        if minute.isdigit() and 0 <= int(minute) <= 59:
            return True
        return False

    def show_error(self,error_msg):
        messagebox.showerror("Error", f"{error_msg}")

    def draw_routes(self, routes):
        for route in routes[1:]:  
            path = route['path']
            full_route = []

            for j in range(len(path) - 1):
                n1 = path[j]
                n2 = path[j + 1]

                lat1, lon1 = self.node_coordinates[n1]
                lat2, lon2 = self.node_coordinates[n2]

                _, segment = self.get_road_route(lat1, lon1, lat2, lon2)

                if j > 0:
                    segment = segment[1:]

                full_route.extend(segment)

            self.map_widget.set_path(full_route, color="black", width=3)

    def draw_best_route(self, routes):
        route = routes[0]  # take best route

        path = route['path']
        full_route = []

        for i in range(len(path) - 1):
            n1 = path[i]
            n2 = path[i + 1]

            lat1, lon1 = self.node_coordinates[n1]
            lat2, lon2 = self.node_coordinates[n2]

            _, segment = self.get_road_route(lat1, lon1, lat2, lon2)

            if i > 0:
                segment = segment[1:]  # avoid duplicates

            full_route.extend(segment)

        self.map_widget.set_path(full_route, color="Red", width=4)

    # just a visual thing, we zoom in on the selected node
    def on_result_selected(self, event):
        selected = self.search_results.curselection()

        if not selected:
            return

        text = self.search_results.get(selected[0])
        # split on ' - ' and take the first part to get the node ID
        # maxsplit=1 means we only split on the first ' - ' in case road names contain hyphens
        node_id = int(text.split(' - ', 1)[0])

        # zoom in on that node
        lat, lon = self.node_coordinates[node_id]
        self.map_widget.set_position(lat, lon)
        self.map_widget.set_zoom(15)
        self.status_label.config(text=f'Jumped to node {node_id}')




    def on_node_clicked(self, node_id):
        # called whenever a marker is clicked on the map
        if self.edit_mode == 'connect':
            self.handle_connect(node_id)
        else:
            # no mode active - just show the node's location
            lat, lon = self.node_coordinates[node_id]
            self.status_label.config(text=f'Node {node_id}  ({lat:.4f}, {lon:.4f})')



    def start_connect_mode(self):
        self.edit_mode = 'connect'
        self.first_node = None
        self.status_label.config(text='Connect: click the FIRST node.')



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

        # get the actual road route between the two nodes
        lat1, lon1 = self.node_coordinates[first]
        lat2, lon2 = self.node_coordinates[second]
        distance_km, route_points = self.get_road_route(lat1, lon1, lat2, lon2)

        # store the connection both ways with the real distance and route points
        self.connections.setdefault(first, []).append((second, distance_km, route_points))
        self.connections.setdefault(second, []).append((first,  distance_km, route_points))

        # draw the road-following line
        self.map_widget.set_path(route_points, color='#888888', width=6)

        self.save_connections()
        self.status_label.config(text=f'Connected {first} ↔ {second}  ({distance_km:.2f} km) — saved.')



    def start_remove_mode(self):
        self.status_label.config(text='Remove: click the FIRST node.')

    def clear_all_connections(self):
        self.status_label.config(text='All connections cleared.')


#--Entry Point:--#
root= tk.Tk()
app = PathfinderApp(root)

root.mainloop()