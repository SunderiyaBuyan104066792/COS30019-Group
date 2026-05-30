import tkinter as tk
from tkinter import ttk, messagebox
import threading
from datetime import datetime
import math
import statistics
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pathFinder import find_routes
from graph.graph import Graph

CANVAS_W = 760
CANVAS_H = 640
PADDING = 45

BG = '#0f0f1a'
PANEL_BG = '#1a1a2e'
FG = '#ffffff'
DIM_EDGE = '#2a2a4a'
DIM_NODE = '#3a3a5a'

ROUTE_COLORS = [
    '#e05252', '#5299e0', '#52e08a', '#e0b452',
    '#b452e0', '#52d4e0', '#e07a52', '#e052b4',
]

ALGORITHMS = ['AS', 'BFS', 'DFS', 'GBFS', 'DLS', 'ALT']
MODELS = ['lstm', 'gru', 'custom']


def _load_scats():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'graph', 'site_road_data', 'road_data.csv')
    sites = []
    try:
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                sites.append(int(row['scats_num']))
    except Exception:
        pass
    return sorted(sites)


class VisApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('TBRGS Route Visualiser')
        self.configure(bg=BG)
        self.resizable(True, True)

        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure('.', background=PANEL_BG, foreground=FG, bordercolor=DIM_NODE)
        style.configure('TLabel', background=PANEL_BG, foreground=FG)
        style.configure('TFrame', background=PANEL_BG)
        style.configure('TLabelframe', background=PANEL_BG, foreground=FG)
        style.configure('TLabelframe.Label', background=PANEL_BG, foreground=FG)
        style.configure('TCombobox', fieldbackground='#e8e8f0', foreground=BG, selectbackground=DIM_NODE)
        style.configure('TSpinbox', fieldbackground='#e8e8f0', foreground=BG)
        style.configure('TButton', background='#2a2a5a', foreground=FG, padding=4)
        style.map('TButton', background=[('active', '#3a3a7a')])

        self._graph = None
        self._node_pos = {}
        self._routes = []
        self._scats = _load_scats()

        self._build_ui()
        self._load_graph()

    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.grid(sticky='nsew')
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky='ns', padx=(0, 10))

        params = ttk.LabelFrame(left, text='Parameters', padding=8)
        params.pack(fill='x')

        vals = [str(s) for s in self._scats]
        fields = [
            ('Origin', 'origin_var', vals, vals[0]),
            ('Destination', 'dest_var', vals, vals[-1]),
            ('Algorithm', 'method_var', ALGORITHMS, 'AS'),
            ('Model', 'model_var', MODELS, 'lstm'),
        ]
        for r, (label, attr, options, default) in enumerate(fields):
            ttk.Label(params, text=label).grid(row=r, column=0, sticky='w', pady=2)
            var = tk.StringVar(value=default)
            setattr(self, attr, var)
            ttk.Combobox(params, textvariable=var, values=options,
                         state='readonly', width=10).grid(row=r, column=1, sticky='w', pady=2)

        spinners = [
            ('Day (1–30)', 'day_var', 1, 30, '4'),
            ('Hour (0–23)', 'hour_var', 0, 23, '8'),
            ('Minute', 'min_var', 0, 59, '0'),
            ('Routes (k)', 'k_var', 1, 10, '3'),
        ]
        base = len(fields)
        for i, (label, attr, lo, hi, default) in enumerate(spinners):
            ttk.Label(params, text=label).grid(row=base + i, column=0, sticky='w', pady=2)
            var = tk.StringVar(value=default)
            setattr(self, attr, var)
            ttk.Spinbox(params, from_=lo, to=hi, textvariable=var,
                        width=6).grid(row=base + i, column=1, sticky='w', pady=2)

        self.find_btn = ttk.Button(params, text='Find Routes',
                                   command=self._on_find, state='disabled')
        self.find_btn.grid(row=base + len(spinners), columnspan=2,
                           sticky='ew', pady=(10, 0))

        rframe = ttk.LabelFrame(left, text='Routes', padding=8)
        rframe.pack(fill='both', expand=True, pady=(10, 0))
        self.route_list = tk.Listbox(rframe, width=24, height=10,
                                     selectmode='single', font=('Courier', 9),
                                     bg=BG, fg=FG, selectbackground=DIM_NODE,
                                     relief='flat', borderwidth=0)
        self.route_list.pack(fill='both', expand=True)
        self.route_list.bind('<<ListboxSelect>>', self._on_route_select)

        self.status_var = tk.StringVar(value='Loading graph…')
        ttk.Label(left, textvariable=self.status_var,
                  foreground=FG, wraplength=200).pack(pady=(8, 0), anchor='w')

        right = ttk.LabelFrame(main, text='Network', padding=4)
        right.grid(row=0, column=1, sticky='nsew')
        self.canvas = tk.Canvas(right, width=CANVAS_W, height=CANVAS_H,
                                background=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)

    def _load_graph(self):
        def run():
            g = Graph()
            g.build()
            self.after(0, lambda: self._on_graph_ready(g))
        threading.Thread(target=run, daemon=True).start()

    def _on_graph_ready(self, graph):
        self._graph = graph
        self._compute_layout()
        self._draw_base()
        self.status_var.set('Ready.')
        self.find_btn.config(state='normal')

    def _compute_layout(self):
        nodes = self._graph.nodes
        lats = [n.lat for n in nodes.values()]
        lons = [n.lon for n in nodes.values()]

        mid_lat = statistics.median(lats)
        mid_lon = statistics.median(lons)
        core_lats = [l for l in lats if abs(l - mid_lat) <= 2.0]
        core_lons = [l for l in lons if abs(l - mid_lon) <= 2.0]

        min_lat, max_lat = min(core_lats), max(core_lats)
        min_lon, max_lon = min(core_lons), max(core_lons)

        cos_lat = math.cos(math.radians((min_lat + max_lat) / 2))
        lat_km = (max_lat - min_lat) * 111 or 1
        lon_km = (max_lon - min_lon) * 111 * cos_lat or 1

        w = CANVAS_W - 2 * PADDING
        h = CANVAS_H - 2 * PADDING
        scale = min(w / lon_km, h / lat_km)
        x0 = PADDING + (w - lon_km * scale) / 2
        y0 = PADDING + (h - lat_km * scale) / 2

        for scats, node in nodes.items():
            cx = x0 + (node.lon - min_lon) * 111 * cos_lat * scale
            cy = y0 + (max_lat - node.lat) * 111 * scale
            cx = max(4.0, min(float(CANVAS_W - 4), cx))
            cy = max(4.0, min(float(CANVAS_H - 4), cy))
            self._node_pos[scats] = (cx, cy)

    def _draw_base(self):
        c = self.canvas
        c.delete('all')

        seen = set()
        for edge in self._graph.edges:
            key = tuple(sorted((edge.from_node.scats_num, edge.to_node.scats_num)))
            if key in seen:
                continue
            seen.add(key)
            a = self._node_pos.get(edge.from_node.scats_num)
            b = self._node_pos.get(edge.to_node.scats_num)
            if a and b:
                c.create_line(a[0], a[1], b[0], b[1],
                              fill=DIM_EDGE, width=1, tags='base_edge')

        for scats, (cx, cy) in self._node_pos.items():
            c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                          fill=FG, outline='', tags='base_node')
            c.create_text(cx + 5, cy - 5, text=str(scats),
                          font=('Arial', 7), fill=FG,
                          anchor='sw', tags='node_label')

    def _on_find(self):
        try:
            origin = int(self.origin_var.get())
            dest = int(self.dest_var.get())
            depart = datetime(2006, 11, int(self.day_var.get()),
                              int(self.hour_var.get()), int(self.min_var.get()))
            k = int(self.k_var.get())
        except ValueError as exc:
            messagebox.showerror('Input Error', str(exc))
            return

        self.find_btn.config(state='disabled')
        self.status_var.set('Searching…')
        self.route_list.delete(0, 'end')
        self._clear_routes()

        model = self.model_var.get()
        method = self.method_var.get()

        def run():
            try:
                routes = find_routes(origin, dest, depart,
                                     model_name=model, method=method, k=k)
                self.after(0, lambda: self._on_routes_found(routes, origin, dest))
            except Exception as exc:
                self.after(0, lambda: self._on_error(str(exc)))

        threading.Thread(target=run, daemon=True).start()

    def _on_routes_found(self, routes, origin, dest):
        self.find_btn.config(state='normal')
        self._routes = routes

        if not routes:
            self.status_var.set('No routes found.')
            return

        self.status_var.set(f'{len(routes)} route(s) found.')

        for i, route in enumerate(routes):
            color = ROUTE_COLORS[i % len(ROUTE_COLORS)]
            self._draw_route(route['path'], color, tag=f'route_{i}')
            m, s = divmod(int(route['cost_seconds']), 60)
            self.route_list.insert('end', f'  {i+1}.  {m} min {s} sec')
            self.route_list.itemconfig(i, foreground=color)

        self._mark_node(origin, '#52e08a', str(origin))
        self._mark_node(dest, '#e05252', str(dest))
        self.route_list.selection_set(0)
        self._highlight_route(0)

    def _draw_route(self, path, color, tag):
        c = self.canvas
        for i in range(len(path) - 1):
            a = self._node_pos.get(path[i])
            b = self._node_pos.get(path[i + 1])
            if a and b:
                c.create_line(a[0], a[1], b[0], b[1],
                              fill=color, width=3, capstyle='round',
                              tags=(tag, 'route_item'))
        for scats in path:
            pos = self._node_pos.get(scats)
            if pos:
                cx, cy = pos
                c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                              fill=color, outline=BG, width=1,
                              tags=(tag, 'route_item'))

    def _mark_node(self, scats, color, label=''):
        pos = self._node_pos.get(scats)
        if not pos:
            return
        cx, cy = pos
        self.canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8,
                                fill=color, outline=BG, width=2, tags='marker')
        self.canvas.create_text(cx, cy - 17, text=label,
                                fill=color, font=('Arial', 8, 'bold'), tags='marker')

    def _highlight_route(self, index):
        for i in range(len(self._routes)):
            alpha = '#1a1a3a' if i != index else ROUTE_COLORS[i % len(ROUTE_COLORS)]
            width = 3 if i == index else 1
            for item in self.canvas.find_withtag(f'route_{i}'):
                kind = self.canvas.type(item)
                if kind == 'line':
                    self.canvas.itemconfig(item, fill=alpha, width=width)
                elif kind == 'oval':
                    self.canvas.itemconfig(item, fill=alpha,
                                           outline=BG if i == index else '')
        self.canvas.tag_raise(f'route_{index}')
        self.canvas.tag_raise('marker')

    def _clear_routes(self):
        self.canvas.delete('route_item')
        self.canvas.delete('marker')

    def _on_route_select(self, _event):
        sel = self.route_list.curselection()
        if sel:
            self._highlight_route(sel[0])

    def _on_error(self, msg):
        self.find_btn.config(state='normal')
        self.status_var.set('Error.')
        messagebox.showerror('Error', msg)


if __name__ == '__main__':
    VisApp().mainloop()
