import gzip
import json
import math
import os
import re
import sys
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk

import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


def _open_any(path: str):
    if path.lower().endswith(".gz"):
        return gzip.open(path, "rt", encoding="utf-8", errors="replace")
    return open(path, "rt", encoding="utf-8", errors="replace")


def _safe_float(x):
    try:
        return float(x)
    except Exception:
        return float("nan")


def _ns_to_ms(ns: float) -> float:
    return ns / 1_000_000.0


def _split_name(name: str):
    parts = name.split(".")
    if not parts:
        return ("(unknown)", name)
    subsystem = parts[0] if parts[0] else "(unknown)"
    return (subsystem, name)


def _rollup_key(name: str, depth: int):
    parts = name.split(".")
    if len(parts) < depth:
        return None
    return ".".join(parts[:depth])


class ProfileData:
    def __init__(self):
        self.path = ""
        self.events = []
        self.timestamps = []
        self.reasons = []
        self.sections_per_event = []
        self.subsystems = set()
        self.all_section_names = set()

    def load(self, path: str):
        self.path = path
        self.events.clear()
        self.timestamps.clear()
        self.reasons.clear()
        self.sections_per_event.clear()
        self.subsystems.clear()
        self.all_section_names.clear()

        with _open_any(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                t = obj.get("t")
                if t != "s":
                    continue

                ts = obj.get("ts")
                r = obj.get("r", "")
                sec = obj.get("sec", [])

                per = {}

                for it in sec:
                    if not isinstance(it, list) or len(it) < 5:
                        continue
                    name = it[0]
                    if not isinstance(name, str) or not name:
                        continue
                    c = int(it[1]) if it[1] is not None else 0
                    tot = int(it[2]) if it[2] is not None else 0
                    mn = int(it[3]) if it[3] is not None else 0
                    mx = int(it[4]) if it[4] is not None else 0
                    if c <= 0:
                        continue
                    per[name] = (c, tot, mn, mx)
                    ss, _ = _split_name(name)
                    self.subsystems.add(ss)
                    self.all_section_names.add(name)

                    k2 = _rollup_key(name, 2)
                    if k2 is not None and k2 != name:
                        if k2 in per:
                            pc, pt, pmn, pmx = per[k2]
                            per[k2] = (pc + c, pt + tot, min(pmn, mn) if pmn else mn, max(pmx, mx))
                        else:
                            per[k2] = (c, tot, mn, mx)
                        ss2, _ = _split_name(k2)
                        self.subsystems.add(ss2)
                        self.all_section_names.add(k2)

                self.timestamps.append(ts if isinstance(ts, int) else 0)
                self.reasons.append(r if isinstance(r, str) else "")
                self.sections_per_event.append(per)

        self.subsystems = set(sorted(self.subsystems))
        self.all_section_names = set(sorted(self.all_section_names))

    def list_sections_for_subsystem(self, subsystem: str):
        out = []
        for name in self.all_section_names:
            ss, _ = _split_name(name)
            if ss == subsystem:
                out.append(name)
        out.sort()
        return out

    def aggregate_section(self, section_name: str):
        total_c = 0
        total_ns = 0
        gmin = None
        gmax = None
        for per in self.sections_per_event:
            s = per.get(section_name)
            if not s:
                continue
            c, tot, mn, mx = s
            total_c += c
            total_ns += tot
            if gmin is None or (mn and mn < gmin):
                gmin = mn
            if gmax is None or (mx and mx > gmax):
                gmax = mx
        avg_ns = (total_ns / total_c) if total_c > 0 else 0.0
        return {
            "count": total_c,
            "total_ns": total_ns,
            "avg_ns": avg_ns,
            "min_ns": gmin or 0,
            "max_ns": gmax or 0,
        }

    def series_for_section(self, section_name: str):
        xs = []
        avgs = []
        mins = []
        maxs = []
        totals = []
        counts = []
        for i, per in enumerate(self.sections_per_event):
            s = per.get(section_name)
            if not s:
                continue
            c, tot, mn, mx = s
            if c <= 0:
                continue
            xs.append(i)
            counts.append(c)
            totals.append(tot)
            avgs.append(tot / c)
            mins.append(mn)
            maxs.append(mx)
        return xs, counts, totals, avgs, mins, maxs


class ProfilerViewer(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Repulsor Profiler Viewer")
        self.geometry("1400x820")
        ctk.set_appearance_mode("System")
        ctk.set_default_color_theme("blue")

        self.data = ProfileData()

        self.topbar = ctk.CTkFrame(self)
        self.topbar.pack(side="top", fill="x", padx=10, pady=(10, 6))

        self.btn_open = ctk.CTkButton(self.topbar, text="Open", command=self.open_file)
        self.btn_open.pack(side="left", padx=(10, 6), pady=8)

        self.path_var = tk.StringVar(value="")
        self.path_label = ctk.CTkLabel(self.topbar, textvariable=self.path_var, anchor="w")
        self.path_label.pack(side="left", fill="x", expand=True, padx=8, pady=8)

        self.search_var = tk.StringVar(value="")
        self.search_entry = ctk.CTkEntry(self.topbar, placeholder_text="Filter sections...", textvariable=self.search_var)
        self.search_entry.pack(side="right", padx=(6, 10), pady=8)
        self.search_var.trace_add("write", lambda *_: self.refresh_section_list())

        self.main = ctk.CTkFrame(self)
        self.main.pack(side="top", fill="both", expand=True, padx=10, pady=(0, 10))

        self.left = ctk.CTkFrame(self.main, width=360)
        self.left.pack(side="left", fill="y", padx=(0, 8), pady=10)

        self.right = ctk.CTkFrame(self.main)
        self.right.pack(side="left", fill="both", expand=True, padx=(8, 0), pady=10)

        self.subsystem_var = tk.StringVar(value="(none)")
        self.subsystem_menu = ctk.CTkOptionMenu(self.left, values=["(none)"], variable=self.subsystem_var, command=lambda *_: self.refresh_section_list())
        self.subsystem_menu.pack(side="top", fill="x", padx=10, pady=(10, 6))

        self.section_label = ctk.CTkLabel(self.left, text="Sections (Ctrl/Shift multi-select)")
        self.section_label.pack(side="top", anchor="w", padx=10, pady=(8, 4))

        self.tree = ttk.Treeview(self.left, columns=("name",), show="tree", selectmode="extended", height=24)
        self.tree.pack(side="top", fill="both", expand=True, padx=10, pady=(0, 10))
        self.tree.column("#0", anchor="w", width=200, stretch=True)
        self.tree.bind("<<TreeviewSelect>>", lambda e: self.on_selection_changed())
        self.left.bind("<Configure>", lambda e: self.after(10, self.adjust_tree_column))

        self.tabs = ctk.CTkTabview(self.right)
        self.tabs.pack(side="top", fill="both", expand=True, padx=10, pady=10)

        self.tab_overview = self.tabs.add("Overview")
        self.tab_timeseries = self.tabs.add("Timeseries")
        self.tab_compare = self.tabs.add("Compare")

        self.overview_top = ctk.CTkFrame(self.tab_overview)
        self.overview_top.pack(side="top", fill="x", padx=10, pady=(10, 6))

        self.sel_var = tk.StringVar(value="Selected: 0")
        self.sel_label = ctk.CTkLabel(self.overview_top, textvariable=self.sel_var)
        self.sel_label.pack(side="left", padx=10, pady=10)

        self.stats_box = ctk.CTkTextbox(self.tab_overview, height=220)
        self.stats_box.pack(side="top", fill="x", padx=10, pady=(0, 10))
        self.stats_box.configure(state="disabled")

        self.fig1 = Figure(figsize=(7.2, 4.8), dpi=100)
        self.ax1 = self.fig1.add_subplot(111)
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=self.tab_timeseries)
        self.canvas1.get_tk_widget().pack(side="top", fill="both", expand=True, padx=10, pady=(10, 0))
        self.toolbar1 = NavigationToolbar2Tk(self.canvas1, self.tab_timeseries)
        self.toolbar1.update()

        self.fig2 = Figure(figsize=(7.2, 4.8), dpi=100)
        self.ax2 = self.fig2.add_subplot(111)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=self.tab_compare)
        self.canvas2.get_tk_widget().pack(side="top", fill="both", expand=True, padx=10, pady=(10, 0))
        self.toolbar2 = NavigationToolbar2Tk(self.canvas2, self.tab_compare)
        self.toolbar2.update()

        self.compare_hint = ctk.CTkLabel(self.tab_compare, text="Select multiple sections on the left to compare their avg time per summary.")
        self.compare_hint.pack(side="bottom", pady=(0, 10))

        self.after(50, self.try_load_cli)

    def try_load_cli(self):
        if len(sys.argv) >= 2 and os.path.exists(sys.argv[1]):
            self.load_path(sys.argv[1])

    def open_file(self):
        path = filedialog.askopenfilename(
            title="Open profiler file",
            filetypes=[
                ("Profiler logs", "*.ndjson *.ndjson.gz *.jsonl *.jsonl.gz *.gz"),
                ("All files", "*.*"),
            ],
        )
        if path:
            self.load_path(path)

    def load_path(self, path: str):
        self.data.load(path)
        self.path_var.set(path)
        subs = sorted(self.data.subsystems) if self.data.subsystems else ["(none)"]
        if not subs:
            subs = ["(none)"]
        self.subsystem_menu.configure(values=subs)
        self.subsystem_var.set(subs[0])
        self.refresh_section_list()
        self.update_stats_text([])
        self.redraw_timeseries([])
        self.redraw_compare([])

    def refresh_section_list(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

        subsystem = self.subsystem_var.get()
        if not subsystem or subsystem == "(none)":
            return

        filt = (self.search_var.get() or "").strip().lower()
        names = self.data.list_sections_for_subsystem(subsystem)

        for name in names:
            if filt and filt not in name.lower():
                continue
            self.tree.insert("", "end", text=name)

    def get_selected_sections(self):
        items = self.tree.selection()
        out = []
        for it in items:
            out.append(self.tree.item(it, "text"))
        out = [x for x in out if isinstance(x, str) and x]
        return out

    def on_selection_changed(self):
        sel = self.get_selected_sections()
        self.sel_var.set(f"Selected: {len(sel)}")
        self.update_stats_text(sel)
        self.redraw_timeseries(sel)
        self.redraw_compare(sel)

    def update_stats_text(self, selected):
        self.stats_box.configure(state="normal")
        self.stats_box.delete("1.0", "end")

        if not self.data.sections_per_event:
            self.stats_box.insert("end", "No data loaded.\n")
            self.stats_box.configure(state="disabled")
            return

        if not selected:
            self.stats_box.insert("end", "Select one or more sections to see aggregated metrics.\n")
            self.stats_box.insert("end", f"\nSummaries: {len(self.data.sections_per_event)}\n")
            self.stats_box.insert("end", f"Subsystems: {len(self.data.subsystems)}\n")
            self.stats_box.configure(state="disabled")
            return

        lines = []
        for name in selected[:20]:
            agg = self.data.aggregate_section(name)
            c = agg["count"]
            tot = agg["total_ns"]
            avg = agg["avg_ns"]
            mn = agg["min_ns"]
            mx = agg["max_ns"]

            lines.append(name)
            lines.append(f"  count: {c}")
            lines.append(f"  avg:   { _ns_to_ms(avg):.4f} ms")
            lines.append(f"  min:   { _ns_to_ms(mn):.4f} ms")
            lines.append(f"  max:   { _ns_to_ms(mx):.4f} ms")
            lines.append(f"  total: { _ns_to_ms(tot):.2f} ms")
            lines.append("")

        if len(selected) > 20:
            lines.append(f"... ({len(selected) - 20} more selected)")

        self.stats_box.insert("end", "\n".join(lines))
        self.stats_box.configure(state="disabled")

    def redraw_timeseries(self, selected):
        self.ax1.clear()
        self.ax1.set_title("Avg time per summary (ms)")
        self.ax1.set_xlabel("Summary index")
        self.ax1.set_ylabel("Avg ms")

        if not selected:
            self.canvas1.draw()
            return

        name = selected[0]
        xs, _, _, avgs, _, _ = self.data.series_for_section(name)
        if xs:
            ys = [_ns_to_ms(v) for v in avgs]
            self.ax1.plot(xs, ys, label=name)
            self.ax1.legend(loc="upper right")

        self.canvas1.draw()

    def redraw_compare(self, selected):
        self.ax2.clear()
        self.ax2.set_title("Compare sections: avg time per summary (ms)")
        self.ax2.set_xlabel("Summary index")
        self.ax2.set_ylabel("Avg ms")

        if not selected:
            self.canvas2.draw()
            return

        for name in selected[:8]:
            xs, _, _, avgs, _, _ = self.data.series_for_section(name)
            if not xs:
                continue
            ys = [_ns_to_ms(v) for v in avgs]
            self.ax2.plot(xs, ys, label=name)

        if selected:
            self.ax2.legend(loc="upper right", fontsize=8)

        self.canvas2.draw()


def main():
    app = ProfilerViewer()
    app.mainloop()


if __name__ == "__main__":
    main()
