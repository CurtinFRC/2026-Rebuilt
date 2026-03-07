#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean


DEFAULT_INPUT = "build/offload/real-offload-latency.json"
DEFAULT_OUTPUT_DIR = "build/offload/real-offload-report"


def resolve_path(project_root: Path, raw: str) -> Path:
    path = Path(raw)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def to_float(value):
    if value is None:
        return None
    return float(value)


def pct(sorted_values, quantile: float) -> float:
    if not sorted_values:
        return float("nan")
    idx = max(0, min(len(sorted_values) - 1, math.ceil(quantile * len(sorted_values)) - 1))
    return sorted_values[idx]


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "min": ordered[0],
        "p50": pct(ordered, 0.50),
        "avg": mean(ordered),
        "p95": pct(ordered, 0.95),
        "max": ordered[-1],
    }


def fmt(value):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "n/a"
    return f"{value:.3f}"


def all_metric_values(tasks: dict, key: str):
    out = []
    for task in tasks.values():
        for sample in task.get("samples", []):
            value = to_float(sample.get(key))
            if value is not None:
                out.append(value)
    return out


def task_metric_values(task: dict, key: str):
    values = []
    for sample in task.get("samples", []):
        value = to_float(sample.get(key))
        if value is not None:
            values.append(value)
    return values


def collect_rows(tasks: dict):
    rows = []
    for task_id, task in tasks.items():
        for sample in task.get("samples", []):
            rows.append(
                {
                    "task": task_id,
                    "sample": int(sample.get("sampleIndex", 0)),
                    "rtt": to_float(sample.get("rttMs")),
                    "server": to_float(sample.get("serverMs")),
                    "exec": to_float(sample.get("execMs")),
                    "queue": to_float(sample.get("queueMs")),
                    "overhead": to_float(sample.get("overheadMs")),
                }
            )
    rows.sort(key=lambda row: (row["task"], row["sample"]))
    return rows


def compute_overall(tasks: dict):
    return {
        "rttMs": stats(all_metric_values(tasks, "rttMs")),
        "serverMs": stats(all_metric_values(tasks, "serverMs")),
        "execMs": stats(all_metric_values(tasks, "execMs")),
        "queueMs": stats(all_metric_values(tasks, "queueMs")),
        "overheadMs": stats(all_metric_values(tasks, "overheadMs")),
    }


def compute_component_averages(tasks: dict):
    rows = collect_rows(tasks)
    queue_values = [row["queue"] for row in rows if row["queue"] is not None]
    exec_values = [row["exec"] for row in rows if row["exec"] is not None]
    server_values = [row["server"] for row in rows if row["server"] is not None]
    client_network_values = [row["overhead"] for row in rows if row["overhead"] is not None]

    queue_avg = mean(queue_values) if queue_values else 0.0
    exec_avg = mean(exec_values) if exec_values else 0.0
    server_avg = mean(server_values) if server_values else 0.0
    client_network_avg = mean(client_network_values) if client_network_values else 0.0
    server_framework_avg = max(0.0, server_avg - queue_avg - exec_avg)
    total_avg = queue_avg + exec_avg + server_framework_avg + client_network_avg

    return {
        "queue": queue_avg,
        "exec": exec_avg,
        "server_framework": server_framework_avg,
        "client_network": client_network_avg,
        "total": total_avg,
    }


def write_summary_files(output_dir: Path, data: dict, tasks: dict, overall: dict, components: dict):
    lines = []
    lines.append("Offload Latency Report")
    lines.append("======================")
    lines.append("")
    lines.append(f"Host: {data.get('host')}:{data.get('port')}")
    lines.append(f"Read timeout: {data.get('readTimeoutMs')} ms")
    lines.append(
        f"Warmup samples: {data.get('warmupSamples')}  Measured samples: {data.get('measuredSamples')}"
    )
    lines.append("")
    lines.append("Overall")
    lines.append("-------")
    for name, summary in overall.items():
        if not summary:
            continue
        lines.append(
            f"{name}: n={summary['count']} min={fmt(summary['min'])} p50={fmt(summary['p50'])} "
            f"avg={fmt(summary['avg'])} p95={fmt(summary['p95'])} max={fmt(summary['max'])}"
        )

    total = components["total"] if components["total"] > 0 else 1.0
    lines.append("")
    lines.append("Avg RTT Component Split")
    lines.append("-----------------------")
    lines.append(f"queueMs(avg): {fmt(components['queue'])} ({components['queue'] / total * 100.0:.1f}%)")
    lines.append(f"execMs(avg): {fmt(components['exec'])} ({components['exec'] / total * 100.0:.1f}%)")
    lines.append(
        f"serverFrameworkMs(avg): {fmt(components['server_framework'])} "
        f"({components['server_framework'] / total * 100.0:.1f}%)"
    )
    lines.append(
        f"clientNetworkMs(avg): {fmt(components['client_network'])} "
        f"({components['client_network'] / total * 100.0:.1f}%)"
    )
    lines.append("")
    lines.append("Per Task")
    lines.append("--------")
    for task_id, task in tasks.items():
        summary = task.get("summary", {})
        rtt = summary.get("rtt", {})
        server = summary.get("server", {})
        execs = summary.get("exec", {})
        overhead = summary.get("overhead", {})
        lines.append(
            f"{task_id}: "
            f"rtt(avg/p95)={fmt(to_float(rtt.get('avgMs')))}/{fmt(to_float(rtt.get('p95Ms')))} "
            f"server(avg)={fmt(to_float(server.get('avgMs')))} "
            f"exec(avg)={fmt(to_float(execs.get('avgMs')))} "
            f"overhead(avg)={fmt(to_float(overhead.get('avgMs')))}"
        )

    (output_dir / "summary.txt").write_text("\n".join(lines), encoding="utf-8")

    markdown = []
    markdown.append("# Offload Latency Report")
    markdown.append("")
    markdown.append(f"- Host: `{data.get('host')}:{data.get('port')}`")
    markdown.append(f"- Read timeout: `{data.get('readTimeoutMs')} ms`")
    markdown.append(
        f"- Warmup samples: `{data.get('warmupSamples')}`, Measured samples: `{data.get('measuredSamples')}`"
    )
    markdown.append("")
    markdown.append("## Overall")
    markdown.append("")
    markdown.append("| Metric | Count | Min | P50 | Avg | P95 | Max |")
    markdown.append("|---|---:|---:|---:|---:|---:|---:|")
    for metric, summary in overall.items():
        if not summary:
            continue
        markdown.append(
            f"| {metric} | {summary['count']} | {fmt(summary['min'])} | {fmt(summary['p50'])} | "
            f"{fmt(summary['avg'])} | {fmt(summary['p95'])} | {fmt(summary['max'])} |"
        )
    markdown.append("")
    markdown.append("## Avg RTT Component Split")
    markdown.append("")
    markdown.append("| Component | Avg ms | Percent |")
    markdown.append("|---|---:|---:|")
    markdown.append(f"| Queue | {fmt(components['queue'])} | {components['queue'] / total * 100.0:.1f}% |")
    markdown.append(f"| Execute | {fmt(components['exec'])} | {components['exec'] / total * 100.0:.1f}% |")
    markdown.append(
        f"| Server Framework | {fmt(components['server_framework'])} | "
        f"{components['server_framework'] / total * 100.0:.1f}% |"
    )
    markdown.append(
        f"| Client + Network | {fmt(components['client_network'])} | "
        f"{components['client_network'] / total * 100.0:.1f}% |"
    )
    (output_dir / "summary.md").write_text("\n".join(markdown), encoding="utf-8")


def write_csv_exports(output_dir: Path, tasks: dict):
    task_csv = output_dir / "task_metrics.csv"
    with task_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "taskId",
                "samples",
                "rttMinMs",
                "rttP50Ms",
                "rttAvgMs",
                "rttP95Ms",
                "rttMaxMs",
                "serverAvgMs",
                "execAvgMs",
                "queueAvgMs",
                "overheadAvgMs",
            ]
        )
        for task_id, task in tasks.items():
            summary = task.get("summary", {})
            queue_values = task_metric_values(task, "queueMs")
            writer.writerow(
                [
                    task_id,
                    len(task.get("samples", [])),
                    to_float(summary.get("rtt", {}).get("minMs")),
                    to_float(summary.get("rtt", {}).get("p50Ms")),
                    to_float(summary.get("rtt", {}).get("avgMs")),
                    to_float(summary.get("rtt", {}).get("p95Ms")),
                    to_float(summary.get("rtt", {}).get("maxMs")),
                    to_float(summary.get("server", {}).get("avgMs")),
                    to_float(summary.get("exec", {}).get("avgMs")),
                    mean(queue_values) if queue_values else None,
                    to_float(summary.get("overhead", {}).get("avgMs")),
                ]
            )

    sample_csv = output_dir / "sample_points.csv"
    with sample_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["taskId", "sampleIndex", "rttMs", "serverMs", "execMs", "queueMs", "overheadMs"])
        for task_id, task in tasks.items():
            for sample in task.get("samples", []):
                writer.writerow(
                    [
                        task_id,
                        sample.get("sampleIndex"),
                        sample.get("rttMs"),
                        sample.get("serverMs"),
                        sample.get("execMs"),
                        sample.get("queueMs"),
                        sample.get("overheadMs"),
                    ]
                )


def pie_autopct_with_ms(values):
    total = sum(values)

    def formatter(percent):
        value = percent * total / 100.0
        return f"{percent:.1f}%\n{value:.3f} ms"

    return formatter


def create_figures(tasks: dict):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(
            "matplotlib is required. Install with: pip install -r requirements.txt"
        ) from exc

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except Exception:
        pass

    components = compute_component_averages(tasks)
    task_ids = list(tasks.keys())
    figures = {}

    pie_values = [
        components["queue"],
        components["exec"],
        components["server_framework"],
        components["client_network"],
    ]
    pie_labels = ["Queue", "Execute", "Server Framework", "Client + Network"]
    fig_pie, ax_pie = plt.subplots(figsize=(7, 6))
    wedges, _texts, _autotexts = ax_pie.pie(
        pie_values,
        labels=pie_labels,
        autopct=pie_autopct_with_ms(pie_values),
        startangle=110,
        textprops={"fontsize": 10},
    )
    ax_pie.set_title("Average RTT Composition")
    ax_pie.legend(wedges, pie_labels, loc="lower left", bbox_to_anchor=(0.0, -0.12))
    figures["Average RTT Composition"] = fig_pie

    fig_rtt, ax_rtt = plt.subplots(figsize=(11, 6))
    lines = []
    for task_id in task_ids:
        samples = sorted(tasks[task_id].get("samples", []), key=lambda sample: int(sample.get("sampleIndex", 0)))
        x_values = [int(sample.get("sampleIndex", 0)) for sample in samples]
        y_values = [to_float(sample.get("rttMs")) for sample in samples]
        line, = ax_rtt.plot(x_values, y_values, marker="o", markersize=2.5, linewidth=1.2, label=task_id)
        lines.append(line)
    ax_rtt.set_title("RTT Over Samples")
    ax_rtt.set_xlabel("Sample Index")
    ax_rtt.set_ylabel("RTT (ms)")
    legend = ax_rtt.legend(fontsize=8, loc="upper right")
    line_by_legend = {}
    for legend_line, real_line in zip(legend.get_lines(), lines):
        legend_line.set_picker(True)
        legend_line.set_pickradius(6)
        line_by_legend[legend_line] = real_line

    def on_pick(event):
        legend_line = event.artist
        target = line_by_legend.get(legend_line)
        if target is None:
            return
        target.set_visible(not target.get_visible())
        legend_line.set_alpha(1.0 if target.get_visible() else 0.25)
        fig_rtt.canvas.draw_idle()

    fig_rtt.canvas.mpl_connect("pick_event", on_pick)
    figures["RTT Over Samples"] = fig_rtt

    queue_avgs = []
    exec_avgs = []
    server_framework_avgs = []
    client_network_avgs = []
    for task_id in task_ids:
        task = tasks[task_id]
        queue_values = task_metric_values(task, "queueMs")
        exec_values = task_metric_values(task, "execMs")
        server_values = task_metric_values(task, "serverMs")
        overhead_values = task_metric_values(task, "overheadMs")
        queue_avg = mean(queue_values) if queue_values else 0.0
        exec_avg = mean(exec_values) if exec_values else 0.0
        server_avg = mean(server_values) if server_values else 0.0
        overhead_avg = mean(overhead_values) if overhead_values else 0.0
        framework_avg = max(0.0, server_avg - queue_avg - exec_avg)
        queue_avgs.append(queue_avg)
        exec_avgs.append(exec_avg)
        server_framework_avgs.append(framework_avg)
        client_network_avgs.append(overhead_avg)

    fig_stack, ax_stack = plt.subplots(figsize=(11, 6))
    indices = range(len(task_ids))
    ax_stack.bar(indices, queue_avgs, label="Queue")
    ax_stack.bar(indices, exec_avgs, bottom=queue_avgs, label="Execute")
    second_bottom = [q + e for q, e in zip(queue_avgs, exec_avgs)]
    ax_stack.bar(indices, server_framework_avgs, bottom=second_bottom, label="Server Framework")
    third_bottom = [a + b for a, b in zip(second_bottom, server_framework_avgs)]
    ax_stack.bar(indices, client_network_avgs, bottom=third_bottom, label="Client + Network")
    ax_stack.set_title("Average Latency Components By Task")
    ax_stack.set_ylabel("Milliseconds")
    ax_stack.set_xticks(list(indices))
    ax_stack.set_xticklabels(task_ids, rotation=18, ha="right")
    ax_stack.legend()
    figures["Average Components By Task"] = fig_stack

    fig_box, ax_box = plt.subplots(figsize=(11, 6))
    box_values = [task_metric_values(tasks[task_id], "rttMs") for task_id in task_ids]
    ax_box.boxplot(box_values, tick_labels=task_ids, showfliers=True, showmeans=True)
    ax_box.set_title("RTT Distribution By Task")
    ax_box.set_ylabel("RTT (ms)")
    ax_box.tick_params(axis="x", rotation=18)
    figures["RTT Distribution By Task"] = fig_box

    fig_scatter, ax_scatter = plt.subplots(figsize=(10, 6))
    for task_id in task_ids:
        task = tasks[task_id]
        exec_values = []
        overhead_values = []
        for sample in task.get("samples", []):
            exec_ms = to_float(sample.get("execMs"))
            overhead_ms = to_float(sample.get("overheadMs"))
            if exec_ms is None or overhead_ms is None:
                continue
            exec_values.append(exec_ms)
            overhead_values.append(overhead_ms)
        ax_scatter.scatter(exec_values, overhead_values, s=16, alpha=0.65, label=task_id)
    ax_scatter.set_title("Exec vs Client+Network Overhead")
    ax_scatter.set_xlabel("Exec (ms)")
    ax_scatter.set_ylabel("Client + Network Overhead (ms)")
    ax_scatter.legend(fontsize=8)
    figures["Exec vs Overhead"] = fig_scatter

    fig_hist, ax_hist = plt.subplots(figsize=(10, 6))
    overall_rtt = all_metric_values(tasks, "rttMs")
    bins = min(40, max(10, len(overall_rtt) // 8 if overall_rtt else 10))
    ax_hist.hist(overall_rtt, bins=bins)
    ax_hist.set_title("Overall RTT Histogram")
    ax_hist.set_xlabel("RTT (ms)")
    ax_hist.set_ylabel("Count")
    figures["Overall RTT Histogram"] = fig_hist

    return figures, components


def save_figures(output_dir: Path, figures: dict):
    for name, figure in figures.items():
        filename = name.lower().replace(" ", "_").replace("+", "plus") + ".png"
        figure.savefig(output_dir / filename, dpi=150, bbox_inches="tight")


def launch_gui(data: dict, overall: dict, components: dict, figures: dict):
    try:
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    except Exception as exc:
        raise SystemExit("Tk GUI backend unavailable for matplotlib.") from exc

    root = tk.Tk()
    root.title("Offload Latency Dashboard")
    root.geometry("1600x980")

    header = ttk.Frame(root, padding=10)
    header.pack(fill=tk.X)
    ttk.Label(
        header,
        text=(
            f"Host {data.get('host')}:{data.get('port')}    "
            f"ReadTimeout {data.get('readTimeoutMs')} ms    "
            f"Warmup {data.get('warmupSamples')}    "
            f"Measured {data.get('measuredSamples')}"
        ),
        font=("Segoe UI", 11, "bold"),
    ).pack(anchor="w")
    total = components["total"] if components["total"] > 0 else 1.0
    ttk.Label(
        header,
        text=(
            f"Split avg: queue={components['queue']:.3f} ms ({components['queue']/total*100.0:.1f}%)  "
            f"exec={components['exec']:.3f} ms ({components['exec']/total*100.0:.1f}%)  "
            f"serverFramework={components['server_framework']:.3f} ms ({components['server_framework']/total*100.0:.1f}%)  "
            f"client+network={components['client_network']:.3f} ms ({components['client_network']/total*100.0:.1f}%)"
        ),
        font=("Segoe UI", 10),
    ).pack(anchor="w", pady=(4, 0))

    metrics_frame = ttk.Frame(root, padding=(10, 0, 10, 4))
    metrics_frame.pack(fill=tk.X)
    columns = ("Metric", "Count", "Min", "P50", "Avg", "P95", "Max")
    tree = ttk.Treeview(metrics_frame, columns=columns, show="headings", height=5)
    for column in columns:
        tree.heading(column, text=column)
        tree.column(column, anchor="center", width=120 if column != "Metric" else 240)
    for metric, summary in overall.items():
        if not summary:
            continue
        tree.insert(
            "",
            tk.END,
            values=(
                metric,
                summary["count"],
                fmt(summary["min"]),
                fmt(summary["p50"]),
                fmt(summary["avg"]),
                fmt(summary["p95"]),
                fmt(summary["max"]),
            ),
        )
    tree.pack(fill=tk.X)

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

    for title, figure in figures.items():
        tab = ttk.Frame(notebook)
        notebook.add(tab, text=title)

        toolbar_frame = ttk.Frame(tab)
        toolbar_frame.pack(fill=tk.X)
        canvas = FigureCanvasTkAgg(figure, master=tab)
        canvas.draw()
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side=tk.LEFT)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    root.mainloop()


def main():
    parser = argparse.ArgumentParser(
        description="Interactive matplotlib app for realOffloadDragShotTest metrics."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Metrics JSON path.")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="Report output directory.")
    parser.add_argument("--no-gui", action="store_true", help="Only write report files and PNG charts.")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parents[1]
    input_path = resolve_path(project_root, args.input)
    output_dir = resolve_path(project_root, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_path.exists():
        raise SystemExit(f"Metrics file not found: {input_path}")

    data = json.loads(input_path.read_text(encoding="utf-8"))
    tasks = data.get("tasks", {})
    if not tasks:
        raise SystemExit(f"No task data found in: {input_path}")

    overall = compute_overall(tasks)
    figures, components = create_figures(tasks)

    write_summary_files(output_dir, data, tasks, overall, components)
    write_csv_exports(output_dir, tasks)
    save_figures(output_dir, figures)

    print(f"Input:   {input_path}")
    print(f"Output:  {output_dir}")
    print(f"Summary: {output_dir / 'summary.txt'}")
    print(f"CSV:     {output_dir / 'task_metrics.csv'}")
    print(f"CSV:     {output_dir / 'sample_points.csv'}")
    print("PNG charts:")
    for name in figures.keys():
        filename = name.lower().replace(" ", "_").replace("+", "plus") + ".png"
        print(f"  - {output_dir / filename}")

    if not args.no_gui:
        launch_gui(data, overall, components, figures)


if __name__ == "__main__":
    main()
