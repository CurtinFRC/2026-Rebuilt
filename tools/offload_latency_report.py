#!/usr/bin/env python3
import argparse
import csv
import json
import math
import webbrowser
from pathlib import Path
from statistics import mean

try:
    import plotly.graph_objects as go
    import plotly.io as pio
except Exception as exc:
    raise SystemExit(
        "plotly is required for the interactive dashboard. Install with: "
        "pip install -r requirements.txt"
    ) from exc


DEFAULT_INPUT = "build/offload/real-offload-latency.json"
DEFAULT_OUTPUT_DIR = "build/offload/real-offload-report"
DEFAULT_HTML_NAME = "interactive_report.html"


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
            row = {
                "task": task_id,
                "sample": int(sample.get("sampleIndex", 0)),
                "rtt": to_float(sample.get("rttMs")),
                "server": to_float(sample.get("serverMs")),
                "exec": to_float(sample.get("execMs")),
                "queue": to_float(sample.get("queueMs")),
                "overhead": to_float(sample.get("overheadMs")),
            }
            rows.append(row)
    rows.sort(key=lambda r: (r["task"], r["sample"]))
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
    queue_vals = [r["queue"] for r in rows if r["queue"] is not None]
    exec_vals = [r["exec"] for r in rows if r["exec"] is not None]
    server_vals = [r["server"] for r in rows if r["server"] is not None]
    client_net_vals = [r["overhead"] for r in rows if r["overhead"] is not None]

    queue_avg = mean(queue_vals) if queue_vals else 0.0
    exec_avg = mean(exec_vals) if exec_vals else 0.0
    server_avg = mean(server_vals) if server_vals else 0.0
    client_net_avg = mean(client_net_vals) if client_net_vals else 0.0

    server_framework_avg = max(0.0, server_avg - queue_avg - exec_avg)
    total_avg = queue_avg + exec_avg + server_framework_avg + client_net_avg

    return {
        "queue": queue_avg,
        "exec": exec_avg,
        "serverFramework": server_framework_avg,
        "clientNetwork": client_net_avg,
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
        f"serverFrameworkMs(avg): {fmt(components['serverFramework'])} "
        f"({components['serverFramework'] / total * 100.0:.1f}%)"
    )
    lines.append(
        f"clientNetworkMs(avg): {fmt(components['clientNetwork'])} "
        f"({components['clientNetwork'] / total * 100.0:.1f}%)"
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
        f"| Server Framework | {fmt(components['serverFramework'])} | {components['serverFramework'] / total * 100.0:.1f}% |"
    )
    markdown.append(
        f"| Client + Network | {fmt(components['clientNetwork'])} | {components['clientNetwork'] / total * 100.0:.1f}% |"
    )
    markdown.append("")
    markdown.append("## Per Task")
    markdown.append("")
    markdown.append("| Task | Samples | RTT Avg | RTT P95 | Server Avg | Exec Avg | Overhead Avg |")
    markdown.append("|---|---:|---:|---:|---:|---:|---:|")
    for task_id, task in tasks.items():
        summary = task.get("summary", {})
        markdown.append(
            f"| {task_id} | {len(task.get('samples', []))} | "
            f"{fmt(to_float(summary.get('rtt', {}).get('avgMs')))} | "
            f"{fmt(to_float(summary.get('rtt', {}).get('p95Ms')))} | "
            f"{fmt(to_float(summary.get('server', {}).get('avgMs')))} | "
            f"{fmt(to_float(summary.get('exec', {}).get('avgMs')))} | "
            f"{fmt(to_float(summary.get('overhead', {}).get('avgMs')))} |"
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


def build_component_pie(tasks: dict):
    components = compute_component_averages(tasks)
    values = [
        components["queue"],
        components["exec"],
        components["serverFramework"],
        components["clientNetwork"],
    ]
    labels = ["Queue", "Execute", "Server Framework", "Client + Network"]
    colors = ["#4C956C", "#2C6E49", "#F4A259", "#BC4B51"]

    fig = go.Figure(
        data=[
            go.Pie(
                labels=labels,
                values=values,
                marker={"colors": colors},
                text=[f"{value:.3f} ms avg" for value in values],
                textinfo="label+percent+text",
                hovertemplate="%{label}<br>avg=%{value:.3f} ms<br>%{percent}<extra></extra>",
                sort=False,
            )
        ]
    )
    fig.update_layout(
        title="Average RTT Composition (ms + percentage)",
        margin={"l": 20, "r": 20, "t": 60, "b": 20},
        height=470,
    )
    return fig, components


def build_rtt_line(tasks: dict):
    fig = go.Figure()
    for task_id, task in tasks.items():
        samples = sorted(task.get("samples", []), key=lambda s: int(s.get("sampleIndex", 0)))
        x_values = [int(sample.get("sampleIndex", 0)) for sample in samples]
        y_values = [to_float(sample.get("rttMs")) for sample in samples]
        fig.add_trace(
            go.Scattergl(
                x=x_values,
                y=y_values,
                mode="lines+markers",
                marker={"size": 5},
                line={"width": 1.5},
                name=task_id,
                hovertemplate="task=%{fullData.name}<br>sample=%{x}<br>rtt=%{y:.3f} ms<extra></extra>",
            )
        )
    fig.update_layout(
        title="RTT Over Samples",
        xaxis_title="Sample Index",
        yaxis_title="RTT (ms)",
        legend_title="Task",
        hovermode="x unified",
        height=430,
        margin={"l": 40, "r": 20, "t": 60, "b": 40},
    )
    fig.update_xaxes(rangeslider={"visible": True})
    return fig


def build_component_stacked_per_task(tasks: dict):
    task_ids = list(tasks.keys())
    queue = []
    execute = []
    server_framework = []
    client_network = []
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

        queue.append(queue_avg)
        execute.append(exec_avg)
        server_framework.append(framework_avg)
        client_network.append(overhead_avg)

    fig = go.Figure()
    fig.add_trace(go.Bar(name="Queue", x=task_ids, y=queue, marker_color="#4C956C"))
    fig.add_trace(go.Bar(name="Execute", x=task_ids, y=execute, marker_color="#2C6E49"))
    fig.add_trace(
        go.Bar(name="Server Framework", x=task_ids, y=server_framework, marker_color="#F4A259")
    )
    fig.add_trace(
        go.Bar(name="Client + Network", x=task_ids, y=client_network, marker_color="#BC4B51")
    )
    fig.update_layout(
        title="Average Latency Components By Task",
        barmode="stack",
        xaxis_title="Task",
        yaxis_title="Average ms",
        height=430,
        margin={"l": 40, "r": 20, "t": 60, "b": 100},
        xaxis={"tickangle": 18},
    )
    return fig


def build_rtt_box(tasks: dict):
    fig = go.Figure()
    for task_id, task in tasks.items():
        values = task_metric_values(task, "rttMs")
        fig.add_trace(
            go.Box(
                y=values,
                name=task_id,
                boxmean=True,
                boxpoints="outliers",
                jitter=0.25,
                pointpos=0,
            )
        )
    fig.update_layout(
        title="RTT Distribution By Task",
        yaxis_title="RTT (ms)",
        height=430,
        margin={"l": 40, "r": 20, "t": 60, "b": 100},
        xaxis={"tickangle": 18},
    )
    return fig


def build_exec_vs_overhead(tasks: dict):
    x_values = []
    y_values = []
    colors = []
    for task_id, task in tasks.items():
        for sample in task.get("samples", []):
            exec_ms = to_float(sample.get("execMs"))
            overhead_ms = to_float(sample.get("overheadMs"))
            if exec_ms is None or overhead_ms is None:
                continue
            x_values.append(exec_ms)
            y_values.append(overhead_ms)
            colors.append(task_id)

    fig = go.Figure()
    fig.add_trace(
        go.Scattergl(
            x=x_values,
            y=y_values,
            mode="markers",
            marker={"size": 8, "opacity": 0.7},
            text=colors,
            hovertemplate="task=%{text}<br>exec=%{x:.3f} ms<br>overhead=%{y:.3f} ms<extra></extra>",
        )
    )
    fig.update_layout(
        title="Exec vs Client+Network Overhead (All Samples)",
        xaxis_title="Exec (ms)",
        yaxis_title="Client + Network Overhead (ms)",
        height=430,
        margin={"l": 40, "r": 20, "t": 60, "b": 40},
    )
    return fig


def build_overall_table(data: dict, overall: dict, components: dict):
    rows = []
    for metric, summary in overall.items():
        if not summary:
            continue
        rows.append(
            "<tr>"
            f"<td>{metric}</td>"
            f"<td>{summary['count']}</td>"
            f"<td>{fmt(summary['min'])}</td>"
            f"<td>{fmt(summary['p50'])}</td>"
            f"<td>{fmt(summary['avg'])}</td>"
            f"<td>{fmt(summary['p95'])}</td>"
            f"<td>{fmt(summary['max'])}</td>"
            "</tr>"
        )
    total = components["total"] if components["total"] > 0 else 1.0
    split_rows = [
        ("Queue", components["queue"]),
        ("Execute", components["exec"]),
        ("Server Framework", components["serverFramework"]),
        ("Client + Network", components["clientNetwork"]),
    ]
    split_html = "".join(
        f"<tr><td>{name}</td><td>{value:.3f}</td><td>{value / total * 100.0:.1f}%</td></tr>"
        for name, value in split_rows
    )
    return f"""
<div class="meta">
  <div><strong>Host:</strong> {data.get('host')}:{data.get('port')}</div>
  <div><strong>Read Timeout:</strong> {data.get('readTimeoutMs')} ms</div>
  <div><strong>Warmup:</strong> {data.get('warmupSamples')}</div>
  <div><strong>Measured:</strong> {data.get('measuredSamples')}</div>
</div>
<h3>Overall Metrics</h3>
<table>
  <thead><tr><th>Metric</th><th>Count</th><th>Min</th><th>P50</th><th>Avg</th><th>P95</th><th>Max</th></tr></thead>
  <tbody>{''.join(rows)}</tbody>
</table>
<h3>Average RTT Component Split</h3>
<table>
  <thead><tr><th>Component</th><th>Avg ms</th><th>Percent</th></tr></thead>
  <tbody>{split_html}</tbody>
</table>
"""


def render_html_dashboard(output_dir: Path, data: dict, tasks: dict, overall: dict):
    pie_fig, components = build_component_pie(tasks)
    rtt_line_fig = build_rtt_line(tasks)
    stacked_fig = build_component_stacked_per_task(tasks)
    box_fig = build_rtt_box(tasks)
    scatter_fig = build_exec_vs_overhead(tasks)
    overview_html = build_overall_table(data, overall, components)

    html_path = output_dir / DEFAULT_HTML_NAME
    sections = [
        ("Average RTT Composition", pie_fig),
        ("RTT Over Samples", rtt_line_fig),
        ("Average Components By Task", stacked_fig),
        ("RTT Distribution By Task", box_fig),
        ("Exec vs Overhead", scatter_fig),
    ]
    figure_blocks = "\n".join(
        f"<section><h2>{title}</h2>{pio.to_html(fig, include_plotlyjs=False, full_html=False, config={'responsive': True, 'displaylogo': False})}</section>"
        for title, fig in sections
    )

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Offload Latency Dashboard</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    body {{
      font-family: "Segoe UI", "Helvetica Neue", Helvetica, Arial, sans-serif;
      margin: 0;
      background: linear-gradient(180deg, #f3f8f4 0%, #ffffff 100%);
      color: #1f2933;
    }}
    header {{
      padding: 20px 24px 12px 24px;
      border-bottom: 1px solid #d9e2ec;
      background: #ffffffcc;
      backdrop-filter: blur(4px);
      position: sticky;
      top: 0;
      z-index: 2;
    }}
    h1 {{
      margin: 0;
      font-size: 22px;
    }}
    main {{
      padding: 16px 24px 28px 24px;
      display: grid;
      gap: 20px;
    }}
    section {{
      background: #ffffff;
      border: 1px solid #d9e2ec;
      border-radius: 12px;
      padding: 14px;
      box-shadow: 0 4px 10px rgba(31, 41, 51, 0.05);
    }}
    h2 {{
      margin: 0 0 8px 0;
      font-size: 16px;
    }}
    h3 {{
      margin: 14px 0 8px 0;
      font-size: 14px;
    }}
    table {{
      border-collapse: collapse;
      width: 100%;
      margin-top: 4px;
      font-size: 13px;
    }}
    th, td {{
      border: 1px solid #d9e2ec;
      padding: 6px 8px;
      text-align: right;
    }}
    th:first-child, td:first-child {{
      text-align: left;
    }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      font-size: 13px;
      margin-bottom: 6px;
    }}
  </style>
</head>
<body>
  <header>
    <h1>Offload Latency Dashboard</h1>
  </header>
  <main>
    <section>
      <h2>Run Metadata + Overall Metrics</h2>
      {overview_html}
    </section>
    {figure_blocks}
  </main>
</body>
</html>"""
    html_path.write_text(html, encoding="utf-8")
    return html_path, components


def main():
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Interactive offload latency dashboard for realOffloadDragShotTest metrics."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Metrics JSON path.")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="Report output directory.")
    parser.add_argument("--no-open", action="store_true", help="Do not open the dashboard in a browser.")
    args = parser.parse_args()

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
    components = compute_component_averages(tasks)

    write_summary_files(output_dir, data, tasks, overall, components)
    write_csv_exports(output_dir, tasks)
    html_path, _ = render_html_dashboard(output_dir, data, tasks, overall)

    print(f"Input:    {input_path}")
    print(f"Output:   {output_dir}")
    print(f"Summary:  {output_dir / 'summary.txt'}")
    print(f"Markdown: {output_dir / 'summary.md'}")
    print(f"CSV:      {output_dir / 'task_metrics.csv'}")
    print(f"CSV:      {output_dir / 'sample_points.csv'}")
    print(f"GUI:      {html_path}")

    if not args.no_open:
        webbrowser.open(html_path.as_uri(), new=2)


if __name__ == "__main__":
    main()
