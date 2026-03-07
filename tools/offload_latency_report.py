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


def to_f(value):
    if value is None:
        return None
    return float(value)


def pct(sorted_values, q: float) -> float:
    if not sorted_values:
        return float("nan")
    idx = max(0, min(len(sorted_values) - 1, math.ceil(q * len(sorted_values)) - 1))
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


def fmt(v):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "n/a"
    return f"{v:.3f}"


def all_metric_values(tasks: dict, key: str):
    out = []
    for task in tasks.values():
        for sample in task.get("samples", []):
            value = to_f(sample.get(key))
            if value is not None:
                out.append(value)
    return out


def write_summary_files(output_dir: Path, data: dict, tasks: dict):
    overall = {
        "rttMs": stats(all_metric_values(tasks, "rttMs")),
        "serverMs": stats(all_metric_values(tasks, "serverMs")),
        "execMs": stats(all_metric_values(tasks, "execMs")),
        "queueMs": stats(all_metric_values(tasks, "queueMs")),
        "overheadMs": stats(all_metric_values(tasks, "overheadMs")),
    }

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
    for name, s in overall.items():
        if not s:
            continue
        lines.append(
            f"{name}: n={s['count']} min={fmt(s['min'])} p50={fmt(s['p50'])} "
            f"avg={fmt(s['avg'])} p95={fmt(s['p95'])} max={fmt(s['max'])}"
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
            f"rtt(avg/p95)={fmt(to_f(rtt.get('avgMs')))}/{fmt(to_f(rtt.get('p95Ms')))} "
            f"server(avg)={fmt(to_f(server.get('avgMs')))} "
            f"exec(avg)={fmt(to_f(execs.get('avgMs')))} "
            f"overhead(avg)={fmt(to_f(overhead.get('avgMs')))}"
        )

    (output_dir / "summary.txt").write_text("\n".join(lines), encoding="utf-8")

    md = []
    md.append("# Offload Latency Report")
    md.append("")
    md.append(f"- Host: `{data.get('host')}:{data.get('port')}`")
    md.append(f"- Read timeout: `{data.get('readTimeoutMs')} ms`")
    md.append(
        f"- Warmup samples: `{data.get('warmupSamples')}`, Measured samples: `{data.get('measuredSamples')}`"
    )
    md.append("")
    md.append("## Overall")
    md.append("")
    md.append("| Metric | Count | Min | P50 | Avg | P95 | Max |")
    md.append("|---|---:|---:|---:|---:|---:|---:|")
    for metric, s in overall.items():
        if not s:
            continue
        md.append(
            f"| {metric} | {s['count']} | {fmt(s['min'])} | {fmt(s['p50'])} | {fmt(s['avg'])} | {fmt(s['p95'])} | {fmt(s['max'])} |"
        )
    md.append("")
    md.append("## Per Task (RTT / Exec / Overhead)")
    md.append("")
    md.append("| Task | RTT Avg | RTT P95 | Exec Avg | Overhead Avg | Overhead % of RTT Avg |")
    md.append("|---|---:|---:|---:|---:|---:|")
    for task_id, task in tasks.items():
        summary = task.get("summary", {})
        rtt_avg = to_f(summary.get("rtt", {}).get("avgMs"))
        rtt_p95 = to_f(summary.get("rtt", {}).get("p95Ms"))
        exec_avg = to_f(summary.get("exec", {}).get("avgMs"))
        overhead_avg = to_f(summary.get("overhead", {}).get("avgMs"))
        ratio = (
            (overhead_avg / rtt_avg * 100.0)
            if rtt_avg and overhead_avg is not None and rtt_avg > 0
            else float("nan")
        )
        md.append(
            f"| {task_id} | {fmt(rtt_avg)} | {fmt(rtt_p95)} | {fmt(exec_avg)} | {fmt(overhead_avg)} | {fmt(ratio)}% |"
        )
    (output_dir / "summary.md").write_text("\n".join(md), encoding="utf-8")


def write_csv_exports(output_dir: Path, tasks: dict):
    task_csv = output_dir / "task_metrics.csv"
    with task_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
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
            rtt = summary.get("rtt", {})
            server = summary.get("server", {})
            execs = summary.get("exec", {})
            overhead = summary.get("overhead", {})
            queue_vals = [to_f(s.get("queueMs")) for s in task.get("samples", [])]
            queue_vals = [v for v in queue_vals if v is not None]
            queue_avg = mean(queue_vals) if queue_vals else None
            w.writerow(
                [
                    task_id,
                    len(task.get("samples", [])),
                    to_f(rtt.get("minMs")),
                    to_f(rtt.get("p50Ms")),
                    to_f(rtt.get("avgMs")),
                    to_f(rtt.get("p95Ms")),
                    to_f(rtt.get("maxMs")),
                    to_f(server.get("avgMs")),
                    to_f(execs.get("avgMs")),
                    queue_avg,
                    to_f(overhead.get("avgMs")),
                ]
            )

    sample_csv = output_dir / "sample_points.csv"
    with sample_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "taskId",
                "sampleIndex",
                "rttMs",
                "serverMs",
                "execMs",
                "queueMs",
                "overheadMs",
            ]
        )
        for task_id, task in tasks.items():
            for s in task.get("samples", []):
                w.writerow(
                    [
                        task_id,
                        s.get("sampleIndex"),
                        s.get("rttMs"),
                        s.get("serverMs"),
                        s.get("execMs"),
                        s.get("queueMs"),
                        s.get("overheadMs"),
                    ]
                )


def render_charts(output_dir: Path, tasks: dict):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib not available: {exc}")
        return False

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except Exception:
        pass

    task_ids = list(tasks.keys())

    compute_sum = sum(all_metric_values(tasks, "execMs"))
    overhead_sum = sum(all_metric_values(tasks, "overheadMs"))
    if compute_sum > 0 or overhead_sum > 0:
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.pie(
            [compute_sum, overhead_sum],
            labels=["Compute (execMs)", "Overhead (rtt-serverMs)"],
            autopct="%1.1f%%",
            startangle=90,
        )
        ax.set_title("Overall Compute vs Overhead")
        fig.tight_layout()
        fig.savefig(output_dir / "compute_vs_overhead_pie.png", dpi=150)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5))
    for task_id in task_ids:
        samples = tasks[task_id].get("samples", [])
        x = list(range(1, len(samples) + 1))
        y = [to_f(s.get("rttMs")) for s in samples]
        y = [v for v in y if v is not None]
        if y:
            ax.plot(x[: len(y)], y, linewidth=1.2, label=task_id)
    ax.set_title("RTT Over Samples")
    ax.set_xlabel("Sample")
    ax.set_ylabel("RTT (ms)")
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(output_dir / "rtt_over_samples.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5))
    datasets = []
    labels = []
    for task_id in task_ids:
        vals = [to_f(s.get("rttMs")) for s in tasks[task_id].get("samples", [])]
        vals = [v for v in vals if v is not None]
        if vals:
            datasets.append(vals)
            labels.append(task_id)
    if datasets:
        ax.boxplot(datasets, labels=labels, showfliers=False)
        ax.set_title("RTT Distribution By Task")
        ax.set_ylabel("RTT (ms)")
        ax.tick_params(axis="x", labelrotation=22)
        fig.tight_layout()
        fig.savefig(output_dir / "task_rtt_boxplot.png", dpi=150)
    plt.close(fig)

    overall_rtt = all_metric_values(tasks, "rttMs")
    if overall_rtt:
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.hist(overall_rtt, bins=min(40, max(10, len(overall_rtt) // 8)))
        ax.set_title("Overall RTT Histogram")
        ax.set_xlabel("RTT (ms)")
        ax.set_ylabel("Count")
        fig.tight_layout()
        fig.savefig(output_dir / "overall_rtt_hist.png", dpi=150)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 5))
    x = list(range(len(task_ids)))
    avg_queue = []
    avg_exec = []
    avg_overhead = []
    for task_id in task_ids:
        samples = tasks[task_id].get("samples", [])
        q = [to_f(s.get("queueMs")) for s in samples]
        e = [to_f(s.get("execMs")) for s in samples]
        o = [to_f(s.get("overheadMs")) for s in samples]
        q = [v for v in q if v is not None]
        e = [v for v in e if v is not None]
        o = [v for v in o if v is not None]
        avg_queue.append(mean(q) if q else 0.0)
        avg_exec.append(mean(e) if e else 0.0)
        avg_overhead.append(mean(o) if o else 0.0)
    ax.bar(x, avg_queue, label="Queue avg (ms)")
    ax.bar(x, avg_exec, bottom=avg_queue, label="Exec avg (ms)")
    bottoms = [a + b for a, b in zip(avg_queue, avg_exec)]
    ax.bar(x, avg_overhead, bottom=bottoms, label="Overhead avg (ms)")
    ax.set_title("Average Latency Components By Task")
    ax.set_ylabel("Milliseconds")
    ax.set_xticks(x)
    ax.set_xticklabels(task_ids, rotation=25, ha="right")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "avg_latency_components_stacked.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5))
    xs = []
    ys = []
    for task in tasks.values():
        for s in task.get("samples", []):
            ex = to_f(s.get("execMs"))
            ov = to_f(s.get("overheadMs"))
            if ex is not None and ov is not None:
                xs.append(ex)
                ys.append(ov)
    if xs and ys:
        ax.scatter(xs, ys, alpha=0.5, s=14)
        ax.set_title("Exec vs Overhead (All Samples)")
        ax.set_xlabel("Exec (ms)")
        ax.set_ylabel("Overhead (ms)")
        fig.tight_layout()
        fig.savefig(output_dir / "overhead_vs_exec_scatter.png", dpi=150)
    plt.close(fig)
    return True


def main():
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Render visual offload latency reports from realOffloadDragShotTest JSON metrics."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Metrics JSON path.")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="Report output directory.")
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

    write_summary_files(output_dir, data, tasks)
    write_csv_exports(output_dir, tasks)
    charts_ok = render_charts(output_dir, tasks)

    print(f"Input:   {input_path}")
    print(f"Output:  {output_dir}")
    print(f"Summary: {output_dir / 'summary.txt'}")
    print(f"Markdown:{output_dir / 'summary.md'}")
    print(f"CSV:     {output_dir / 'task_metrics.csv'}")
    print(f"CSV:     {output_dir / 'sample_points.csv'}")
    if charts_ok:
        print("Charts:")
        print(f"  - {output_dir / 'compute_vs_overhead_pie.png'}")
        print(f"  - {output_dir / 'rtt_over_samples.png'}")
        print(f"  - {output_dir / 'task_rtt_boxplot.png'}")
        print(f"  - {output_dir / 'overall_rtt_hist.png'}")
        print(f"  - {output_dir / 'avg_latency_components_stacked.png'}")
        print(f"  - {output_dir / 'overhead_vs_exec_scatter.png'}")
    else:
        print("Charts skipped because matplotlib is unavailable.")


if __name__ == "__main__":
    main()
