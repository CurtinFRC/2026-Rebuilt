#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
from statistics import mean


def resolve_path(project_root: Path, raw: str) -> Path:
    path = Path(raw)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def flatten_metric(tasks: dict, key: str):
    values = []
    for task in tasks.values():
        for sample in task.get("samples", []):
            value = sample.get(key)
            if value is not None:
                values.append(float(value))
    return values


def metric_stats(values):
    if not values:
        return None
    ordered = sorted(values)
    n = len(ordered)

    def pct(q):
        idx = max(0, min(n - 1, math.ceil(q * n) - 1))
        return ordered[idx]

    return {
        "count": n,
        "min": ordered[0],
        "p50": pct(0.50),
        "avg": mean(ordered),
        "p95": pct(0.95),
        "max": ordered[-1],
    }


def write_summary(output_path: Path, data: dict, tasks: dict):
    overall = {
        "rttMs": metric_stats(flatten_metric(tasks, "rttMs")),
        "serverMs": metric_stats(flatten_metric(tasks, "serverMs")),
        "execMs": metric_stats(flatten_metric(tasks, "execMs")),
        "overheadMs": metric_stats(flatten_metric(tasks, "overheadMs")),
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
    for name, stats in overall.items():
        if not stats:
            continue
        lines.append(
            f"{name}: n={stats['count']} min={stats['min']:.3f} p50={stats['p50']:.3f} "
            f"avg={stats['avg']:.3f} p95={stats['p95']:.3f} max={stats['max']:.3f}"
        )
    lines.append("")
    lines.append("Per Task")
    lines.append("--------")
    for task_id, task in tasks.items():
        summary = task.get("summary", {})
        rtt = summary.get("rtt", {})
        execs = summary.get("exec", {})
        overhead = summary.get("overhead", {})
        lines.append(
            f"{task_id}: rtt(avg/p95)={rtt.get('avgMs', float('nan')):.3f}/{rtt.get('p95Ms', float('nan')):.3f} "
            f"exec(avg)={execs.get('avgMs', float('nan')):.3f} "
            f"overhead(avg)={overhead.get('avgMs', float('nan')):.3f}"
        )

    output_path.write_text("\n".join(lines), encoding="utf-8")


def render_charts(output_dir: Path, tasks: dict):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib not available: {exc}")
        return False

    task_ids = list(tasks.keys())

    compute = sum(flatten_metric(tasks, "execMs"))
    overhead = sum(flatten_metric(tasks, "overheadMs"))
    if compute > 0 or overhead > 0:
        fig, ax = plt.subplots(figsize=(6, 6))
        labels = ["Compute (execMs)", "Overhead (rtt-serverMs)"]
        ax.pie([compute, overhead], labels=labels, autopct="%1.1f%%", startangle=90)
        ax.set_title("Overall Compute vs Overhead")
        fig.tight_layout()
        fig.savefig(output_dir / "compute_vs_overhead_pie.png", dpi=140)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 6))
    for task_id in task_ids:
        samples = tasks[task_id].get("samples", [])
        x = list(range(1, len(samples) + 1))
        y = [float(s["rttMs"]) for s in samples]
        ax.plot(x, y, marker="o", markersize=2, linewidth=1, label=task_id)
    ax.set_title("RTT Over Samples")
    ax.set_xlabel("Sample Index")
    ax.set_ylabel("RTT (ms)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(output_dir / "rtt_over_samples.png", dpi=140)
    plt.close(fig)

    p50 = []
    p95 = []
    for task_id in task_ids:
        summary = tasks[task_id].get("summary", {})
        rtt = summary.get("rtt", {})
        p50.append(float(rtt.get("p50Ms", float("nan"))))
        p95.append(float(rtt.get("p95Ms", float("nan"))))

    fig, ax = plt.subplots(figsize=(12, 5))
    x = list(range(len(task_ids)))
    width = 0.38
    ax.bar([i - width / 2 for i in x], p50, width=width, label="p50 RTT")
    ax.bar([i + width / 2 for i in x], p95, width=width, label="p95 RTT")
    ax.set_title("RTT Percentiles By Task")
    ax.set_ylabel("Latency (ms)")
    ax.set_xticks(x)
    ax.set_xticklabels(task_ids, rotation=25, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "rtt_percentiles_by_task.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 5))
    for task_id in task_ids:
        samples = tasks[task_id].get("samples", [])
        x = list(range(1, len(samples) + 1))
        y = [float(s["overheadMs"]) for s in samples if s.get("overheadMs") is not None]
        if y:
            ax.plot(x[: len(y)], y, marker="o", markersize=2, linewidth=1, label=task_id)
    ax.set_title("Overhead Over Samples")
    ax.set_xlabel("Sample Index")
    ax.set_ylabel("Overhead (ms)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(output_dir / "overhead_over_samples.png", dpi=140)
    plt.close(fig)
    return True


def main():
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Render offload latency visuals from realOffloadDragShotTest metrics JSON."
    )
    parser.add_argument(
        "--input",
        default="build/offload/real-offload-latency.json",
        help="Metrics JSON path (absolute, or relative to project root).",
    )
    parser.add_argument(
        "--output-dir",
        default="build/offload/real-offload-report",
        help="Output report directory (absolute, or relative to project root).",
    )
    args = parser.parse_args()

    input_path = resolve_path(project_root, args.input)
    output_dir = resolve_path(project_root, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_path.exists():
        raise SystemExit(f"Metrics file not found: {input_path}")

    data = json.loads(input_path.read_text(encoding="utf-8"))
    tasks = data.get("tasks", {})
    if not tasks:
        raise SystemExit(f"No task metrics found in: {input_path}")

    write_summary(output_dir / "summary.txt", data, tasks)
    charts_ok = render_charts(output_dir, tasks)

    print(f"Input:  {input_path}")
    print(f"Output: {output_dir}")
    print(f"Summary: {output_dir / 'summary.txt'}")
    if charts_ok:
        print("Charts:")
        print(f"  - {output_dir / 'compute_vs_overhead_pie.png'}")
        print(f"  - {output_dir / 'rtt_over_samples.png'}")
        print(f"  - {output_dir / 'rtt_percentiles_by_task.png'}")
        print(f"  - {output_dir / 'overhead_over_samples.png'}")
    else:
        print("Charts skipped (matplotlib not available).")


if __name__ == "__main__":
    main()
