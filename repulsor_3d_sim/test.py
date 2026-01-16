from __future__ import annotations
import os
import time
import socket
from typing import List, Tuple, Dict
from ntcore import NetworkTableInstance, MultiSubscriber

CLIENT_NAME = os.environ.get("NT_CLIENT_NAME", "nt_probe")

POSE_BASE = os.environ.get("NT_POSE_BASE", "/AdvantageKit/RealOutputs/Odometry/Robot")
POSE_X_KEY = os.environ.get("NT_POSE_X_KEY", "translation/x")
POSE_Y_KEY = os.environ.get("NT_POSE_Y_KEY", "translation/y")
POSE_T_KEY = os.environ.get("NT_POSE_THETA_KEY", "rotation/yaw")

MAX_DEPTH = int(os.environ.get("NT_MAX_DEPTH", "7"))
PRINT_KEYS_LIMIT = int(os.environ.get("NT_PRINT_KEYS_LIMIT", "60"))
TOPIC_LIST_LIMIT = int(os.environ.get("NT_TOPIC_LIST_LIMIT", "120"))

def norm(p: str) -> str:
    p = (p or "").strip()
    if p == "/":
        return ""
    while p.startswith("/"):
        p = p[1:]
    while p.endswith("/"):
        p = p[:-1]
    return p

def join(a: str, b: str) -> str:
    a = norm(a)
    b = norm(b)
    if not a:
        return b
    if not b:
        return a
    return a + "/" + b

def local_ips() -> List[str]:
    ips = set()
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None):
            ip = info[4][0]
            if ":" not in ip and not ip.startswith("169.254."):
                ips.add(ip)
    except Exception:
        pass
    ips.add("127.0.0.1")
    ips.add("localhost")
    return sorted(ips)

def connect(server: str) -> NetworkTableInstance:
    inst = NetworkTableInstance.getDefault()
    inst.stopClient()
    inst.setServerTeam(4788)
    inst.startClient4(CLIENT_NAME)
    # inst.startDSClient()
    inst.setServer(server, NetworkTableInstance.kDefaultPort4)

    t0 = time.time()
    while time.time() - t0 < 2.0 and not inst.isConnected():
        time.sleep(0.05)
    return inst

def score(keys: List[str]) -> Tuple[int, int, int]:
    fv = 0
    rv = 0
    pose = 0
    for k in keys:
        if k.startswith("object_") or k.startswith("extrinsics/"):
            fv += 2
        if k.startswith("obs_"):
            rv += 3
        if k == POSE_X_KEY or k == POSE_Y_KEY or k == POSE_T_KEY:
            pose += 10
    return fv, rv, pose

def topic_table_and_key(topic_name: str) -> Tuple[str, str]:
    # topic_name like "/AdvantageKit/RealOutputs/..."
    tn = topic_name.strip()
    if not tn.startswith("/"):
        tn = "/" + tn
    parts = [p for p in tn.split("/") if p]
    if not parts:
        return "", ""
    if len(parts) == 1:
        return "", parts[0]
    table = "/".join(parts[:-1])
    key = parts[-1]
    return table, key

def probe(server: str):
    inst = connect(server)
    print(f"\n=== SERVER {server} connected={inst.isConnected()} ===")

    ms = MultiSubscriber(inst, ["/AdvantageKit/RealOutputs/Odometry/Robot"])
    time.sleep(0.35)

    try:
        conns = inst.getConnections()
        if conns:
            for c in conns:
                print("conn:", c.remote_id, c.remote_ip, c.remote_port, c.last_update)
        else:
            print("conn: (none listed)")
    except Exception as e:
        print("conn: error", repr(e))

    # ---- List topics (more reliable than getSubTables/getKeys in NT4) ----
    topics = inst.getTopics("/")
    print("Topics discovered:", len(topics))
    for t in topics[:min(TOPIC_LIST_LIMIT, len(topics))]:
        try:
            print("  ", t.getName(), "-", t.getTypeString())
        except Exception:
            # Some builds may not expose type string nicely; ignore
            print("  ", t.getName())
    if len(topics) > TOPIC_LIST_LIMIT:
        print("  ...")

    # Build a "table -> keys" map from topic names
    table_keys: Dict[str, List[str]] = {}
    for t in topics:
        name = t.getName()
        table, key = topic_table_and_key(name)
        table_keys.setdefault(table, []).append(key)

    # Keep table paths within MAX_DEPTH for reporting
    def depth_ok(table: str) -> bool:
        if not table:
            return True
        return table.count("/") < MAX_DEPTH  # since table is normalized without leading "/"

    paths = sorted([p for p in table_keys.keys() if depth_ok(p)], key=lambda x: (x.count("/"), x))

    best_fv = ("", -1)
    best_rv = ("", -1)
    best_pose = ("", -1)

    hits = []
    for p in paths:
        keys = sorted(set(table_keys.get(p, [])))
        if not keys:
            continue
        fv, rv, pose = score(keys)
        if fv > best_fv[1]:
            best_fv = (p, fv)
        if rv > best_rv[1]:
            best_rv = (p, rv)
        if pose > best_pose[1]:
            best_pose = (p, pose)
        if fv > 0 or rv > 0 or pose > 0:
            hits.append((p, fv, rv, pose, keys))

    print("Best guesses:", "FV", best_fv, "RV", best_rv, "POSE", best_pose)

    if hits:
        print("\nTop matches:")
        for (p, fv, rv, pose, keys) in sorted(hits, key=lambda x: (-(x[1]+x[2]+x[3]), x[0]))[:12]:
            name = p or "(root)"
            print(f"\n[{name}] fv={fv} rv={rv} pose={pose} keys={len(keys)}")
            for k in keys[:PRINT_KEYS_LIMIT]:
                print("  ", k)
            if len(keys) > PRINT_KEYS_LIMIT:
                print("  ...")
    else:
        print("\nNo matching keys found on this server.")

    # ---- Optional: explicitly subscribe to pose topics and print values ----
    base = POSE_BASE.rstrip("/")
    x_topic = inst.getDoubleTopic(base + "/" + POSE_X_KEY)
    y_topic = inst.getDoubleTopic(base + "/" + POSE_Y_KEY)
    t_topic = inst.getDoubleTopic(base + "/" + POSE_T_KEY)

    x_sub = x_topic.subscribe(0.0)
    y_sub = y_topic.subscribe(0.0)
    t_sub = t_topic.subscribe(0.0)

    time.sleep(0.15)
    print("\nPose subscribe check:")
    print("  base:", base)
    print("  x =", x_sub.get(), "y =", y_sub.get(), "theta =", t_sub.get())

def main():
    candidates = []
    env_server = os.environ.get("NT_SERVER", "").strip()
    if env_server:
        candidates.append(env_server)
    candidates.extend(local_ips())

    seen = set()
    uniq = []
    for c in candidates:
        if c not in seen:
            seen.add(c)
            uniq.append(c)

    print("Candidate servers:", uniq)
    for s in uniq[:8]:
        probe(s)

if __name__ == "__main__":
    main()
