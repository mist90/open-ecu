#!/usr/bin/env python3
"""Report mechanical truth, not filtered speed. Rise times are first crossings, not settling."""
import csv
import itertools
import sys


def summarize(rows):
    initial = rows[0][1]
    target = rows[0][2]
    span = target - initial
    speeds = [r[1] for r in rows]
    direction = 1 if span >= 0 else -1
    first = next((r[0] for r in rows if direction*(r[1]-initial) >= .9*abs(span)), None)
    settle = None
    dwell = None
    for ms, v, *_ in rows:
        if abs(v-target) <= .02*target:
            if dwell is None:
                dwell = ms
            if ms-dwell >= 3000:
                settle = dwell
                break
        else:
            dwell = None
    tail = [r for r in rows if r[0]>=rows[-1][0]+2-3000]
    return dict(t90_s=None if first is None else (first-rows[0][0])/1000,
                tsettle_s=None if settle is None else (settle-rows[0][0])/1000,
                initial_rps=initial, peak_error_rpm=max(0,max(direction*(v-target) for v in speeds))*60,
                tail_pp_rpm=60*(max(r[1] for r in tail)-min(r[1] for r in tail)),
                tail_max_error_rpm=60*max(abs(r[1]-target) for r in tail),
                peak_iq_A=max(abs(r[3]) for r in rows), peak_phase_A=rows[-1][4])


def analyze(path):
    result = []
    with open(path) as f:
        src = csv.DictReader(f)
        for key, group in itertools.groupby(src, lambda r: (r["kg"],r["kp"],r["ki"],r["scene"])):
            kg, kp, ki, scene = key
            rows = [(int(r["ms"]),float(r["wheel_rps"]),float(r["target_rps"]),float(r["torque_iq_A"]),
                     float(r["peak_phase_A"])) for r in group]
            assert rows[0][0]==0 and all(b[0]-a[0]==2 for a,b in zip(rows, rows[1:]))
            for j, (_, samples) in enumerate(itertools.groupby(rows, lambda r: r[2])):
                window = list(samples)
                if scene in ("pulse", "hold-pulse"):
                    # The fixed target does not make the load change a reference step. Keep the
                    # launch metrics separate from both sides of that independently imposed event.
                    res = summarize([r for r in window if r[0] < 10000])
                    if scene == "hold-pulse":
                        for field in ("t90_s", "tsettle_s", "peak_error_rpm"):
                            res[field] = None
                    after = summarize([r for r in window if r[0] >= 12000])
                    res.update({k: after[k] for k in ("tail_pp_rpm","tail_max_error_rpm","peak_phase_A")})
                    res["peak_iq_A"] = max(abs(r[3]) for r in window)
                    res["loaded_min_rps"] = min(r[1] for r in window if 10000<=r[0]<12000)
                    res["release_max_rps"] = max(r[1] for r in window if 12000<=r[0]<14000)
                    res["recovery_s"] = after["tsettle_s"]
                else:
                    res = summarize(window)
                result.append(dict(kg=kg,kp=kp,ki=ki,scene=scene,step=j,target=window[0][2],**res))
    return result


if __name__ == "__main__":
    rows = analyze(sys.argv[1])
    if "--check" in sys.argv[2:]:
        # Deliberately use actual wheel motion and broad physical acceptance windows.
        # The 100/500/1000 Hz comparison can move a Hall edge by a sample.
        for r in rows:
            light = r["kg"] == "0"
            if r["scene"] == "quiet":
                low, high = (.7, 2.) if light else (4., 7.)
                assert r["t90_s"] is not None and low < r["t90_s"] < high, ("launch", r)
                assert r["tsettle_s"] is not None and r["tail_pp_rpm"] < 5, ("settled", r)
            if r["scene"] == "sweep":
                assert r["tail_pp_rpm"] < 6 and r["tail_max_error_rpm"] < 4, ("chatter", r)
            if r["scene"] in ("3to4","5to4"):
                assert r["t90_s"] is not None and r["t90_s"] < (1. if light else 5.), ("rolling step",r)
                assert r["tsettle_s"] is not None and r["tsettle_s"] < (3. if light else 14.), ("rolling recovery",r)
                assert r["tail_max_error_rpm"] < 4 and r["peak_error_rpm"] < 30, ("rolling accuracy",r)
            if r["scene"] == "hold-pulse" and light:
                assert r["loaded_min_rps"] > 2.5 and r["release_max_rps"] < 5.1 and r["tail_pp_rpm"] < 5, ("pulse",r)
    writer = csv.DictWriter(sys.stdout, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
    writer.writeheader()
    for row in rows:
        writer.writerow({k:round(v,5) if isinstance(v,float) else v for k,v in row.items()})
