import ctypes, json, threading, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from adrive.data import DataBundle
from adrive.models import Point, PlanningError
from adrive.planner import Planner
from adrive.planning_worker import IsolatedPlanner
from adrive.scheduling import LoopWaiter


def main():
    bundle = DataBundle.load('data')
    call = ctypes.WinDLL('kernel32').GetCurrentProcessId
    results = {}
    for cls in (Planner, IsolatedPlanner):
        cancel = threading.Event()
        def work():
            count = 0
            while not cancel.is_set():
                try:
                    cls(bundle).plan(Point(0, -9162.8125, 350.1875), Point(0, -8967.125, 511.1875), cancel=cancel)
                except PlanningError:
                    pass
                count += 1
            return count
        samples = []
        waiter = LoopWaiter()
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(work)
            until = time.monotonic() + 3
            while time.monotonic() < until:
                began = time.monotonic()
                for _ in range(30):
                    call()
                waiter.wait(.005)
                samples.append((time.monotonic()-began)*1000)
            cancel.set()
            count = future.result(timeout=10)
        waiter.close()
        values = sorted(samples)
        results[cls.__name__] = dict(samples=len(values), plans=count, median_ms=values[len(values)//2], p95_ms=values[int(len(values)*.95)], max_ms=max(values), over_180=sum(v>180 for v in values), over_250=sum(v>250 for v in values))
    Path('runtime/planning-isolation-probe.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))

if __name__ == '__main__':
    main()

