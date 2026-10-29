"""CPU-heavy planning runs outside the capture/control interpreter.

The supervisor owns cancellation and reaps the child, including native searches.
Only locally created temporary files are unpickled; there is no network input.
"""
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import threading

from .data import DataBundle
from .models import PlanningError
from .planner import Planner


class IsolatedPlanner:
    def __init__(self, bundle):
        self.bundle = bundle

    def plan(self, start, goal, *, cancel=None, **options):
        cancel = cancel if cancel is not None else threading.Event()
        if cancel.is_set():
            raise PlanningError('cancelled', '规划已取消')
        with tempfile.TemporaryDirectory(prefix='adrive-plan-') as directory:
            request, response = (Path(directory) / name for name in ('request', 'response'))
            # Production bundles are loaded in the child, avoiding a large copy
            # or JSON decode while the controller is monitoring safety deadlines.
            source = self.bundle.source_path
            with request.open('wb') as stream:
                pickle.dump((source, None if source else self.bundle, start, goal, options), stream)
            process = subprocess.Popen(
                [sys.executable, '-m', 'adrive.planning_worker', str(request), str(response)],
                cwd=Path(__file__).resolve().parent.parent,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            try:
                while process.poll() is None:
                    if cancel.wait(.01):
                        raise PlanningError('cancelled', '规划已取消')
                if cancel.is_set():
                    raise PlanningError('cancelled', '规划已取消')
                if process.returncode or not response.exists():
                    raise PlanningError('worker_failed', '独立规划进程异常退出，未生成路线')
                with response.open('rb') as stream:
                    route, error = pickle.load(stream)
                if error:
                    raise PlanningError(*error)
                return route
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait()


def main():
    request, response = map(Path, sys.argv[1:])
    try:
        with request.open('rb') as stream:
            source, bundle, start, goal, options = pickle.load(stream)
        if source:
            bundle = DataBundle.load(source)
        result = Planner(bundle).plan(start, goal, **options), None
    except PlanningError as exc:
        result = None, (exc.code, str(exc))
    except Exception as exc:
        result = None, ('worker_failed', f'独立规划失败：{exc}')
    with response.open('wb') as stream:
        pickle.dump(result, stream)


if __name__ == '__main__':
    main()
