from __future__ import print_function

import argparse
import json

from demo_core import DemoStateMachine, load_config
from demo_core.logging_utils import TeeLogger, default_log_path


def build_overrides(args):
    overrides = {"runtime": {"dry_run": {}}}
    dry_run = overrides["runtime"]["dry_run"]
    if args.real:
        dry_run.update({"camera": False, "base": False, "arm": False})
    if args.dry_run:
        dry_run.update({"camera": True, "base": True, "arm": True})
    for name in ("camera", "base", "arm"):
        value = getattr(args, "{}_real".format(name))
        if value:
            dry_run[name] = False
    if args.avoidance:
        overrides["avoidance"] = {"strategy": args.avoidance}
    if not dry_run:
        overrides["runtime"].pop("dry_run")
    if not overrides["runtime"]:
        overrides.pop("runtime")
    return overrides


def run(args):
    config = load_config(
        args.parameters,
        config_path=args.config,
        overrides=build_overrides(args),
    )
    if args.print_config or args.validate_only:
        print(json.dumps(config.data, indent=2, sort_keys=True))
    if args.validate_only:
        print("[demo] configuration valid")
        return 0
    runtime = DemoStateMachine(config)
    return 0 if runtime.run(max_ticks=args.max_ticks) else 1


def main():
    parser = argparse.ArgumentParser(description="Run the event-driven JetTank demo FSM.")
    parser.add_argument("--config", default=None, help="runtime switches; defaults to config.json beside this script")
    parser.add_argument("--parameters", default=None, help="tuned values; defaults to empirical_parameters.json beside this script")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--print-config", action="store_true")
    parser.add_argument("--real", action="store_true", help="enable camera, base and arm")
    parser.add_argument("--dry-run", action="store_true", help="force all hardware dry-run")
    parser.add_argument("--camera-real", action="store_true")
    parser.add_argument("--base-real", action="store_true")
    parser.add_argument("--arm-real", action="store_true")
    parser.add_argument("--avoidance", choices=("disabled", "scripted", "tangentbug_depth"))
    parser.add_argument("--max-ticks", type=int, default=10000)
    parser.add_argument("--log-file", default=None)
    parser.add_argument("--no-log-file", action="store_true")
    args = parser.parse_args()
    if args.no_log_file:
        return run(args)
    path = args.log_file or default_log_path("demo")
    with TeeLogger(path):
        return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
