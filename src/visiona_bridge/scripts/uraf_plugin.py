#!/usr/bin/env python3
"""URAF plugin CLI - PLAN.md section 20 (uraf plugin install/list)."""

import argparse
import json
import sys
from pathlib import Path

from visiona_bridge.uraf.plugin_registry import PluginRegistry


def main():
    parser = argparse.ArgumentParser(description="URAF plugin manager")
    sub = parser.add_subparsers(dest="cmd")

    sub.add_parser("list", help="List installed plugins")

    install_p = sub.add_parser("install", help="Install plugin from manifest path")
    install_p.add_argument("path", help="Path to plugin.yaml or plugin directory")

    enable_p = sub.add_parser("enable", help="Enable a plugin")
    enable_p.add_argument("name")

    disable_p = sub.add_parser("disable", help="Disable a plugin")
    disable_p.add_argument("name")

    uninstall_p = sub.add_parser("uninstall", help="Uninstall a plugin")
    uninstall_p.add_argument("name")

    args = parser.parse_args()
    reg = PluginRegistry()

    if args.cmd == "list":
        print(json.dumps(reg.list_plugins(), indent=2))
    elif args.cmd == "install":
        path = Path(args.path)
        manifest = path / "plugin.yaml" if path.is_dir() else path
        entry = reg.install_from_manifest(manifest)
        print(json.dumps(entry, indent=2))
    elif args.cmd == "enable":
        reg.set_enabled(args.name, True)
    elif args.cmd == "disable":
        reg.set_enabled(args.name, False)
    elif args.cmd == "uninstall":
        ok = reg.uninstall(args.name)
        sys.exit(0 if ok else 1)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
