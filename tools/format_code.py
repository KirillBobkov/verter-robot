#!/usr/bin/env python3
"""Format maintained source files; never follow runtime or archived trees."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
ROOTS = ('ros_ws/src', 'services', 'firmware', 'tools', 'deploy', 'tests', 'design', 'web/frontend', 'docs')
EXCLUDED = {
    'archive',
    'third_party',
    'vendor',
    'lib',
    'build',
    'build_artifacts',
    'install',
    'log',
    'node_modules',
    'dist',
    '__pycache__',
    '.git',
    '.pio',
    '.pytest_cache',
}
PRETTIER_SUFFIXES = {'.js', '.ts', '.tsx', '.css', '.html', '.json', '.yaml', '.yml', '.xml', '.urdf', '.rviz'}


def source_files(root=ROOT):
    paths = []
    for folder in ROOTS:
        for directory, children, names in os.walk(root / folder, followlinks=False):
            children[:] = [
                name
                for name in children
                if name not in EXCLUDED
                and not name.startswith('.venv')
                and not (Path(directory) / name).is_symlink()
                and (Path(directory) / name).relative_to(root).as_posix() != 'services/wifi-connect/ui'
            ]
            for name in names:
                path = Path(directory) / name
                if path.is_symlink() or name == 'package-lock.json':
                    continue
                if path.suffix in PRETTIER_SUFFIXES | {'.py', '.cpp', '.h', '.sh'} or path == root / 'tools/verter':
                    paths.append(path)
    paths.append(root / 'config/assets_manifest.json')
    return sorted(path for path in paths if path.is_file())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check without modifying files')
    args = parser.parse_args()
    programs = {name: shutil.which(name) for name in ('ruff', 'clang-format', 'prettier', 'node')}
    missing = [name for name, path in programs.items() if not path]
    if missing:
        parser.error('Missing tools: ' + ', '.join(missing) + '; see docs/OPERATIONS.md')
    files = source_files()
    python = [str(path) for path in files if path.suffix == '.py']
    cpp = [str(path) for path in files if path.suffix in {'.cpp', '.h'}]
    web = [str(path) for path in files if path.suffix in PRETTIER_SUFFIXES]
    shell = [str(path) for path in files if path.suffix == '.sh' or path.name == 'verter']
    commands = []
    if python:
        commands.append([programs['ruff'], 'format', *(['--check'] if args.check else []), *python])
    if cpp:
        commands.append([programs['clang-format'], *(['--dry-run', '--Werror'] if args.check else ['-i']), *cpp])
    # Resolve plugins next to the Prettier installation, not in application dependencies.
    package = Path(programs['prettier']).resolve().parents[1]
    plugins = []
    for name in ('@prettier/plugin-xml', 'prettier-plugin-sh'):
        script = 'console.log(require.resolve(process.argv[1], {paths: [process.argv[2]]}))'
        plugin = subprocess.check_output([programs['node'], '-e', script, name, str(package)], text=True).strip()
        plugins.extend(['--plugin', plugin])
    if web or shell:
        commands.append(
            [
                programs['prettier'],
                '--check' if args.check else '--write',
                '--config',
                str(ROOT / '.prettierrc.json'),
                *plugins,
                *web,
                *shell,
            ]
        )
    for command in commands:
        subprocess.run(command, cwd=ROOT, check=True)
    print(f'Checked {len(files)} maintained source/configuration files')


if __name__ == '__main__':
    main()
