"""Download missing Ubuntu/ROS dependencies into the workspace without sudo."""
from pathlib import Path
import os
import subprocess

workspace = Path(__file__).resolve().parents[1]
deps = workspace / '.deps'
debs = deps / 'debs'
root = deps / 'root'
debs.mkdir(parents=True, exist_ok=True)
root.mkdir(parents=True, exist_ok=True)
result = subprocess.run(
    ['apt-get', '-s', 'install', 'ros-humble-behaviortree-cpp',
     'ros-humble-gazebo-ros-pkgs', 'libgazebo-dev'],
    check=True, text=True, capture_output=True)
packages = [line.split()[1] for line in result.stdout.splitlines() if line.startswith('Inst ')]
(deps / 'apt-packages.txt').write_text('\n'.join(packages) + '\n')
if packages:
    subprocess.run(['apt-get', 'download', *packages], cwd=debs, check=True)
for deb in sorted(debs.glob('*.deb')):
    subprocess.run(['dpkg-deb', '-x', str(deb), str(root)], check=True)

# Development packages refer to libraries already installed on the host.
for link in root.rglob('*'):
    if link.is_symlink() and not link.exists():
        target = Path(os.readlink(link))
        original = Path('/') / link.relative_to(root)
        candidate = target if target.is_absolute() else original.parent / target
        if candidate.exists():
            link.unlink()
            link.symlink_to(candidate)
libdir = root / 'usr/lib/x86_64-linux-gnu'
libdir.mkdir(parents=True, exist_ok=True)
for source in Path('/usr/lib/x86_64-linux-gnu').glob('*.so*'):
    dest = libdir / source.name
    if source.exists() and not dest.exists() and not dest.is_symlink():
        dest.symlink_to(source)
prefix = root / 'opt/ros/humble'
if prefix.is_dir():
    (prefix / 'local_setup.bash').write_text("""#!/usr/bin/env bash
_bot_dep_prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for _bot_dep_setup in "$_bot_dep_prefix"/share/*/local_setup.bash; do
  [[ -f "$_bot_dep_setup" ]] && source "$_bot_dep_setup"
done
unset _bot_dep_setup _bot_dep_prefix
""")
print(f'Local dependencies ready: {root}')
