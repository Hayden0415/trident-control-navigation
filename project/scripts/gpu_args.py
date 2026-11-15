#!/usr/bin/env python3
"""Local graphics setup for the course container; never changes Docker daemon configuration."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys

def options_from_spec(path):
    spec = json.loads(Path(path).read_text())
    if spec.get('kind') != 'nvidia.com/gpu':
        raise RuntimeError('Expected an NVIDIA CDI inventory')
    edits = spec.get('containerEdits', {})
    mounts = {entry['containerPath']: entry['hostPath'] for entry in edits.get('mounts', [])}
    # Resolve the SONAME aliases that CDI normally creates with a hook. Bind
    # their existing host targets directly, leaving the container filesystem
    # and host configuration unchanged.
    for hook in edits.get('hooks', []):
        args = hook.get('args', [])
        if 'create-symlinks' not in args:
            continue
        for index, argument in enumerate(args):
            if argument != '--link' or index + 1 >= len(args):
                continue
            source, target = args[index + 1].split('::', 1)
            host = (Path(target).parent / source).resolve()
            if host.is_file():
                mounts[target] = str(host)
    nodes = list(edits.get('deviceNodes', []))
    for device in spec.get('devices', []):
        if device['name'] == 'all':
            nodes.extend(device.get('containerEdits', {}).get('deviceNodes', []))
    result, groups = [], set()
    for node in {item['path'] for item in nodes}:
        if not Path(node).exists():
            raise RuntimeError(f'GPU device disappeared: {node}')
        result.extend(['--device', f'{node}:{node}:rw'])
        groups.add(Path(node).stat().st_gid)
    for group in sorted(groups):
        result.extend(['--group-add', str(group)])
    for target, source in sorted(mounts.items()):
        if not Path(source).is_file() or not Path(target).is_absolute():
            raise RuntimeError('GPU inventory contains a missing or non-file driver mount')
        if ',' in source or ',' in target:
            raise RuntimeError('GPU driver mount path contains a comma')
        result.extend(['--mount', f'type=bind,src={source},dst={target},readonly'])
    result += ['--env', '__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
               '--env', '__GLX_VENDOR_LIBRARY_NAME=nvidia', '--env', 'NVIDIA_VISIBLE_DEVICES=void',
               '--env', 'LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu']
    return result


def main():
    if shutil.which('nvidia-smi'):
        runtimes=json.loads(subprocess.check_output(['docker','info','--format','{{json .Runtimes}}'],text=True))
        if 'nvidia' in runtimes:
            args=['--gpus','all','--env','NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display']
        elif shutil.which('nvidia-ctk'):
            with tempfile.TemporaryDirectory(prefix='course-gpu-') as tmp:
                path=Path(tmp)/'spec.json'
                subprocess.run(['nvidia-ctk','cdi','generate','--format=json','--output',str(path)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                args=options_from_spec(path)
        else:
            raise RuntimeError('NVIDIA显卡需要宿主机的NVIDIA Container Toolkit（nvidia-ctk）支持，请配置后重试。')
    else:
        devices=list(Path('/dev/dri').glob('renderD*'))+list(Path('/dev/dri').glob('card*'))
        # 虚拟机 virtio-gpu/virgl 无法驱动 Gazebo 的 Ogre2 渲染（Create scene 抛异常），强制 CPU 软件渲染
        args=['--env','LIBGL_ALWAYS_SOFTWARE=1']
        for device in devices:
            args+=['--device',str(device),'--group-add',str(device.stat().st_gid)]
    sys.stdout.buffer.write(b'\0'.join(x.encode() for x in args)+(b'\0' if args else b''))


if __name__=='__main__':
    try:main()
    except (RuntimeError,subprocess.CalledProcessError) as exc:
        print(str(exc),file=sys.stderr);sys.exit(1)
