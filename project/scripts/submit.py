#!/usr/bin/env python3
"""Host-side build, checks, native Gazebo/RViz recording and submission ZIP."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import zipfile
from submission_desktop import Desktop

ROOT = Path(__file__).resolve().parents[1]


def run(cmd, **kwargs):
    return subprocess.run(cmd, check=True, **kwargs)


def json_write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def capture(cmd): return subprocess.check_output(cmd, text=True).strip()


def probe_video(path):
    data = json.loads(capture(['ffprobe', '-v', 'error', '-show_format', '-show_streams', '-of', 'json', str(path)]))
    streams = [s for s in data.get('streams', []) if s.get('codec_type') == 'video']
    if not streams or float(data.get('format', {}).get('duration', 0)) < 2:
        raise RuntimeError('录像缺失有效视频流或时长不足2秒')
    # Full decode validates that the finalized MP4 can actually be played.
    run(['ffmpeg', '-v', 'error', '-xerror', '-i', str(path), '-map', '0:v:0', '-f', 'null', '-'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    return {'duration_s': float(data['format']['duration']), 'width': streams[0]['width'],
            'height': streams[0]['height'], 'codec': streams[0]['codec_name'], 'full_decode_passed': True}


def source_files(root):
    raw = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z', '--cached', '--others', '--exclude-standard'])
    roots = {'src', 'scripts', 'config', 'docs'}
    top = {'README.md', 'Dockerfile', 'docker-entrypoint.sh', '.gitignore', '.dockerignore', 'provenance.json', 'LICENSE', 'NOTICE'}
    excluded = {'.git', '.vscode', '__pycache__', '.pytest_cache', 'build', 'install', 'log'}
    for name in sorted(set(os.fsdecode(p) for p in raw.split(b'\0') if p)):
        rel = Path(name)
        if rel.is_absolute() or '..' in rel.parts or any(p in excluded for p in rel.parts): continue
        if rel.parts[0] not in roots and name not in top: continue
        file = root/rel
        if not file.exists(): continue
        if file.is_symlink() or not file.resolve().is_relative_to(root.resolve()):
            raise RuntimeError('提交源码不支持符号链接：'+name)
        if file.is_file(): yield file, 'project/'+rel.as_posix()


def package(root, output, report, archive):
    files = list(source_files(root))
    files += [(report, 'report'+report.suffix.lower())]
    for p in sorted(output.rglob('*')):
        if p.is_file(): files.append((p, 'evidence/'+p.relative_to(output).as_posix()))
    checksums = {}
    temporary = archive.with_suffix('.zip.partial')
    try:
        with zipfile.ZipFile(temporary, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=5) as z:
            for file, name in files:
                z.write(file, name)
                h = hashlib.sha256()
                with file.open('rb') as f:
                    for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
                checksums[name] = h.hexdigest()
            z.writestr('SHA256SUMS.json', json.dumps(checksums, indent=2)+'\n')
        with zipfile.ZipFile(temporary) as z:
            if z.testzip() is not None: raise RuntimeError('ZIP完整性检查失败')
        temporary.rename(archive)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    ap = argparse.ArgumentParser(description='在宿主机自动验收、录制Gazebo与RViz并打包。失败记录也会保留。')
    ap.add_argument('--student-id', required=True, help='学号或组号，只用字母、数字、下划线和短横线')
    ap.add_argument('--report', required=True, type=Path, help='已填写的作业报告（Markdown或PDF）')
    ap.add_argument('--image', help='镜像标签；默认构建本次专用标签')
    ap.add_argument('--skip-build', action='store_true', help='调试：复用--image，正式提交不要使用')
    ap.add_argument('--skip-checks', action='store_true', help='调试：跳过前置测试，结果不计正式通过')
    ap.add_argument('--timeout', type=float, default=180., help='任务超时，单位仿真秒')
    ap.add_argument('--goal-x', type=float, default=1.475, help='调试用目标X，正式提交保持默认')
    ap.add_argument('--goal-y', type=float, default=4.375, help='调试用目标Y，正式提交保持默认')
    ap.add_argument('--output-dir', type=Path, default=ROOT/'submissions')
    args = ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', args.student_id): ap.error('student-id格式不合法')
    if not math.isfinite(args.timeout) or args.timeout <= 0 or args.timeout > 1200: ap.error('timeout须在(0,1200]内')
    if not all(math.isfinite(v) for v in (args.goal_x, args.goal_y)): ap.error('目标坐标必须为有限数值')
    if args.skip_build and not args.image: ap.error('--skip-build必须同时指定--image')
    report = args.report.expanduser().resolve()
    if not report.is_file() or report.stat().st_size == 0 or report.suffix.lower() not in ('.md', '.pdf'):
        ap.error('请提供已填写的非空Markdown/PDF报告')
    for cmd in ('docker', 'git', 'ffmpeg', 'ffprobe', 'xauth'):
        if not shutil.which(cmd): ap.error('缺少宿主机命令：'+cmd+'；请按README安装依赖')
    if not os.environ.get('DISPLAY'): ap.error('请在已登录的Linux图形桌面终端运行，当前没有DISPLAY')
    desktop = Desktop(); before = desktop.windows()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:6]
    name = 'trident-submit-'+stamp.lower(); image = args.image or 'trident-control-course:submit-'+stamp.lower()
    output_parent = args.output_dir.expanduser().resolve(); output_parent.mkdir(parents=True, exist_ok=True)
    output = output_parent/(args.student_id+'-'+stamp); output.mkdir()
    archive = output_parent/(output.name+'.zip')
    metadata = {'schema_version': 1, 'student_id': args.student_id, 'created_utc': datetime.now(timezone.utc).isoformat(), 'run_id': stamp,
                'git_commit': capture(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']),
                'git_status': capture(['git', '-C', str(ROOT), 'status', '--porcelain']),
                'built_current_source': not args.skip_build, 'checks': [], 'passed': False,
                'recording': 'X11 desktop, native Gazebo and RViz side by side; no audio'}
    recorder = None; started_container = False; auth = None; recording_log = None
    try:
        if not args.skip_build:
            print('1/5 构建当前源码镜像……', flush=True)
            with (output/'build.log').open('w') as f: run(['docker', 'build', '-t', image, str(ROOT)], stdout=f, stderr=subprocess.STDOUT)
        metadata['image_id'] = capture(['docker', 'image', 'inspect', image, '--format', '{{.Id}}'])
        if not args.skip_checks:
            print('2/5 运行算法、实时地图和闭环自检……', flush=True)
            checks = [('student_map', ['python3', '-m', 'pytest', '-q', 'src/nav/obstacle-map/nav_obstacle_map/tests/test_student_map.py']),
                      ('live_map', ['python3', 'scripts/check_live_map.py']),
                      ('closed_loop', ['python3', 'scripts/check_closed_loop.py'])]
            for label, command in checks:
                with (output/(label+'.log')).open('w') as f:
                    try:
                        p = subprocess.run(['docker', 'run', '--rm', '--name', name+'-'+label,
                                            '-e', 'ROS_DOMAIN_ID=88', '-e', 'ROS_LOCALHOST_ONLY=1',
                                            '-e', 'PYTHONPATH=/opt/nav_ws/src/nav/obstacle-map/nav_obstacle_map/python',
                                            image]+command, stdout=f, stderr=subprocess.STDOUT, timeout=180)
                        code = p.returncode
                    except subprocess.TimeoutExpired:
                        subprocess.run(['docker', 'rm', '-f', name+'-'+label], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        code = 124
                metadata['checks'].append({'name': label, 'exit_code': code, 'passed': code == 0})
                print('  '+label+(': PASS' if code == 0 else ': FAIL（仍将录制任务）'), flush=True)
        print('3/5 启动独立Gazebo与RViz，准备录像……', flush=True)
        auth = tempfile.TemporaryDirectory(prefix='trident-submit-xauth-')
        authority = Path(auth.name)/'authority'; authority.touch(); authority.chmod(0o600)
        raw = subprocess.check_output(['xauth', 'nlist', os.environ['DISPLAY']])
        raw = b'\n'.join(b'ffff'+line[4:] for line in raw.splitlines())+b'\n'
        run(['xauth', '-f', str(authority), 'nmerge', '-'], input=raw, stdout=subprocess.DEVNULL)
        if not authority.stat().st_size: raise RuntimeError('无法取得当前桌面的Xauthority')
        gpu = subprocess.check_output([sys.executable, str(ROOT/'scripts/gpu_args.py')])
        gpu_args = [os.fsdecode(x) for x in gpu.split(b'\0') if x]
        cmd = ['docker', 'run', '--detach', '--init', '--name', name, '--shm-size', '1g',
               '-e', 'ROS_DOMAIN_ID=88', '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'DISPLAY',
               '-e', 'XAUTHORITY=/tmp/course.xauth', '-e', 'QT_QPA_PLATFORM=xcb', '-e', 'QT_X11_NO_MITSHM=1',
               '-e', 'ROS_LOG_DIR=/submission/ros-logs', '-e', 'SUBMIT_UID='+str(os.getuid()), '-e', 'SUBMIT_GID='+str(os.getgid()),
               '--volume', '/tmp/.X11-unix:/tmp/.X11-unix:ro', '--volume', str(authority)+':/tmp/course.xauth:ro',
               '--volume', str(output)+':/submission']+gpu_args+[image, 'python3', 'scripts/run_submission_case.py',
               '--output', '/submission', '--timeout', str(args.timeout), '--goal-x', str(args.goal_x), '--goal-y', str(args.goal_y)]
        run(cmd, stdout=subprocess.DEVNULL); started_container = True
        deadline = time.monotonic()+100
        while not (output/'ready.json').exists():
            if (output/'result.json').exists(): raise RuntimeError('仿真启动失败，参见result.json与模块日志')
            if capture(['docker', 'inspect', name, '--format', '{{.State.Running}}']) != 'true':
                raise RuntimeError('仿真容器提前退出，参见container.log')
            if time.monotonic() > deadline: raise RuntimeError('等待仿真启动超时')
            time.sleep(.3)
        metadata['windows'] = desktop.arrange(before)
        video = output/'final.mp4'; recording_log = (output/'recording.log').open('w')
        recorder = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-y', '-f', 'x11grab',
                                     '-framerate', '12', '-video_size', f'{desktop.width}x{desktop.height}',
                                     '-draw_mouse', '0', '-i', os.environ['DISPLAY'], '-vf', 'scale=1920:-2',
                                     '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-crf', '23',
                                     '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)],
                                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=recording_log)
        time.sleep(1)
        if recorder.poll() is not None: raise RuntimeError('桌面录制启动失败，参见recording.log')
        (output/'start.signal').touch()
        print('4/5 自动发送目标并录制；请保持两个窗口可见，勿手动发goal或遮挡窗口。', flush=True)
        deadline = time.monotonic()+max(60., args.timeout*3)+100
        while not (output/'result.json').exists():
            if recorder.poll() is not None: raise RuntimeError('录制进程提前退出')
            if capture(['docker', 'inspect', name, '--format', '{{.State.Running}}']) != 'true':
                raise RuntimeError('任务容器提前退出，参见container.log')
            if time.monotonic() > deadline: raise RuntimeError('任务超过宿主机等待上限')
            time.sleep(.3)
        recorder.communicate(input=b'q\n', timeout=30); metadata['recorder_exit_code'] = recorder.returncode; recorder = None
        recording_log.close(); recording_log = None
        metadata['video'] = probe_video(video)
        metadata['case'] = json.loads((output/'result.json').read_text())
        metadata['passed'] = (metadata['case']['passed'] and metadata['case']['full_field_goal'] and metadata['recorder_exit_code'] == 0 and not args.skip_build
                              and len(metadata['checks']) == 3 and all(c['passed'] for c in metadata['checks']))
    except (Exception, KeyboardInterrupt) as exc:
        metadata['error'] = str(exc) or '用户中断'
        print('本次运行未通过：'+metadata['error'], file=sys.stderr)
    finally:
        if recorder is not None:
            try: recorder.communicate(input=b'q\n', timeout=15)
            except (subprocess.TimeoutExpired, BrokenPipeError): recorder.kill(); recorder.wait()
        if recording_log: recording_log.close()
        if started_container:
            with (output/'container.log').open('w') as f:
                subprocess.run(['docker', 'logs', name], stdout=f, stderr=subprocess.STDOUT)
            subprocess.run(['docker', 'rm', '-f', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for label in ('student_map', 'live_map', 'closed_loop'):
            subprocess.run(['docker', 'rm', '-f', name+'-'+label], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if auth: auth.cleanup()
        desktop.close()
        (output/'start.signal').unlink(missing_ok=True)
        json_write(output/'manifest.json', metadata)
    print('5/5 压缩源码、报告、录像与日志……', flush=True)
    package(ROOT, output, report, archive)
    print(('验收通过' if metadata['passed'] else '验收未通过（保留失败记录）')+'\n提交包：'+str(archive)+'\n运行目录：'+str(output))
    return 0 if metadata['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
