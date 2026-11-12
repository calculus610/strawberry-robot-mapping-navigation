"""Install the approved profile only while the robot manager is idle without a lease."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent
REMOTE = r'''
from pathlib import Path
from datetime import datetime,timezone
from stat import S_ISREG
import fcntl,hashlib,json,os,subprocess,sys,tempfile,time
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
packet=json.loads(sys.stdin.buffer.read())
source=Path('/home/wheeltec/demo-repository/web_manager_ws1/src/wheeltec_web_manager/wheeltec_web_manager/external_manual.py')
original_root=Path('/home/wheeltec/manual_native180_20261022')
experiment=Path('/home/wheeltec/strawberry_octomap_20261112')
service='wheeltec-web-manager.service'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def check(ok,msg):
 if not ok:raise RuntimeError(msg)
def systemctl(*args):return subprocess.run(['systemctl','--user',*args],check=True,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE).stdout.strip()
def safe(s):
 h=s.get('cloud_control_health',{})
 return (s.get('state')=='IDLE' and s.get('mode')=='none' and s.get('active') is False
  and s.get('pending_mode')=='none' and not s.get('stop_phase') and s.get('safety_locked') is False
  and s.get('cloud_control_active') is False and h.get('valid') is False and h.get('remaining_ms')==0
  and isinstance(h.get('heartbeat_age_sec'),(int,float)) and 0<=h['heartbeat_age_sec']<1.0
  and s.get('cloud_cleanup_complete') is True and s.get('external_conflicts')==[])
rclpy.init();node=rclpy.create_node('web_manager_patch_observer');samples=[]
def receive(m):
 try:samples.append((time.monotonic(),json.loads(m.data)))
 except ValueError:pass
sub=node.create_subscription(String,'/robot_mode/status',receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def status(timeout=6):
 samples.clear();begin=time.monotonic()
 while time.monotonic()-begin<timeout:
  rclpy.spin_once(node,timeout_sec=0.1)
  if len(samples)>=3 and samples[-1][0]-samples[0][0]>=0.5:return samples[-1][1]
 raise RuntimeError('No fresh manager status')
def summary(s):return {k:s.get(k) for k in ('state','mode','active','pending_mode','safety_locked','cloud_control_active','cloud_cleanup_complete','cloud_control_health','external_conflicts')}
try:
 with (original_root/'session.lock').open('rb') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  before=status();check(safe(before),'Manager not idle or lease not released; patch aborted')
  print('WEB_DEPLOY_PREFLIGHT_IDLE_NO_LEASE',file=sys.stderr,flush=True)
  check(systemctl('is-active',service)=='active','Manager not active')
  old_pid=systemctl('show',service,'-p','MainPID','--value')
  check(Path('/proc',old_pid).stat().st_uid==os.getuid(),'Manager UID differs')
  env=Path('/proc',old_pid,'environ').read_bytes().split(b'\0')
  domains=[value for value in env if value.startswith(b'ROS_DOMAIN_ID=')]
  check(domains in ([],[b'ROS_DOMAIN_ID=0']),'Manager ROS domain differs from zero/default')
  check(S_ISREG((original_root/'session.lock').stat(follow_symlinks=False).st_mode),'Session lock is not regular')
  check(sha(source)==packet['old_sha256'],'Production source changed; patch aborted')
  check(not source.is_symlink(),'Production source unexpectedly symlinked')
  install_package=Path('/home/wheeltec/demo-repository/web_manager_ws1/build/wheeltec_web_manager/wheeltec_web_manager')
  check((install_package/'external_manual.py').resolve()==source,'Editable install does not load this source')
  safe_publishers_before=sorted((p.node_namespace,p.node_name) for p in node.get_publishers_info_by_topic('/cmd_vel_safe'))
  check(safe_publishers_before==[('/','wheeltec_cloud_safety')],'Unexpected safe velocity publisher')
  old_bytes=source.read_bytes();mode=source.stat().st_mode&0o777
  backup=Path(tempfile.mkdtemp(prefix='web_profile_backup_',dir=experiment))
  (backup/'external_manual.py').write_bytes(old_bytes);(backup/'external_manual.py').chmod(mode)
  staged=source.with_name('external_manual.py.strawberry_tmp')
  with staged.open('xb') as stream:stream.write(packet['source'].encode());stream.flush();os.fsync(stream.fileno())
  staged.chmod(mode)
  check(sha(staged)==packet['new_sha256'],'Staging hash mismatch')
  check(safe(status()),'Manager changed state before installation; existing service untouched')
  staged.replace(source)
  print('WEB_SOURCE_INSTALLED backup='+str(backup),file=sys.stderr,flush=True)
  systemctl('restart',service)
  check(systemctl('is-active',service)=='active','Manager did not restart')
  new_pid=systemctl('show',service,'-p','MainPID','--value')
  check(new_pid not in ('0',old_pid),'Manager PID did not change')
  after=None;deadline=time.monotonic()+20
  while time.monotonic()<deadline:
   try:after=status(4)
   except RuntimeError:continue
   if safe(after):break
  check(after is not None and safe(after),'Manager did not return to idle/no-lease/healthy heartbeat')
  safe_publishers_after=sorted((p.node_namespace,p.node_name) for p in node.get_publishers_info_by_topic('/cmd_vel_safe'))
  check(safe_publishers_after==safe_publishers_before,'Safe velocity publisher graph changed')
  check(sha(source)==packet['new_sha256'],'Installed source hash mismatch')
  record={'deployed_utc':datetime.now(timezone.utc).isoformat(),'production_file':str(source),'old_sha256':packet['old_sha256'],'new_sha256':packet['new_sha256'],'backup':str(backup),'mode':oct(mode),'service':service,'old_pid':old_pid,'new_pid':new_pid,'before':summary(before),'after':summary(after),'safe_velocity_publishers':safe_publishers_after,'mapping_started':False,'control_lease_requested':False,'motion_commands_sent':False}
  (experiment/'web_profile_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
  print(json.dumps(record,ensure_ascii=False))
finally:
 node.destroy_node();rclpy.shutdown()
'''

baseline = (ROOT / 'production_external_manual.baseline.py').read_bytes()
candidate = (ROOT / 'external_manual.candidate.py').read_bytes()
packet = json.dumps({'old_sha256': hashlib.sha256(baseline).hexdigest(),
                     'new_sha256': hashlib.sha256(candidate).hexdigest(),
                     'source': candidate.decode()}).encode()
script = ('source /opt/ros/humble/setup.bash\n'
          'export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1\n'
          'exec python3 -B -c ' + shlex.quote(REMOTE))
config = ROOT.parents[2] / 'Tailscale部署_20261021/ssh-tailnet.conf'
command = ['ssh', '-F', str(config), '-o', 'ConnectTimeout=12', '-o', 'ServerAliveInterval=10',
           '-o', 'ServerAliveCountMax=3', 'wheeltec-tailnet',
           'bash --noprofile --norc -c ' + shlex.quote(script)]
result = subprocess.run(command, input=packet, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
print(result.stderr.decode(errors='replace'), end='')
print(result.stdout.decode(errors='replace'), end='')
if result.returncode:
    raise SystemExit(result.returncode)
(ROOT / 'deployment.json').write_bytes(result.stdout)
