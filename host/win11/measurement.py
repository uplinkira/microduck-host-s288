"""S288 输出端 SI 实测；调用方暂停普通控制，禁止伪装 XL330/Go1 辨识。"""
from dataclasses import asdict, dataclass
from datetime import datetime
import csv
import json
import math
from pathlib import Path
import statistics
import struct
import tempfile
import threading
import time

TRAJECTORIES = ('sin_time_square', 'up_and_down', 'lift_and_drop')


def trajectory(name, t):
    """独立实现 BAM 数学轨迹，返回(rad,通电)。抬起释放阶段必须零增益。"""
    t = max(0., min(6., t))
    smooth = lambda s: s*s*(3-2*s)
    if name == 'sin_time_square':
        return math.sin(t*t), True
    if name == 'up_and_down':
        return (smooth(t/3) if t <= 3 else 1-.2*smooth((t-3)/3))*math.pi/2, True
    if name == 'lift_and_drop':
        return -math.pi/2*smooth(min(t/2, 1)), t < 2
    raise ValueError('未知轨迹')


def distribution(xs):
    if not xs:
        return dict(count=0, mean=None, p99=None, max=None, std=None)
    xs = sorted(xs)
    return dict(count=len(xs), mean=statistics.mean(xs), p99=xs[math.ceil(.99*len(xs))-1],
                max=xs[-1], std=statistics.pstdev(xs))


def summarize(rows, rounds):
    samples = [r for r in rows if r['stage'] == 'sample']
    valid = [r for r in samples if r['feedback_valid']]
    times = {}
    for row in valid:
        times.setdefault((row['trial'], row['id']), []).append(row['tx_s'])
    gaps = [b-a for ts in times.values() for a,b in zip(ts, ts[1:])]
    errors = [r['position']-r['goal_position'] for r in valid if r['torque_enable']]
    return dict(samples=len(samples), valid=len(valid), missing=len(samples)-len(valid),
                valid_rate=len(valid)/len(samples) if samples else None,
                roundtrip_ms=distribution([r['roundtrip_s']*1000 for r in valid]),
                all_attempt_ms=distribution([r['roundtrip_s']*1000 for r in samples]),
                round_ms=distribution([r['elapsed_s']*1000 for r in rounds]),
                complete_rounds=sum(r['valid']==r['expected'] for r in rounds),
                interval_ms=distribution([g*1000 for g in gaps]),
                effective_hz=1/statistics.mean(gaps) if gaps and statistics.mean(gaps)>0 else None,
                tracking_rmse_rad=math.sqrt(statistics.mean(e*e for e in errors)) if errors else None,
                per_id={str(mid):dict(attempts=sum(r['id']==mid for r in samples),
                                     valid=sum(r['id']==mid for r in valid),
                                     roundtrip_ms=distribution([r['roundtrip_s']*1000 for r in valid if r['id']==mid]))
                        for mid in sorted({r['id'] for r in samples})},
                note='主机请求至有效反馈，不是电机内部ACK/机械延迟；不自动判定跟踪滞后<40ms')


@dataclass(frozen=True)
class MeasurementConfig:
    kind: str = 'latency'
    ids: tuple = (7,)
    cycles: int = 1000
    duration_s: float = 5.0
    hz: float = 50.0
    gains: tuple = (3.0,)
    kd: float = .3
    amplitude_deg: float = 3.0
    trajectories: tuple = ('sin_time_square',)
    mass: float | None = None
    arm_mass: float | None = None
    length: float | None = None
    pendulum_direction: int = 1
    fixture_confirmed: bool = False
    torque_limit: float = .6
    temperature_limit: float = 70.0
    operator: str = ''
    firmware: str = ''
    fixture: str = ''

    def validate(self):
        if self.kind not in ('latency','position','trajectory'):
            raise ValueError('采集类型无效')
        if not self.ids or len(set(self.ids))!=len(self.ids) or any(type(m) is not int or not 0<=m<15 for m in self.ids):
            raise ValueError('ID须为不重复的0–14')
        if self.kind!='latency' and len(self.ids)!=1:
            raise ValueError('主动运动试验仅允许单台')
        if any(not math.isfinite(x) for x in (self.duration_s,self.hz,self.kd,self.amplitude_deg,self.torque_limit,self.temperature_limit,*self.gains)):
            raise ValueError('参数须为有限数')
        if not 1<=self.cycles<=10000 or not 1<=self.hz<=200 or not .1<=self.duration_s<=60:
            raise ValueError('次数1–10000，频率1–200Hz，时长0.1–60s')
        if not self.gains or len(self.gains)>3 or any(not 0<k<=2128 for k in self.gains) or not 0<=self.kd<=21.2:
            raise ValueError('输出端增益超协议范围；最多三档，不是XL330固件值')
        if not 0<self.torque_limit<=.6 or not 0<self.temperature_limit<=70:
            raise ValueError('试验停止阈值：力矩不超过0.6N·m，壳温不超过70℃')
        if self.kind!='latency' and (not self.fixture_confirmed or not self.operator.strip() or not self.firmware.strip()):
            raise ValueError('主动试验须确认支架/范围并填写操作者、固件版本')
        if self.kind=='position' and not 0<abs(self.amplitude_deg)<=5:
            raise ValueError('小幅阶跃试验须为非零且不超过±5°；此约束仅用于试验，不限制普通关节调试')
        if self.kind=='trajectory':
            if not self.trajectories or any(x not in TRAJECTORIES for x in self.trajectories):
                raise ValueError('轨迹选择无效')
            if any(x is None or not math.isfinite(x) or x<=0 for x in (self.mass,self.arm_mass,self.length)):
                raise ValueError('必须填写正的配重kg、均匀杆质量kg、长度m')
            if self.pendulum_direction not in (-1,1) or not self.fixture.strip():
                raise ValueError('须填写摆锤零位/方向与台架说明')
        return self


class MeasurementRunner:
    def __init__(self, bus, io_lock, output_dir, stop_event=None, *, clock=time.perf_counter, wait=None):
        self.bus, self.io_lock, self.output_dir = bus, io_lock, Path(output_dir)
        self.stop = stop_event or threading.Event()
        self.clock, self.wait = clock, wait or self.stop.wait
        self.progress = ''

    def _request(self, mid, target, kp, kd, stage, trial=0, enabled=False, direction=1):
        if self.stop.is_set() and stage!='release':
            raise InterruptedError('用户停止')
        tx=self.clock(); fb=None; error=''
        watchdog = 0 if stage in ('preflight','release') else 1
        try:
            fb=self.bus.transact(mid,1,0.0,0.0,target,kp,kd,timeout_bit=watchdog)
        except Exception as exc:
            error=str(exc)
        rx=self.clock()
        fields=('outputPos','outputSpd','outputTor','vol','Temp','MError')
        valid=bool(fb) and fb.get('motor_id')==mid and all(isinstance(fb.get(k),(int,float)) and math.isfinite(fb[k]) for k in fields)
        if fb and not valid:
            error=error or '反馈ID错误或必要数值缺失/非有限'
        from motor_sweep import build_control_packet,RATIO
        packet=build_control_packet(mid,1,watchdog,0,0,target,kp,kd)
        _,_,qcode,kpc,kdc=struct.unpack('<hhihh',packet[4:16])
        center=self.centers.get(mid,0.0)
        row=dict(stage=stage,trial=trial,id=mid,tx_s=tx-self.started,rx_s=rx-self.started,
                 timestamp=(tx+rx)/2-self.started,roundtrip_s=rx-tx,
                 goal_position=direction*(target-center),q_cmd_abs_rad=target,
                 dq_cmd_rad_s=0.0,tau_ff_Nm=0.0,kp_Nm_rad=kp,kd_Nm_s_rad=kd,
                 q_wire_rad=qcode*2*math.pi/(RATIO*32768),kp_wire=kpc*RATIO**2/1280000,kd_wire=kdc*RATIO**2/128000000,
                 torque_enable=enabled,feedback_valid=valid,error=error,position=None,speed=None,
                 torque_feedback_Nm=None,input_volts=None,temp=None,merror=None,timeout=None)
        if valid:
            row.update(position=direction*(fb['outputPos']-center),speed=direction*fb['outputSpd'],
                       torque_feedback_Nm=direction*fb['outputTor'],input_volts=fb['vol'],temp=fb['Temp'],
                       merror=fb['MError'],timeout=fb.get('timeout',0))
        self.rows.append(row)
        self.raw.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n'); self.raw.flush()
        if self.writer is None:
            self.writer=csv.DictWriter(self.csvfile,fieldnames=list(row)); self.writer.writeheader()
        self.writer.writerow(row); self.csvfile.flush()
        return row

    def _check(self,row,active):
        if not row['feedback_valid']:
            if active: raise RuntimeError('无有效反馈，中止主动试验')
            return
        if row['merror'] or row['temp']>=self.config.temperature_limit or not 6.4<=row['input_volts']<=12.6:
            raise RuntimeError('电机故障/壳温/供电越界')
        if active and (row['timeout'] or abs(row['torque_feedback_Nm'])>self.config.torque_limit):
            raise RuntimeError('超时锁存或试验力矩阈值触发')
        if active and row['torque_enable']:
            error = abs(row['position']-row['goal_position'])
            if error > math.radians(45):
                raise RuntimeError('位置跟踪误差超过45°，立即中止台架试验')
            if error > math.radians(12):
                if self.large_error_since is None:
                    self.large_error_since = self.clock()
                elif self.clock()-self.large_error_since >= .2:
                    raise RuntimeError('位置跟踪误差持续超过12°达0.2秒，中止台架试验')
            else:
                self.large_error_since = None

    def _return_zero(self,mid,center,direction,kp,trial):
        row=self._request(mid,center,0,0,'return_probe',trial,False,direction); self._check(row,True)
        q0=row['position']; duration=max(.5,abs(q0)/math.radians(10)); start=self.clock(); i=0
        while self.clock()-start<duration+.5:
            q=q0*(1-min((self.clock()-start)/duration,1))
            row=self._request(mid,center+direction*q,kp,self.config.kd,'return',trial,True,direction)
            self._check(row,True); i+=1
            self.wait(max(0,start+i/self.config.hz-self.clock()))
        if abs(row['position'])>math.radians(2):
            raise RuntimeError('返回台架零位误差超过2°，请人工复核')

    def run(self,config,*,centers=None,metadata=None):
        self.config=config.validate(); self.centers=dict(centers or {}); active=config.kind!='latency'
        if active and any(m not in self.centers or not math.isfinite(self.centers[m]) for m in config.ids):
            raise ValueError('主动采集须有输出端标定零位')
        from motor_sweep import POS_MAX
        for c in self.centers.values():
            bound=abs(math.radians(config.amplitude_deg)) if config.kind=='position' else math.pi/2
            if not math.isfinite(c) or abs(c)+bound>POS_MAX: raise ValueError('目标超出协议位置编码范围')
        self.output_dir.mkdir(parents=True,exist_ok=True)
        folder=Path(tempfile.mkdtemp(prefix=datetime.now().strftime('s288_%Y%m%d_%H%M%S_'),dir=self.output_dir))
        result=dict(schema='s288_measurement_v2',motor='s288',config=asdict(config),metadata=metadata or {},
                    centers=self.centers,status='running',errors=[],bam_compatible=False,
                    note='输出端SI；未提供PWM/load。S288拟合须适配，不能假装XL330/Go1。',release_feedback={})
        def save():
            (folder/'session.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        save(); self.started=self.clock(); self.rows=[]; self.rounds=[]; self.writer=None
        self.large_error_since = None
        with (folder/'samples.jsonl').open('w',encoding='utf-8') as self.raw, (folder/'samples.csv').open('w',encoding='utf-8-sig',newline='') as self.csvfile, self.io_lock:
            try:
                for mid in config.ids:
                    row=self._request(mid,self.centers.get(mid,0),0,0,'preflight'); self._check(row,active)
                    if active and abs(row['position'])>math.radians(2): raise RuntimeError('当前离标定零位超过2°，先人工摆正')
                if config.kind=='latency':
                    for cycle in range(config.cycles):
                        start=self.clock(); valid=0
                        for mid in config.ids:
                            row=self._request(mid,0,0,0,'sample'); self._check(row,False); valid+=int(row['feedback_valid'])
                        self.rounds.append(dict(cycle=cycle,elapsed_s=self.clock()-start,valid=valid,expected=len(config.ids)))
                        self.progress=f'往返测量 {cycle+1}/{config.cycles}轮'
                else:
                    mid=config.ids[0]; center=self.centers[mid]; trial=0
                    direction=config.pendulum_direction if config.kind=='trajectory' else 1
                    names=config.trajectories if config.kind=='trajectory' else ('position',)
                    for kp in config.gains:
                        for name in names:
                            trial+=1; duration=6.0 if name!='position' else config.duration_s
                            start=self.clock(); i=0; trial_rows=[]
                            try:
                                while self.clock()-start<duration:
                                    t=self.clock()-start
                                    q,on=trajectory(name,t) if name!='position' else (math.radians(config.amplitude_deg) if .2*duration<=t<.8*duration else 0.,True)
                                    row=self._request(mid,center+direction*q,kp if on else 0,config.kd if on else 0,'sample',trial,on,direction)
                                    trial_rows.append(dict(row,timestamp=t+row['roundtrip_s']/2))
                                    self._check(row,True); i+=1; self.progress=f'第{trial}组 {name} Kp={kp:g} {t:.1f}/{duration:g}s'
                                    self.wait(max(0,start+i/config.hz-self.clock()))
                            finally:
                                data=dict(schema='s288_trial_v1',motor='s288',trajectory=name,kp=kp,damping=config.kd,
                                          mass=config.mass,arm_mass=config.arm_mass,length=config.length,
                                          config=asdict(config),entries=trial_rows,bam_compatible=False,
                                          trial_completed=(self.clock()-start>=duration and not self.stop.is_set()))
                                (folder/f'trial_{trial:02d}.json').write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
                            self._return_zero(mid,center,direction,kp,trial)
                result['status']='completed'
            except InterruptedError as exc:
                result['status']='cancelled'; result['errors'].append(str(exc))
            except Exception as exc:
                result['status']='failed'; result['errors'].append(str(exc))
            finally:
                # 取消/异常时不再回零，所有试验ID均发三次零增益；写日志失败也继续卸力。
                for mid in config.ids:
                    ok=False
                    for _ in range(3):
                        try: ok=self._request(mid,0,0,0,'release')['feedback_valid'] or ok
                        except Exception as exc: result['errors'].append(f'ID{mid}卸力记录失败：{exc}')
                    result['release_feedback'][str(mid)]=ok
                if not all(result['release_feedback'].values()):
                    result['errors'].append('卸力无完整有效反馈，实机状态无法保证，请检查供电/通信')
                    if result['status']=='completed': result['status']='failed'
        result['summary']=summarize(self.rows,self.rounds); result['rounds']=self.rounds
        result['elapsed_s']=self.clock()-self.started; save()
        return folder,result
