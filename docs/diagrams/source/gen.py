# Generates DOT sources for the cmd 16 / 18 slide diagrams.
import html, os, sys
OUT = sys.argv[1]
C = dict(  # fill, border, text   (deck theme colors)
  entry=('#0E2841', '#0E2841', '#FFFFFF'),
  cmd=('#156082', '#0E2841', '#FFFFFF'),
  sense=('#DCEFF8', '#0F9ED5', '#0E2841'),
  ctrl=('#FCE6D8', '#E97132', '#0E2841'),
  act=('#DDF0D5', '#196B24', '#0E2841'),
  log=('#EEEEEE', '#7F7F7F', '#0E2841'),
  stop=('#F6D5D5', '#C00000', '#0E2841'),
  note=('#FFFFFF', '#9FB3C8', '#404040'),
  dec=('#FFF4CC', '#BF9000', '#0E2841'),
)
MONO = 'JetBrains Mono'
def esc(s): return html.escape(s, quote=False)
def lab(func=None, lines=(), title=None, fs=20, mono_lines=()):
    parts = []
    if title: parts.append(f'<B>{esc(title)}</B>')
    if func: parts.append(f'<FONT FACE="{MONO}" POINT-SIZE="{fs}"><B>{esc(func)}</B></FONT>')
    for l in mono_lines: parts.append(f'<FONT FACE="{MONO}" POINT-SIZE="{fs-3}">{esc(l)}</FONT>')
    for l in lines: parts.append(f'<FONT POINT-SIZE="{fs-3}">{esc(l)}</FONT>')
    return '<' + '<BR/>'.join(parts) + '>'
def N(id, kind, func=None, lines=(), title=None, shape='box', mono_lines=(), extra=''):
    f, b, t = C[kind]
    st = 'filled,rounded' if shape == 'box' else 'filled'
    return (f'  {id} [shape={shape} style="{st}" fillcolor="{f}" color="{b}" fontcolor="{t}" penwidth=2 '
            f'label={lab(func, lines, title, mono_lines=mono_lines)} {extra}];\n')
def E(a, b, label=None, extra=''):
    l = f' label=<<FONT POINT-SIZE="17"><B>{esc(label)}</B></FONT>>' if label else ''
    return f'  {a} -> {b} [{l} {extra}];\n'
HEAD = '''digraph G {
  graph [fontname="Inter" fontsize=22 bgcolor="white" pad=0.3 nodesep=0.35 ranksep=0.45 %s];
  node  [fontname="Inter" fontsize=20 margin="0.18,0.08"];
  edge  [fontname="Inter" color="#404040" penwidth=2 arrowsize=0.9];
'''
def CL(id, title, body, extra=''):
    return (f'  subgraph cluster_{id} {{ label=<<B>{esc(title)}</B>> labeljust=l style="rounded,filled" '
            f'fillcolor="#F7F9FB" color="#9FB3C8" penwidth=1.5 fontcolor="#0E2841" fontsize=22 {extra}\n{body}  }}\n')
def write(name, s):
    open(os.path.join(OUT, name + '.dot'), 'w').write(s + '}\n')

# ---------------------------------------------------------------- 01 overview
s = HEAD % 'rankdir=TB newrank=true compound=true nodesep=0.5'
s += N('PMM', 'entry', 'process_main_menu()', ['loop() checks USB and XBee'])
s += N('GET', 'entry', 'get_command_from_ground_station()', ["readStringUntil('\\n') → toInt()"])
s += N('SW', 'entry', None, ['switch', '(command)'], shape='diamond', extra='margin="0.02,0.02"')
s += E('PMM','GET') + E('GET','SW')
s += N('C16', 'cmd', 'lab8_mag_heading_hold()', ['thrusters hold the heading inside ±5°'], title='cmd 16')
s += N('C18', 'cmd', 'lab8_wheel_hold_dump()', ['RW holds heading while a fan drains it'], title='cmd 18')
s += E('SW','C16','16') + E('SW','C18','18')
s += CL('sense', '1 · Sense',
  N('IMU','sense','lab8_read_imu()',['ICM-20948 getAGMT(): gyro_Z, mag_X, mag_Y']) +
  N('HDG','sense','lab8_mag_heading()',['atan2(mag_X − off_X, mag_Y − off_Y)']) +
  N('ENC','sense','lab8_measure_RW_rpm()',['encoder counts → RPM  (cmd 18 only)']))
s += CL('ctrl', '2 · Decide',
  N('WRAP','ctrl','lab8_wrap180()',['error = target − heading']) +
  N('BB','ctrl','lab8_hold_fan_state()',['bang-bang fan choice on predicted stop error']) +
  N('TBL','ctrl','lab8_rw_pwm_from_rpm()',['RW speed-curve lookup  (cmd 18 only)']))
s += CL('act', '3 · Actuate',
  N('THR','act','thrusters_set(+1 / 0 / −1)',['thrusters.cpp → GPIO 27 (+Z), 13 (−Z)']) +
  N('RW','act','driver.setOutput(rw_pwm)',['TB9051FTG → reaction wheel  (cmd 18 only)']))
s += CL('log', '4 · Log, then repeat',
  N('SD','log','dataFile.println()',['CSV row to SD every 50 ms','+ USB / XBee telemetry']) +
  N('STOP','stop',None,["thrusters_off(), RW to 0, dataFile.close()"], title="'X' key ends the test"))
s += '  IMU -> HDG -> ENC [style=invis]; WRAP -> BB -> TBL [style=invis]; THR -> RW [style=invis]; SD -> STOP [style=invis];\n'
s += '  ENC -> SD [style=invis]; TBL -> THR [style=invis];\n'
s += '  {rank=same; C16; C18} {rank=same; IMU; WRAP} {rank=same; HDG; BB} {rank=same; ENC; TBL} {rank=same; SD; THR} {rank=same; STOP; RW}\n'
s += E('C16','IMU', extra='lhead=cluster_sense') + E('C18','IMU', extra='lhead=cluster_sense')
s += E('HDG','BB', extra='ltail=cluster_sense lhead=cluster_ctrl') + E('TBL','THR', extra='ltail=cluster_ctrl lhead=cluster_act') + E('THR','SD', extra='ltail=cluster_act lhead=cluster_log')
write('01-overview', s)

# ---------------------------------------------------------------- 02 cmd 16
s = HEAD % 'rankdir=TB newrank=true compound=true nodesep=0.4'
s += N('A','cmd','lab8_mag_heading_hold()',['sd_createDataFile("Lab8_maghold"), CSV header, thrusters_off()'], title='cmd 16')
s += N('C','dec',None,["key = 'C'?"], shape='diamond', extra='margin="0.02,0.02"')
s += N('CAL','sense',None,['turn KestrelSAT a full turn, track min/max of mag X, Y','offset = (max + min) / 2'], title='Calibrate magnetometer  (LED yellow)')
s += N('OFF','sense',None,['MAG_OFFSET_X_UT = −50.52 µT','MAG_OFFSET_Y_UT = −27.41 µT'], title='Use saved offsets')
s += N('BIAS','sense',None,['point at the heading, send a key, hold still 3 s','bias = mean(gyro_Z)','target = lab8_mag_heading(mean mag X, Y)'], title='Gyro bias and target  (LED white)')
s += E('A','C') + E('C','CAL','yes') + E('C','OFF','no') + E('CAL','BIAS') + E('OFF','BIAS')
s += CL('loop', 'Hold loop  (LED rose)  ·  runs every pass, logs every 50 ms',
  N('K','dec',None,['key?'], shape='diamond', extra='margin="0.02,0.02"') +
  N('R','sense','lab8_read_imu()',['low-pass mag X, Y  (τ = 0.2 s)','heading = lab8_mag_heading()','rate = gyro_Z − bias']) +
  N('ER','ctrl','lab8_wrap180(target − heading)',['× MAG_HEADING_SIGN (−1) → gyro_Z sign']) +
  N('BB','ctrl','lab8_hold_fan_state()',['on = 5° band (MAG_HOLD_BAND_DEG)','off = 0.5° from center']) +
  N('TS','act','thrusters_set(fan_state)',['+1 → +Z fan   −1 → −Z fan   0 → off']) +
  N('LG','log',None,['SD + USB every 50 ms, XBee every 4th row'], title='Log row') +
  N('Z','ctrl',None,['target = current heading'], title="'Z' re-zero"))
s += N('X','stop',None,['thrusters_off(), dataFile.close()','LED green, back to the menu'], title="'X' ends the test")
s += E('BIAS','K', extra='lhead=cluster_loop')
s += E('K','R','none') + E('R','ER') + E('ER','BB') + E('BB','TS') + E('TS','LG')
s += '  LG -> K [color="#7F7F7F" style=dashed constraint=false];\n'
s += E('K','Z',"'Z'") + '  Z -> R [constraint=false];\n' + E('K','X',"'X'")
s += '  {rank=same; R; Z}\n'
write('02-cmd16-mag-heading-hold', s)

# ---------------------------------------------------------------- 03 bang-bang
s = HEAD % 'rankdir=TB nodesep=0.45'
s += N('IN','ctrl','lab8_hold_fan_state(error, rate, fan_state, on_deg, off_deg)',[])
s += N('U','ctrl',None,['u = error − rate·|rate| / (2α) − rate·t_lag','α = HOLD_ALPHA_DPS2 = 1.0 °/s²     t_lag = HOLD_FAN_LAG_S = 0.5 s'], title='Predicted stopping error: where will KestrelSAT stop if we brake now?')
s += N('S','dec',None,['fan_state'], shape='diamond', extra='margin="0.02,0.02"')
s += N('P','dec',None,['u < off_deg ?'], shape='diamond', extra='margin="0.02,0.02"')
s += N('M','dec',None,['u > −off_deg ?'], shape='diamond', extra='margin="0.02,0.02"')
s += N('O','dec',None,['off ≥ 500 ms?','(HOLD_MIN_OFF_MS)'], shape='diamond', extra='margin="0.02,0.02"')
s += N('OFF','log',None,['record t_off (spin-down timer)'], title='fan_state = 0')
s += N('KEEP','act',None,['same fan stays on'], title='keep pushing')
s += N('ON','dec',None,['compare u','with on_deg'], shape='diamond', extra='margin="0.02,0.02"')
s += N('FP','act',None,['+Z fan on'], title='fan_state = +1')
s += N('FM','act',None,['−Z fan on'], title='fan_state = −1')
s += N('ST','log',None,['both fans off'], title='fan_state = 0')
s += E('IN','U') + E('U','S') + E('S','P','+1') + E('S','O','0') + E('S','M','−1')
s += E('P','OFF','yes') + E('P','KEEP','no') + E('M','OFF','yes') + E('M','KEEP','no')
s += E('O','ST','no') + E('O','ON','yes')
s += E('ON','FP','u > on_deg') + E('ON','ST','inside band') + E('ON','FM','u < −on_deg')
s += N('NOTE','note',None,['cmd 16:   on_deg = 5°  (±5° band),  off_deg = 0.5°','cmd 18 phase 4:   on_deg = 2°,  off_deg = 0.5°','Fans never switch directly from one to the other.'], title='Thresholds used')
s += '  {rank=same; OFF; KEEP; ON}\n  KEEP -> NOTE [style=invis]; \n'
write('03-bang-bang-hold-fan-state', s)

# ---------------------------------------------------------------- 04 cmd 18 phases
s = HEAD % 'rankdir=TB newrank=true nodesep=0.5'
s += N('A','cmd','lab8_wheel_hold_dump()',['lab8_prompt_int(): starting RW speed 1–100 %','sd_createDataFile("Lab8_whldump"), wait for a key'], title='cmd 18')
s += N('P1','sense',None,['driver.setOutput(rw_start): RW spins up, KestrelSAT held by hand','7–10 s: average gyro_Z, mag X/Y and |RW RPM|'], title='Phase 1  ·  0–10 s  ·  LED white')
s += N('P2','sense',None,['bias, target = lab8_mag_heading(averages), rpm_f','[WARN] if encoder reads < 60 RPM'], title='Phase 2  ·  10–13 s  ·  LED blue: release')
s += N('P3','act',None,['thrusters_set(+1): +Z fan on full the whole phase','RW speed loop holds the heading, so the wheel must slow','(see the phase 3 control diagram)'], title='Phase 3  ·  wheel-held dump  ·  LED teal')
s += N('D','dec',None,['rpm_f < 60 RPM','or 300 s?'], shape='diamond', extra='margin="0.02,0.02"')
s += N('P4','act',None,['rw_pwm = 0 (RW off)','lab8_hold_fan_state(err, rate, 2°, 0.5°) → thrusters_set()','fans hold the heading until X'], title='Phase 4  ·  thrusters hold heading')
s += N('X','stop','lab8_end_test()',['thrusters_off(), driver.setOutput(0)','dataFile.close(), LED green'], title="'X' key")
s += CL('tick','Every 50 ms, all phases',
  N('T1','sense','lab8_read_imu()',['gyro_Z, mag_X, mag_Y']) +
  N('T2','sense','lab8_measure_RW_rpm()',['|RPM|, filtered τ = 0.2 s']) +
  N('T3','log',None,['standard Lab 8 columns +','heading, target, w_RW_target'], title='Log CSV row') +
  N('T4','stop','lab8_abort_requested()',["'X' → end (abort if before phase 4)"]))
s += '  T1 -> T2 -> T3 -> T4 [style=invis];\n'
s += E('A','P1') + E('P1','P2') + E('P2','P3') + E('P3','D') + E('D','P3','no', extra='constraint=false') + E('D','P4','yes') + E('P4','X',"'X'")
s += '  {rank=same; P1; T1} \n'
write('04-cmd18-wheel-hold-dump-phases', s)

# ---------------------------------------------------------------- 05 cmd 18 phase-3 cascade
s = HEAD.replace('digraph G {','digraph G {\n  layout=neato; splines=true; overlap=false;') % ''
s += N('TGT','sense',None,['from phase 2 calibration'], title='target heading')
s += N('HDG','sense','lab8_mag_heading()',['filtered mag X, Y'])
s += N('IMU','sense','lab8_read_imu()',['rate = gyro_Z − bias, filtered'])
s += N('E1','ctrl','lab8_wrap180()',['err = (target − heading)','× MAG_HEADING_SIGN'])
s += N('PD','ctrl',None,['a_des = 0.5·err − 1.2·rate','(WHL_KP, WHL_KD)'], title='Heading PD → wanted body accel')
s += N('FF','ctrl',None,['ẇ = (a_des − α·fan_state) / 0.12','clamp ±30 RPM/s  (WHL_COUPLING)'], title='Cancel fan torque → wheel accel')
s += N('INT','ctrl',None,['w_target += ẇ·dt'], title='RW speed target')
s += N('TR','ctrl',None,['w_trim += 2·(w_target − rpm_f)·dt','clamp ±150 RPM'], title='Speed trim (integral)')
s += N('LUT','ctrl','lab8_rw_pwm_from_rpm()',['(w_target + w_trim) → PWM','via RW_TABLE speed curve'])
s += N('DRV','act','driver.setOutput(rw_pwm)',['TB9051FTG → reaction wheel'])
s += N('FAN','act','thrusters_set(+1)',['+Z fan on full all phase'])
s += N('SAT','entry',None,['fan torque vs. wheel reaction'], title='KestrelSAT body')
s += N('ENC','sense','lab8_measure_RW_rpm()',['rpm_f = |RPM|, filtered'])
s += N('NOTE','note',None,['IMU and magnetometer see the body motion and close the loop every 50 ms.','The fan torque is steady, so the wheel can only hold the heading','by slowing down, which drains its stored momentum.'], title='Why the wheel slows')
s += E('TGT','E1') + E('HDG','E1') + E('E1','PD') + E('PD','FF') + E('FF','INT') + E('INT','TR') + E('TR','LUT') + E('LUT','DRV') + E('ENC','TR', 'rpm_f')
s += E('IMU','PD','rate') + E('DRV','SAT') + E('FAN','SAT')
s += '  FAN -> FF [label=<<FONT POINT-SIZE="16"><B>fan_state</B></FONT>> style=dashed color="#196B24" constraint=false];\n'
s += '  DRV -> ENC [style=dashed color="#0F9ED5" label=<<FONT POINT-SIZE="16"><B>encoder</B></FONT>>];\n'
for k,(x,y) in {'TGT': (0, 4), 'HDG': (1, 4), 'IMU': (2, 4), 'E1': (0, 3), 'PD': (1, 3), 'FF': (2, 3), 'ENC': (0, 2), 'TR': (1, 2), 'INT': (2, 2), 'DRV': (0, 1), 'LUT': (1, 1), 'SAT': (0, 0), 'FAN': (1, 0), 'NOTE': (2.2, 1)}.items():
    s += '  %s [pos="%d,%d!"];\n' % (k, x*440, y*125)
write('05-cmd18-phase3-control-loop', s)

# ---------------------------------------------------------------- 06 thruster driver
s = HEAD % 'rankdir=TB nodesep=0.5 ranksep=0.55 newrank=true'
s += N('SET','cmd','thrusters_set(cmd)',['cmd from −1 to +1','cmd ≥ 0 → thrusters_set_each(cmd, 0)','cmd < 0 → thrusters_set_each(0, −cmd)'])
s += N('EACH','ctrl','thrusters_set_each(plusZ, minusZ)',['clamp 0–1, save plusZ_cmd / minusZ_cmd','(thrusters_get_*() reads them for the logs)'])
s += N('WF','ctrl','write_fan(pin, ch, duty)',['duty < FAN_MIN_DUTY → 0','counts = duty × 1023  (10-bit)'])
s += N('INIT','entry','thrusters_init()',['in setup(): ledcAttach(pin, 100 Hz, 10-bit)','then thrusters_off()'])
s += N('OFFN','stop','thrusters_off()',['= thrusters_set_each(0, 0)','used by every stop and abort'])
s += N('L1','act','ledcWrite()',['GPIO 27, 100 Hz PWM'])
s += N('L2','act','ledcWrite()',['GPIO 13, 100 Hz PWM'])
s += N('F1','act',None,['logic-level N-MOSFET, low side','torques KestrelSAT +Z (CCW from above)'], title='+Z fan  (Pi-FAN 5 V, 0.2 A)')
s += N('F2','act',None,['logic-level N-MOSFET, low side','torques KestrelSAT −Z (CW from above)'], title='−Z fan  (Pi-FAN 5 V, 0.2 A)')
s += '  {rank=same; SET; EACH; WF} {rank=same; INIT; OFFN; L1; L2}\n'
s += E('SET','EACH') + E('EACH','WF') + E('WF','L1','+Z duty') + E('WF','L2','−Z duty') + E('L1','F1') + E('L2','F2')
s += '  INIT -> OFFN [style=dashed]; OFFN -> EACH [style=dashed];\n'
write('06-thruster-driver', s)
