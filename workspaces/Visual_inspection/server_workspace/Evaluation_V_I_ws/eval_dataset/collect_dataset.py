#!/usr/bin/env python3
"""
collect_dataset.py  —  Visual Inspection Dataset Collector
Fully flexible: you enter angle, distance, occlusion, n_images yourself.

python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
"""

import os, sys, re, time, csv, shutil, subprocess
from pathlib import Path
from datetime import datetime

# ── Paths ────────────────────────────────────────────────────────────────────
BASE     = Path.home() / 'Documents/Visual_Inspection_ws'
EVAL     = BASE / 'evaluation'
CAPTURES = BASE / 'captures/inspection'
LOG_CSV  = EVAL / 'capture_log.csv'
SVC_TEST = BASE / 'test_scripts/test_inspection_service.py'

# ── Colors ───────────────────────────────────────────────────────────────────
R='\033[1;31m'; G='\033[1;32m'; Y='\033[1;33m'
C='\033[1;36m'; W='\033[1m';    X='\033[0m'

def clr():    os.system('clear')
def hdr(t):   print(f'\n{C}{"═"*58}\n  {t}\n{"═"*58}{X}')
def ok(t):    print(f'{G}  ✓  {t}{X}')
def bad(t):   print(f'{R}  ✗  {t}{X}')
def info(t):  print(f'{Y}  →  {t}{X}')
def div():    print('  ' + '─'*54)
def prompt(q, d=''): 
    hint = f' [{d}]' if d else ''
    v = input(f'  {W}{q}{hint}: {X}').strip()
    return v if v else d

# ── CSV ──────────────────────────────────────────────────────────────────────
COLS = ['timestamp','folder','filename','object_type','distance_m',
        'angle_deg','angle_direction','occlusion_pct','n_objects',
        'ibvs_time_s','ibvs_fps','initial_error_px','final_error_px','converged',
        'coarse_time_s','pipeline_time_s',
        'detection_confidence','objects_inspected',
        'ground_truth_value','notes']

def init_log():
    EVAL.mkdir(parents=True, exist_ok=True)
    if not LOG_CSV.exists():
        with open(LOG_CSV,'w',newline='') as f:
            csv.writer(f).writerow(COLS)

def log_row(**kw):
    kw.setdefault('timestamp', datetime.now().strftime('%Y-%m-%d_%H:%M:%S'))
    with open(LOG_CSV,'a',newline='') as f:
        csv.DictWriter(f, fieldnames=COLS).writerow(kw)

def count_rows():
    if not LOG_CSV.exists(): return 0
    return max(0, sum(1 for _ in open(LOG_CSV)) - 1)

def count_in(subfolder):
    d = EVAL / subfolder
    if not d.exists(): return 0
    return len(list(d.glob('*.jpg')))

# ── Service call ────────────────────────────────────────────────────────────────────

def run_and_parse(target_object: str = '', location_label: str = 'eval'):
    """
    Call /visual_inspection/inspect via test_inspection_service.py.
    After the run, reads metrics from metadata.json saved by the service.
    Returns (img_path, ibvs_time, ibvs_err, converged, objects_inspected, status, conf).
    """
    before_meta = set()
    before_imgs = set()
    if CAPTURES.exists():
        before_meta = {f for f in CAPTURES.rglob('metadata.json')}
        before_imgs = {f for f in CAPTURES.rglob('img_*.jpg')}

    objects_inspected = 0
    status = 'unknown'
    info(f'Calling service: object="{target_object or "any"}"  loc="{location_label}"')

    src = (f'source /opt/ros/humble/setup.bash && '
           f'source {BASE}/inspection_ws/install/setup.bash')
    obj_flag = f'--object "{target_object}"' if target_object else ''
    loc_flag = f'--location "{location_label}"'
    cmd = f'bash -c "{src} && python3 {SVC_TEST} {obj_flag} {loc_flag}"'

    proc = None
    try:
        proc = subprocess.Popen(cmd, shell=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True)
        for line in proc.stdout:
            l = line.strip()
            if l: print(f'     {l}')
            m2 = re.search(r'objects_inspected\s*:\s*(\d+)', l)
            if m2: objects_inspected = int(m2.group(1))
            m3 = re.search(r'status\s*:\s*(\S+)', l)
            if m3: status = m3.group(1)
        proc.wait(timeout=180)
    except Exception as e:
        bad(f'Service call error: {e}')
    finally:
        if proc:
            try: proc.kill(); proc.wait(timeout=3)
            except Exception: pass

    time.sleep(2)

    # ── Read metrics from metadata.json (written by inspection_service) ────────
    import json as _json
    ibvs_time, ibvs_err, converged, conf  = 0.0, 0.0, False, 0.0
    ibvs_fps, coarse_time, pipeline_time  = 0.0, 0.0, 0.0
    initial_err = 0.0
    new_imgs = []

    if CAPTURES.exists():
        after_meta = {f for f in CAPTURES.rglob('metadata.json')}
        after_imgs = {f for f in CAPTURES.rglob('img_*.jpg')}
        new_meta = after_meta - before_meta
        new_imgs = sorted(after_imgs - before_imgs, key=lambda f: f.stat().st_mtime)

        if new_meta:
            meta_f = max(new_meta, key=lambda f: f.stat().st_mtime)
            try:
                md = _json.loads(meta_f.read_text())
                conf          = float(md.get('confidence',           0.0))
                converged     = bool( md.get('ibvs_converged',       False))
                ibvs_err      = float(md.get('ibvs_error_px',        0.0))
                ibvs_time     = float(md.get('ibvs_time_s',          0.0))
                ibvs_fps      = float(md.get('ibvs_fps',             0.0))
                coarse_time   = float(md.get('coarse_time_s',        0.0))
                pipeline_time = float(md.get('pipeline_time_s',      0.0))
                initial_err   = float(md.get('initial_ibvs_error_px',0.0))
            except Exception as e:
                bad(f'metadata.json parse error: {e}')

    # ── Image path: Logitech first, Insta360 live grab as fallback ────────────
    # Fallback fires for ANY case where no Logitech image was saved:
    #   - ibvs_timeout  : object detected, IBVS could not converge
    #   - no_detection  : YOLO found nothing in Insta360 frame
    #   - any other failure
    is_insta360 = False
    if new_imgs:
        img_path = new_imgs[0]
        ok(f'[Logitech] conf={conf:.3f} | coarse_err={initial_err:.1f}px | '
           f'ibvs={ibvs_time:.2f}s | final_err={ibvs_err:.1f}px | '
           f'fps={ibvs_fps} | pipeline={pipeline_time:.1f}s | converged={converged}')
    else:
        # No Logitech image — always try to grab the live Insta360 frame
        reason   = status  # e.g. 'ibvs_timeout', 'no_detection', 'unknown'
        tmp_path = f'/tmp/insta360_fallback_{int(time.time())}.jpg'
        grabber  = str(BASE / 'evaluation/grab_insta360.py')
        src      = (f'source /opt/ros/humble/setup.bash && '
                    f'source {BASE}/inspection_ws/install/setup.bash')
        grab_cmd = f'bash -c "{src} && python3 {grabber} {tmp_path}"'
        info(f'[Insta360] No Logitech image (status={reason}) — grabbing live frame...')
        ret = subprocess.run(grab_cmd, shell=True, timeout=12,
                             capture_output=True, text=True)
        if ret.returncode == 0 and Path(tmp_path).exists():
            img_path    = Path(tmp_path)
            is_insta360 = True
            ok(f'[Insta360] Frame grabbed (status={reason}) → {tmp_path}')
        else:
            img_path = None
            bad(f'[Insta360] Frame grab failed (status={reason}): {ret.stderr.strip()[:100]}')

    return img_path, ibvs_time, ibvs_err, converged, objects_inspected, status, conf, ibvs_fps, coarse_time, pipeline_time, initial_err, is_insta360





def save_img(img_path, dest: Path, fname: str) -> bool:
    dest.mkdir(parents=True, exist_ok=True)
    if img_path and img_path.exists():
        shutil.copy2(img_path, dest / fname)
        ok(f'Saved  →  {(dest/fname).relative_to(EVAL)}')
        return True
    bad('No new image detected — check pipeline')
    return False

def next_n(folder: Path, prefix='img_') -> int:
    existing = list(folder.glob(f'{prefix}*.jpg'))
    nums = []
    for f in existing:
        m = re.search(r'(\d+)', f.stem.replace(prefix,''))
        if m: nums.append(int(m.group(1)))
    return max(nums)+1 if nums else 1

# ── Single capture ────────────────────────────────────────────────────────────
def one_capture(dest: Path, fname_prefix: str, subfolder_name: str,
                obj_type: str, distance: str, angle_deg: str, angle_dir: str,
                occlusion: str, n_objects: str='1',
                ground_truth: str='N/A', ask_caption: bool=False) -> bool:
    n     = next_n(dest, prefix=fname_prefix)
    fname = f'{fname_prefix}{n:02d}.jpg'

    input(f'\n  {W}▶  Ready? Press ENTER to capture...{X}')
    print()
    img, ibvs_t, err_px, conv, n_insp, svc_status, det_conf, fps, coarse_t, pipe_t, init_err, is_insta360 = run_and_parse(
        target_object=obj_type, location_label=subfolder_name)

    # If IBVS failed, fall back to Insta360 overview — add _insta360 suffix
    eff_fname = fname.replace('.jpg', '_insta360.jpg') if is_insta360 else fname
    saved = save_img(img, dest, eff_fname)

    if conv:
        ok(f'IBVS: {ibvs_t:.2f}s | coarse={init_err:.1f}px→final={err_px:.1f}px | fps={fps} | pipeline={pipe_t:.1f}s | conf={det_conf:.3f}')
    elif is_insta360:
        ok(f'[Insta360] Saved as {eff_fname} (reason: {svc_status})')
    elif n_insp > 0:
        ok(f'Inspected {n_insp} | status={svc_status} | conf={det_conf:.3f}')
    else:
        bad(f'IBVS: DID NOT CONVERGE (status={svc_status})')

    caption = ''
    if ask_caption:
        caption = input(f'  {Y}Caption — describe exactly what camera sees: {X}').strip()

    # Build notes: append camera source + failure reason when Insta360 was used
    auto_note = f'[insta360:{svc_status}]' if is_insta360 else ''
    full_note  = ' '.join(filter(None, [auto_note, caption]))

    log_row(folder=subfolder_name, filename=eff_fname, object_type=obj_type,
            distance_m=distance, angle_deg=angle_deg, angle_direction=angle_dir,
            occlusion_pct=occlusion, n_objects=n_objects,
            ibvs_time_s=round(ibvs_t,3), ibvs_fps=fps,
            initial_error_px=round(init_err,2), final_error_px=round(err_px,2),
            converged=conv, coarse_time_s=round(coarse_t,3), pipeline_time_s=round(pipe_t,2),
            detection_confidence=round(det_conf,4), objects_inspected=n_insp,
            ground_truth_value=ground_truth, notes=full_note)
    ok(f'Logged  ({count_rows()} total so far)')
    return saved

# ── Sessions ──────────────────────────────────────────────────────────────────

def ask_obj():
    print(f'''
  Objects (YOLO classes + overview-only):
    1  fire_extinguisher   (conf ≥ 0.5, full IBVS)
    2  gauge               (conf ≥ 0.3, IBVS + sweep)
    3  door                (conf ≥ 0.5, full IBVS)
    4  person              (conf ≥ 0.5, full IBVS)
    5  unknown             (overview only — no IBVS)
    6  main_cylinder       (overview only — no IBVS)''')
    m = {'1':'fire_extinguisher','2':'gauge','3':'door',
         '4':'person','5':'unknown','6':'main_cylinder'}
    return m.get(prompt('Select object','1'), 'fire_extinguisher')

def session_reference():
    hdr('SESSION 1 — REFERENCE IMAGES')
    print(f'''
  Gold-standard baseline for all quality metrics.
  SETUP:  robot at 1m, directly facing object, good lighting.
  Collect 5+ images per object type.
''')
    obj = ask_obj()

    while True:
        dist  = prompt('Exact distance you measured (m)', '1.0')
        n     = int(prompt('How many images at this position','5'))
        prefix = obj.split('_')[0]+'_ref_'
        dest   = EVAL / 'reference'
        info(f'{n} × {obj} at {dist}m  →  evaluation/reference/')

        for i in range(n):
            div()
            print(f'  {C}Capture {i+1}/{n}{X}')
            one_capture(dest, prefix, 'reference', obj, dist, '0','center','0')

        print(f'\n  {Y}Images in reference/: {count_in("reference")}{X}')
        cont = prompt('Continue reference? (y=same obj / n=new obj / q=menu','y').lower()
        if cont == 'q': break
        if cont == 'n': obj = ask_obj()

def session_angle():
    hdr('SESSION 2 — ANGLE EVALUATION')
    print(f'''
  Covers full 180° horizontal + vertical angles.
  Object: fire_extinguisher (fixed) — only robot moves.

  HOW TO ENTER ANGLES:
    Horizontal: enter degrees (e.g. 15, 30, 45, 60, 90, 135)
                direction: L (left) or R (right)
    Vertical:   enter degrees (e.g. 15, 30)
                direction: up or down
    Head-on:    enter 0

  DISTANCE RANGE:  Enter the actual measured distance each time.
    e.g. 2.0m then 2.2m then 1.8m to cover ±20cm variation.
''')
    obj = ask_obj()

    while True:
        angle_in  = prompt('Angle in degrees (e.g. 0, 15, 30, 45, 90)','0')
        angle_dir = prompt('Direction (L / R / up / down / center)','center').lower()
        dist      = prompt('Exact measured distance (m)','2.0')
        n         = int(prompt('How many images at this position','10'))

        # Build folder name
        if angle_in == '0' or angle_dir == 'center':
            sub_angle = '0deg'
            folder    = 'angle_eval/horizontal/0deg'
        elif angle_dir in ('up','down'):
            sub_angle = f'{angle_in}deg_{angle_dir}'
            folder    = f'angle_eval/vertical/{sub_angle}'
        else:
            sub_angle = f'{angle_in}deg_{angle_dir.upper()}'
            folder    = f'angle_eval/horizontal/{sub_angle}'

        dest = EVAL / folder
        info(f'{n} images | angle={sub_angle} | dist={dist}m | {folder}/')
        print(f'  {Y}Position robot at {dist}m, {sub_angle} offset from object.{X}')

        for i in range(n):
            div()
            print(f'  {C}Capture {i+1}/{n} | {sub_angle} | {dist}m{X}')
            one_capture(dest, 'img_', folder, obj, dist,
                        angle_in, angle_dir, '0')

        print(f'\n  {Y}Images in {folder}/: {count_in(folder)}{X}')
        cont = prompt('Next? (y=new angle / q=menu','y').lower()
        if cont == 'q': break

def session_distance():
    hdr('SESSION 3 — DISTANCE EVALUATION')
    print(f'''
  Find image quality vs distance.
  Object: gauge (best for testing OCR at far distance)
  Angle: 0° head-on (fixed)

  Measure the actual distance and type it.
  Suggested: 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0 m
''')
    obj = ask_obj()

    while True:
        dist = prompt('Exact measured distance (m)','2.0')
        n    = int(prompt('How many images','10'))

        # Snap to nearest folder bracket
        d = float(dist)
        if d <= 1.25:   folder = 'distance_eval/1m'
        elif d <= 1.75: folder = 'distance_eval/1.5m'
        elif d <= 2.25: folder = 'distance_eval/2m'
        elif d <= 2.75: folder = 'distance_eval/2.5m'
        elif d <= 3.25: folder = 'distance_eval/3m'
        elif d <= 3.75: folder = 'distance_eval/3.5m'
        else:           folder = 'distance_eval/4m'

        # Create folder with actual measured value in name
        folder = f'distance_eval/{dist}m'
        dest   = EVAL / folder
        info(f'{n} images at {dist}m  →  evaluation/{folder}/')

        for i in range(n):
            div()
            print(f'  {C}Capture {i+1}/{n} | distance={dist}m | 0° head-on{X}')
            one_capture(dest, 'img_', folder, obj, dist, '0','center','0')

        print(f'\n  {Y}Images in {folder}/: {count_in(folder)}{X}')
        cont = prompt('Next distance? (y / q=menu','y').lower()
        if cont == 'q': break

def session_gauge():
    hdr('SESSION 4 — GAUGE ACCURACY GROUND TRUTH')
    print(f'''
  Collect images at KNOWN gauge readings for MAE/RMSE evaluation.
  You need physical equipment (pressure gauge / water gauge).
  SKIP with q if no gauge available.

  Setup: manually set pointer to a known value, tape it in place.
  Distance: 2m | Angle: 0°
''')
    while True:
        true_val = prompt('True gauge reading (e.g. 2.5) or q to exit','')
        if true_val.lower() == 'q': break
        dist = prompt('Exact measured distance (m)','2.0')
        n    = int(prompt('How many images at this reading','3'))
        folder = 'gauge_accuracy'
        dest   = EVAL / folder
        info(f'{n} images | gauge={true_val} | {dist}m')

        for i in range(n):
            div()
            print(f'  {C}Capture {i+1}/{n} | gauge reading = {true_val}{X}')
            # Fix: use _n(\d+).jpg pattern to avoid reading decimal in distance as counter
            _ex_ns = [int(m.group(1)) for f in dest.glob(f'gauge_{true_val}_*.jpg')
                      if (m := re.search(r'_n(\d+)\.jpg$', f.name))]
            _next_n = max(_ex_ns) + 1 if _ex_ns else 1
            fname = f'gauge_{true_val}_{dist}m_n{_next_n}.jpg'
            # use direct save
            input(f'\n  {W}▶  Ready? Press ENTER to capture...{X}')
            print()
            img, ibvs_t, err_px, conv, _, _, det_conf, fps, coarse_t, pipe_t, init_err, is_insta360 = run_and_parse(
                target_object='gauge', location_label=folder)
            # Fall back to Insta360 overview if IBVS failed
            eff_fname = fname.replace('.jpg', '_insta360.jpg') if is_insta360 else fname
            if save_img(img, dest, eff_fname):
                if conv:          ok(f'IBVS {ibvs_t:.2f}s | coarse={init_err:.1f}px→final={err_px:.1f}px | fps={fps} | conf={det_conf:.3f}')
                elif is_insta360: ok(f'[Insta360] Saved as {eff_fname} (reason: ibvs_timeout / no_detection)')
                else:             bad('IBVS timeout — no image saved')
            gauge_note = f'gauge={true_val}' + (f' [insta360:{status}]' if is_insta360 else '')
            log_row(folder=folder, filename=eff_fname, object_type='gauge',
                    distance_m=dist, angle_deg='0', angle_direction='center',
                    occlusion_pct='0', n_objects='1',
                    ibvs_time_s=round(ibvs_t,3), ibvs_fps=fps,
                    initial_error_px=round(init_err,2), final_error_px=round(err_px,2),
                    converged=conv, coarse_time_s=round(coarse_t,3), pipeline_time_s=round(pipe_t,2),
                    detection_confidence=round(det_conf,4), objects_inspected=1,
                    ground_truth_value=true_val, notes=gauge_note)
            ok(f'Logged ({count_rows()} total)')


        cont = prompt('Another reading? (y / q=menu','y').lower()
        if cont == 'q': break

def session_vlm():
    hdr('SESSION 5 — VLM PASS/FAIL IMAGES')
    scenarios = {
        '1': ('fire_ext_pass',  'fire_extinguisher', 'PASS',
              'Extinguisher on wall, NOTHING blocking it'),
        '2': ('fire_ext_fail',  'fire_extinguisher', 'FAIL',
              'Place large box/chair DIRECTLY in front blocking access'),
        '3': ('exit_pass',      'emergency_exit',    'PASS',
              'Door/exit with completely CLEAR walkway'),
        '4': ('exit_fail',      'emergency_exit',    'FAIL',
              'Stack 2-3 chairs or boxes BLOCKING the door'),
        '5': ('door_pass',      'door',              'PASS',
              'Regular door — open or closed, clear state'),
        '6': ('door_fail',      'door',              'FAIL',
              'Door blocked by objects'),
        '7': ('cylinder_pass',  'main_cylinder',     'PASS',
              'Machinery/cylinder — floor dry, no leak'),
        '8': ('cylinder_fail',  'main_cylinder',     'FAIL',
              'Water on floor near cylinder (simulates oil leak)'),
    }
    print()
    for k,(f,o,d,s) in scenarios.items():
        mark = G+'PASS'+X if d=='PASS' else R+'FAIL'+X
        print(f'  {k}: {f:20s}  {mark}  — {s}')
    print()

    choice = prompt('Select scenario','1')
    if choice not in scenarios:
        bad('Invalid'); return
    folder_name, obj_type, expected, setup = scenarios[choice]
    subfolder = f'vlm_eval/{folder_name}'
    dist = prompt('Distance (m)','2.0')
    n    = int(prompt('How many images','10'))

    print(f'\n  {Y}SETUP: {setup}{X}')
    input('  Set up scene, position robot. Press ENTER when ready...')

    dest = EVAL / subfolder
    info(f'{n} images  →  evaluation/{subfolder}/')
    print(f'  {Y}Write 1-sentence caption after each capture.{X}\n')

    for i in range(n):
        div()
        print(f'  {C}Capture {i+1}/{n} | {folder_name}{X}')
        one_capture(dest, f'{folder_name}_', subfolder, obj_type,
                    dist, '0','center','0','1', expected, ask_caption=True)

    print(f'\n  {Y}Images in {subfolder}/: {count_in(subfolder)}{X}')
    cont = prompt('Collect more for this scenario? (y / q=menu','q').lower()
    if cont == 'y':
        n2 = int(prompt('How many more','5'))
        for i in range(n2):
            div()
            print(f'  {C}Extra capture {i+1}/{n2}{X}')
            one_capture(dest, f'{folder_name}_', subfolder, obj_type,
                        dist,'0','center','0','1', expected, ask_caption=True)

def session_occlusion():
    hdr('SESSION 6 — OCCLUSION EVALUATION')
    print(f'''
  How much occlusion can the system tolerate?
  Object: fire_extinguisher | Distance: 2m | Angle: 0°

  Cover the object with tape/cardboard.
  Enter the actual percentage you covered (0, 10, 25, 33, 50, 66, 75, 90...)
''')
    obj = ask_obj()

    while True:
        pct    = prompt('Occlusion percentage you applied (e.g. 0, 25, 50, 75)','0')
        dist   = prompt('Exact measured distance (m)','2.0')
        n      = int(prompt('How many images','10'))
        folder = f'occlusion/{pct}pct'
        dest   = EVAL / folder

        print(f'\n  {Y}Apply {pct}% occlusion to object now.{X}')
        input('  Press ENTER when ready...')
        info(f'{n} images | occlusion={pct}% | dist={dist}m  →  evaluation/{folder}/')

        for i in range(n):
            div()
            print(f'  {C}Capture {i+1}/{n} | occlusion={pct}%{X}')
            one_capture(dest, 'img_', folder, obj, dist,'0','center', pct)

        print(f'\n  {Y}Images in {folder}/: {count_in(folder)}{X}')
        cont = prompt('Next occlusion level? (y / q=menu','y').lower()
        if cont == 'q': break

def session_multi():
    hdr('SESSION 7 — MULTI-OBJECT SCENES')
    print(f'''
  Test ByteTrack with multiple objects in view.
  Scenes to try:
    1: 2 same-class side by side (2 fire extinguishers)
    2: Mixed class (1 gauge + 1 fire extinguisher)
    3: 1 front + 1 back (tests front/back zone detection)
    4: 3 objects across a panel
''')
    scenes = {
        '1':('2 fire_extinguishers side by side','2_objects','fire_extinguisher','2'),
        '2':('1 gauge + 1 fire_ext','2_objects','mixed','2'),
        '3':('1 front + 1 back zone','2_objects','fire_extinguisher','2'),
        '4':('3 objects across panel','3_objects','gauge','3'),
    }
    for k,(d,f,o,n) in scenes.items(): print(f'  {k}: {d}')
    print()
    choice = prompt('Select scene','1')
    if choice not in scenes: bad('Invalid'); return

    desc, folder_key, obj, n_obj = scenes[choice]
    folder    = f'multi_object/{folder_key}'
    dist      = prompt('Distance (m)','2.0')
    n         = int(prompt('How many images','5'))
    dest      = EVAL / folder

    print(f'\n  {Y}Set up: {desc}{X}')
    input('  Press ENTER when ready...')

    for i in range(n):
        div()
        print(f'  {C}Capture {i+1}/{n} | {desc}{X}')
        one_capture(dest, 'img_', folder, obj, dist,'0','center','0', n_obj)

    print(f'\n  {Y}Images in {folder}/: {count_in(folder)}{X}')

def show_summary():
    hdr('CURRENT COLLECTION STATUS')
    sections = [
        ('reference',                   'Reference images'),
        ('angle_eval/horizontal',       'Angle H — all'),
        ('angle_eval/vertical',         'Angle V — all'),
        ('distance_eval',               'Distance — all'),
        ('gauge_accuracy',              'Gauge ground truth'),
        ('vlm_eval/fire_ext_pass',      'VLM fire_ext PASS'),
        ('vlm_eval/fire_ext_fail',      'VLM fire_ext FAIL'),
        ('vlm_eval/exit_pass',          'VLM exit PASS'),
        ('vlm_eval/exit_fail',          'VLM exit FAIL'),
        ('vlm_eval/door_pass',          'VLM door PASS'),
        ('occlusion',                   'Occlusion — all'),
        ('multi_object',                'Multi-object'),
    ]
    for path, label in sections:
        d = EVAL / path
        if d.exists():
            imgs = len(list(d.rglob('*.jpg')))
            bar = G if imgs >= 10 else (Y if imgs > 0 else R)
            print(f'  {bar}{"■"*min(imgs,30):30s}{X}  {label:<28} {imgs} images')
        else:
            print(f'  {R}{"□"*30}{X}  {label:<28} 0 images')
    print(f'\n  Total logged: {count_rows()} rows in CSV')

# ── Main menu ─────────────────────────────────────────────────────────────────

def main():
    init_log()
    sessions = {
        '1': ('Reference images',            session_reference),
        '2': ('Angle evaluation (any angle)', session_angle),
        '3': ('Distance evaluation',          session_distance),
        '4': ('Gauge ground truth',           session_gauge),
        '5': ('VLM PASS/FAIL images',         session_vlm),
        '6': ('Occlusion evaluation',         session_occlusion),
        '7': ('Multi-object scenes',          session_multi),
        's': ('Show collection status',       show_summary),
    }

    while True:
        clr()
        hdr('VISUAL INSPECTION — DATASET COLLECTION')
        print(f'''
  {W}MAKE SURE T1, T2, T3 ARE RUNNING FIRST:{X}
    T1: ros2 run visual_inspection_ros camera_node
    T2: ros2 run visual_inspection_ros servo_node
    T3: ros2 run visual_inspection_ros ibvs_action_server

  {Y}Total images logged: {count_rows()}{X}
''')
        for k,(label,_) in sessions.items():
            print(f'    {C}{k}{X}: {label}')
        print(f'    {C}q{X}: Quit')
        print()

        choice = input(f'  {W}▶  Select session: {X}').strip().lower()
        if choice == 'q':
            break
        elif choice in sessions:
            sessions[choice][1]()
            input(f'\n{G}  Done! Press ENTER to return to menu...{X}')
        else:
            bad('Invalid choice'); time.sleep(1)

    clr()
    hdr('SESSION ENDED')
    show_summary()
    print(f'\n  {W}SCP to laptop:{X}')
    print(f'  scp -r rgen@192.168.8.181:{EVAL} \\')
    print(f'      /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/')

if __name__ == '__main__':
    main()
