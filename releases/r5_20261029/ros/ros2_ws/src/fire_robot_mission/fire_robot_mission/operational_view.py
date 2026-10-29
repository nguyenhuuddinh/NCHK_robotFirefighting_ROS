"""R5 camera status and keys; renders session reports, grants no permissions."""
import math
import time


def status_lines(report):
    r = report or {}; budget = r.get('budget') or {}; feedback = r.get('operator_feedback') or {}
    band = r.get('ir_display_band',r.get('ir_band'))
    handback=r.get('handback') or {};release=handback.get('release') or {}
    range_label = {'TOO_CLOSE':'QUA GAN', 'SPRAY_RANGE':'TRONG VUNG PHUN',
                   'APPROACH':'CAN TIEN', 'TOO_FAR':'QUA XA / CAN TIEN',
                   'NO_FLAME':'KHONG LUA (IR)', 'NO_FLAME_SIGNATURE':'KHONG LUA (IR)', 'UNKNOWN':'CHUA DU DU LIEU'}.get(band,'CHO IR MOI')
    rows=[
        ('R5 CANDIDATE / LIVE INTENTS ENABLED - INTEGRATION PENDING'
         if (r.get('execution') or {}).get('enabled') else
         'R5 CANDIDATE / INTENT OBSERVER - LIVE OUTPUT DISCONNECTED'),
        str(r.get('phase', 'WAIT_DATA'))+' | '+str(r.get('reason', 'WAIT_REPORT')),
        'IR '+range_label+' ['+str(band)+'] ADC '+str(r.get('ir_counts')),
        'IR tuoi='+str(r.get('ir_source_age_ms'))+'ms | xac nhan='+str(r.get('range_confirm_samples',0))+'/'+str(r.get('ir_confirm_required',5))+' | proof tam/STOP='+str(r.get('aligned_stop_proof')),
        'GUARD '+str(r.get('cancel_guard_state'))+' | '+str(r.get('cancel_guard_blockers') or []),
        'YOLO fresh='+str(r.get('yolo_fresh'))+' fire='+str(r.get('fire_confirmed'))+' centered='+str(r.get('centered')),
        ('IR: TIEN THEO VUNG / KHONG CAP QUANG DUONG-THOI GIAN' if budget.get('unbounded') else
         'I remaining: '+str(round(budget.get('remaining_m', .30), 3))+'m / '+str(round(budget.get('remaining_s', 15.), 1))+'s'),
        'TRA NAV2 '+str(release.get('phase','WAIT'))+' | '+str(release.get('blockers') or handback.get('blockers') or []),
        'KEY '+str(feedback.get('key'))+' accepted='+str(feedback.get('accepted'))+' | '+str(feedback.get('reason')),
        'A arm | F fire watch | T align | I IR | P one pulse | X cancel cycle | Q STOP/exit',
    ]
    auto=r.get('automatic') or {}
    sequence=r.get('spray_sequence') or {}
    if sequence.get('stage') in ('REVIEW','ARM_NEXT'):
        count=sequence.get('confirmation_sample_count',0)
        required=sequence.get('confirmation_required',r.get('ir_confirm_required',3))
        rows[3]='IR luot tiep='+str(count)+'/'+str(required)+' | '+str(sequence.get('reason','CHO DANH GIA SAU OFF'))
    if sequence.get('perception_policy')=='REVIEW_AFTER_OFF_HOME' and sequence.get('pulse_state')=='FIRING':
        remaining=sequence.get('pulse_remaining_s')
        rows.insert(2,'DANG PHUN '+str(sequence.get('pulse_s',5))+'s TU ACK ESP | CON '+str(round(remaining,1) if isinstance(remaining,(int,float)) else '?')+
            's | IR/YOLO KIEM LAI SAU OFF')
    if sequence:
        rows.insert(2,'PHUN LIEN TIEP '+str(sequence.get('consecutive_count',sequence.get('pulse_index',0)))+'/'+str(sequence.get('max_pulses',3))+
            ' | ON chuoi '+str(sequence.get('consecutive_count',sequence.get('pulse_index',0))*sequence.get('pulse_s',5))+'/'+str(sequence.get('max_total_on_s',15))+
            's | '+str(sequence.get('reason','')))
    if sequence.get('post_off_decision') and sequence.get('pulse_state') in ('VERIFY_OFF','REVIEW','DONE'):
        rows.insert(2,'SAU OFF: '+str(sequence['post_off_decision'])+' | CHO HOME TRUOC LAN ON/TIEN TIEP')
    if r.get('phase')=='GOAL_BLOCKED':
        rows[1]='DANG XU LY LUA: HUY GOAL MOI, CHO STOP/OFF; X HUY CHU KY'
    if auto.get('enabled'):
        rows[0]='R5 AUTOMATIC / DAT GOAL - TU XU LY LUA'
        rows[1]=rows[1].replace('READY_PRESS_A','AUTO_START_READY').replace('PRESS_P','AUTO_P').replace('PRESS_I','AUTO_IR').replace('PRESS_T','AUTO_ALIGN')
        rows[-1]='X huy chu ky | Q / ESC STOP va thoat'
        task=r.get('task') or {}
        stage={'STARTUP':'KHOI DONG','NAVIGATION':'NAV2','ALIGN':'CAN TAM','IR':'IR','SPRAY':'PHUN','RETURN_NAV2':'TRA NAV2','STOP':'DUNG'}.get(task.get('stage'),'DANG CHO')
        outcome={'RUNNING':'DANG THUC HIEN','COMPLETED':'HOAN THANH','INTERRUPTED':'BI NGAT'}.get(task.get('outcome'),'DANG CHO')
        rows[1]=stage+' | '+outcome
        rows.insert(2,'LY DO: '+str(task.get('reason') or auto.get('blocker') or r.get('reason')))
        # Detailed guard/phase names remain in raw JSON, not ten simultaneous
        # apparent states on the camera panel.
        rows=[row for row in rows if not row.startswith(('GUARD ','KEY '))]
        if r.get('retry_fire_clear_required'):
            rows.insert(3,(('PHUC HOI: NHAN GOAL MOI; CHO YOLO + IR HET LUA 3s' if r.get('recovery_require_ir_clear')
                 else 'PHUC HOI: NHAN GOAL MOI; CHO YOLO HET LUA 3s')
                if r.get('recovery_clear_policy') else
                'DA TRA NAV2: NHAN GOAL MOI; CHO '+('YOLO + IR HET LUA 0.5s ROI LUA MOI'
                    if r.get('recovery_require_ir_clear') else 'YOLO MAT ROI THAY LAI LUA'))
                if r.get('cycle') is None and r.get('phase') in ('ARMED_IDLE','NAVIGATION')
                else 'NGAT CHU KY: DANG DUNG / OFF / NHA QUYEN')
    if not r.get('armed') and not r.get('fault'):
        proof = r.get('startup_readiness') or {}
        blockers = list(proof.get('blockers') or []) + list(proof.get('adapter_blockers') or [])
        text = ('DA NHAN A: CHO IR MOI | CON '+str(round(proof.get('arm_request_remaining_s') or 0., 1))+'s'
                if proof.get('arm_request_pending') else
                'ARM SAN SANG: NHAN A' if proof.get('ready') is True else
                'ARM CHO '+str(round(proof.get('stable_s', 0.), 1))+'/3s | '+
                (', '.join(blockers) if blockers else 'DANG TICH LUY DU LIEU MOI'))
        rows.insert(2, ('AUTO CHO DU PROOF / TU ARM' if auto.get('enabled') else text))
    if r.get('phase') == 'WAIT_P':
        rows.insert(2, 'P CHO: '+str(r['p_admission_blocker']) if r.get('p_admission_blocker')
                    else 'AUTO: SAN SANG MOT LUOT PHUN' if auto.get('enabled') else 'P SAN SANG: NHAN P MOT LAN | TU CHOI THI NHAN LAI')
    if r.get('phase') == 'IR_CONFIRM':
        proof=r.get('stationary_proof') or {}
        rows.insert(2, 'IR XAC NHAN '+str(proof.get('sample_count',0))+'/'+str(r.get('ir_confirm_required',5))+' | '+
                    str(proof.get('state') or 'CHO ACTOR')+' | '+
                    str(proof.get('reason') or proof.get('first_actor_blocker') or
                        proof.get('first_ir_blocker') or 'CHO MAU MOI'))
    if r.get('blocked_goal_cancel'):
        rows.insert(2,'GOAL DAT SOM: DANG HUY/DA DUNG | CHO NAV2 READY ROI DAT LAI')
    if r.get('cycle') is not None and not r.get('closed'):
        rows.insert(2,'DANG XU LY LUA / NHA QUYEN: CHUA NHAN GOAL MOI | X HUY TRUOC PHUN')
    if r.get('input_hold'):
        rows.insert(2,'TAM DUNG: '+', '.join(r.get('input_hold_blockers') or [])+
            ' | DU LIEU MOI + DUNG '+str(round(r.get('input_recovery_stable_s',0.),1))+'/0.5s THI TU TIEP')
    if r.get('phase')=='WAIT_STOP':
        explanations={'WAIT_LATE_CANCEL_RESPONSE':'CHO NAV2 TRA LOI HUY GOAL - XE GIU DUNG',
            'WAIT_LATE_CANCEL_STATUS':'CHO NAV2 XAC NHAN GOAL DA HUY - KHONG GUI LAI',
            'WAIT_LATE_STATIONARY_PROOF':'CHO XE DUNG HAN - DU PROOF SE TU TIEP',
            'WAIT_CANCEL_SERVICE_RECOVERY':'CHO KET NOI DICH VU NAV2 PHUC HOI - XE GIU DUNG'}
        for blocker in r.get('cancel_guard_blockers') or []:
            if blocker in explanations:
                rows.insert(2,explanations[blocker]);break
    if r.get('fault') == 'BOOT_OR_SERIAL_EPOCH_CHANGED':
        rows.insert(2, 'MAT / DOI PHIEN SERIAL S3: GIU DUNG, CHO PHUC HOI')
    rebind=r.get('runtime_rebind') or {}
    if rebind.get('held'):
        rows[1]='TAM DUNG | CHO DU LIEU / QUYEN DIEU KHIEN PHUC HOI'
        rows.insert(2,'GIU NAV2 / DINH VI / CAMERA | KHONG CAN DAT POSE LAI')
        rows.insert(3,'DANG CHO: '+str(rebind.get('reason')))
        rows.insert(4,'IR RAW tuoi='+str(r.get('ir_raw_source_age_ms'))+'ms | PREVIEW tuoi='+str(r.get('ir_source_age_ms'))+'ms')
        rows=[s for s in rows if not s.startswith('DANG XU LY LUA / NHA QUYEN:')]
    if r.get('navigation_only'):
        rows.insert(2,'NAV2 HOAT DONG | BOM TAM KHOA: CHO HOME THAT / SUA LOI SERVO')
    stop=r.get('shutdown') or {}
    if stop.get('requested'):
        return [rows[0],'DANG DUNG PHIEN | '+str(stop.get('reason')),
            'ZERO / OFF / HOME / RELEASE: '+str(stop.get('blockers')),
            'confirmed='+str(stop.get('confirmed'))+' | '+str(round(stop.get('elapsed_s',0.),1))+'s / 4s',
            'GIU NODE TELEMETRY MO TRONG KHI CHO XAC NHAN DUNG']
    return rows


def key_name(code):
    if code == 27:
        return 'ESC'
    if 0 <= code <= 255:
        key = chr(code).upper()
        return key if key in 'AFTIPXQ' else None
    return None


class SessionView:
    def __init__(self, automatic=False):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.name = 'R5 automatic - goal only / X cancel / Q STOP' if automatic else 'R5 operator - A/F/T/I/P/X/Q'
        self.image = None
        self.image_at = None
        self.visible = False

    def image_input(self, message):
        value = self.cv2.imdecode(self.np.frombuffer(message.data, dtype=self.np.uint8), self.cv2.IMREAD_COLOR)
        if value is not None:
            self.image, self.image_at = value, time.monotonic()

    def tick(self, report):
        if self.visible:
            # OpenCV 4.5.4 GTK does not implement WND_PROP_VISIBLE: it returns
            # -1 even for an open window. AUTOSIZE is 0/1 while the window
            # exists and -1 after deletion. Probe before imshow so a genuine
            # close cannot silently recreate the window and bypass STOP.
            window = self.cv2.getWindowProperty(self.name, self.cv2.WND_PROP_AUTOSIZE)
            if not math.isfinite(window):
                raise RuntimeError('GUI_WINDOW_EXISTENCE_PROBE_INVALID')
            if window < 0:
                return 'WINDOW_CLOSED'
        fresh = self.image_at is not None and 0 <= time.monotonic()-self.image_at <= .5
        frame = self.image.copy() if fresh else self.np.zeros((480, 720, 3), dtype=self.np.uint8)
        rows = status_lines(report)
        if not fresh:
            rows.insert(1, 'STALE / NO IMAGE: waiting; session remains open')
        panel = self.np.zeros((len(rows)*25+10, frame.shape[1], 3), dtype=self.np.uint8)
        for i, text in enumerate(rows):
            self.cv2.putText(panel, text[:150], (8, 22+i*25), self.cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 230, 230), 1)
        self.cv2.imshow(self.name, self.np.vstack((frame, panel)))
        self.visible = True
        return key_name(self.cv2.waitKey(1) & 0xff)

    def close(self):
        self.cv2.destroyAllWindows()
