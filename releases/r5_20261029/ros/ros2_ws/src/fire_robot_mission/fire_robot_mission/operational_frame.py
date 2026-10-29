"""Decoded first-frame proof for pre-arm readiness; no camera/device access."""
from .operational_cycle import finite

IMAGE_TOPIC='/yolo/image_annotated/compressed'


class FrameProof:
    def __init__(self):
        self.received=None;self.source_age_ms=None;self.reason='WAIT_FIRST_DECODED_FRAME'
        self.valid_frames=0;self.decode_errors=0;self.last_stamp=None

    def observe(self,msg,now,clock_ns):
        self.received=None
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(clock_ns-stamp)/1e6
        if (not finite(now) or msg.header.frame_id not in ('camera_frame','camera_optical_frame')
                or stamp<=0 or not -.02<=age/1000<=.3
                or self.last_stamp is not None and stamp<=self.last_stamp):
            self.reason='CAMERA_STAMP_FRAME_OR_SEQUENCE_INVALID';return None
        if msg.format.lower() not in ('jpeg','jpg') or not msg.data:
            self.reason='WAIT_NONEMPTY_JPEG_FRAME';return None
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            self.decode_errors+=1;self.reason='FRAME_DECODE_UNAVAILABLE:'+type(exc).__name__;return None
        try:frame=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        except (cv2.error,ValueError,TypeError):
            self.decode_errors+=1;self.reason='FRAME_DECODE_FAILED';return None
        if frame is None or len(frame.shape)!=3 or min(frame.shape[:2])<=0:
            self.decode_errors+=1;self.reason='FRAME_DECODE_FAILED';return None
        self.last_stamp=stamp;self.received=now;self.source_age_ms=max(0.,age)
        self.valid_frames+=1;self.reason='FRESH_DECODED_FRAME'
        return frame

    def fresh(self,now):
        return (self.received is not None and finite(now) and 0<=now-self.received<=.35
                and self.source_age_ms+(now-self.received)*1000<=300)

    def report(self,now):
        return dict(fresh=self.fresh(now),reason=self.reason if self.fresh(now) else
            'WAIT_FIRST_DECODED_FRAME' if self.valid_frames==0 else 'WAIT_FRESH_DECODED_FRAME',
            last_input_reason=self.reason,valid_frames=self.valid_frames,decode_errors=self.decode_errors,
            total_age_ms=None if self.received is None else max(0.,now-self.received)*1000+self.source_age_ms)
