"""Pin the desktop app's YOLO weights independently of the sealed R5 model."""
import hashlib
from pathlib import Path

MODEL_PATH = Path('/media/huudinh/New Volume/esp32/Fire_smoke_person_detection/best_minireplay.pt')
MODEL_SHA256 = 'e792f896faf3fb584842898e93913cdfd0fc33b5d48d620163fadd15f02212c4'


def validate_model(path=None, expected=None):
    path = MODEL_PATH if path is None else Path(path)
    expected = MODEL_SHA256 if expected is None else expected
    if path.is_symlink() or not path.is_file():
        raise ValueError('APP_YOLO_MODEL_MISSING')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != expected:
        raise ValueError('APP_YOLO_MODEL_HASH_MISMATCH')
    return path
