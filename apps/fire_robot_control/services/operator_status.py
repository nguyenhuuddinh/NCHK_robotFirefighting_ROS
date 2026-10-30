"""Convert validated operator reports to conservative UI-only labels.

The robot's existing operator remains the authority. This module only formats
reports; it never issues commands, fabricates readiness, or infers safety.
"""
from __future__ import annotations

import json
import math


MAX_REPORT_BYTES = 256 * 1024
MAX_AGE_S = 0.5
STAGES = {
    "STARTUP": "Đang khởi động", "NAVIGATION": "Điều hướng",
    "ALIGN": "Đang căn tâm", "IR": "Đang xử lý IR",
    "SPRAY": "Đang xử lý phun", "RETURN_NAV2": "Đang trả quyền Nav2",
    "STOP": "Đang dừng",
}


def _bad_constant(_):
    raise ValueError("non-finite JSON number")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def describe(report_bytes, received_at, now):
    """Return unknown/invalid/stale or fields directly grounded in the report."""
    if report_bytes is None or received_at is None:
        return {"quality": "UNKNOWN", "label": "Chờ dữ liệu robot"}
    if (type(received_at) not in (int, float) or type(now) not in (int, float)
            or not math.isfinite(received_at) or not math.isfinite(now)
            or now < received_at):
        return {"quality": "INVALID", "label": "Thời gian dữ liệu không hợp lệ"}
    if not isinstance(report_bytes, bytes) or len(report_bytes) > MAX_REPORT_BYTES:
        return {"quality": "INVALID", "label": "Báo cáo không hợp lệ"}
    try:
        report = json.loads(report_bytes, parse_constant=_bad_constant,
                            object_pairs_hook=_unique_object)
    except (ValueError, UnicodeDecodeError, RecursionError):
        return {"quality": "INVALID", "label": "Báo cáo không hợp lệ"}
    if (not isinstance(report, dict) or type(report.get("schema_version")) is not int
            or report.get("schema_version") != 1
            or report.get("scope") != "R5_SESSION_INTENT_ONLY"):
        return {"quality": "INVALID", "label": "Nguồn báo cáo không hợp lệ"}
    age = now - received_at
    if age > MAX_AGE_S:
        return {"quality": "STALE", "label": "Chờ báo cáo mới", "receive_age_s": age}
    for key in ("task", "shutdown", "runtime_rebind"):
        if report.get(key) is not None and not isinstance(report[key], dict):
            return {"quality": "INVALID", "label": "Báo cáo không hợp lệ"}
    task = report.get("task") if isinstance(report.get("task"), dict) else {}
    if task.get("stage") is not None and not isinstance(task["stage"], str):
        return {"quality": "INVALID", "label": "Báo cáo không hợp lệ"}
    shutdown = report.get("shutdown") if isinstance(report.get("shutdown"), dict) else {}
    rebind = report.get("runtime_rebind") if isinstance(report.get("runtime_rebind"), dict) else {}
    stage = task.get("stage") if isinstance(task.get("stage"), str) else None
    if shutdown.get("requested") is True:
        if shutdown.get("done") is True:
            label = "Đã xác nhận dừng" if shutdown.get("confirmed") is True else "Chưa xác nhận dừng"
        else:
            label = "Đang dừng phiên"
    elif report.get("fault"):
        fault = report.get('fault')
        label = ('Nav2 gián đoạn — chờ phục hồi định vị' if fault == 'RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST'
                 else 'Đang phục hồi kết nối robot' if fault == 'BOOT_OR_SERIAL_EPOCH_CHANGED'
                 else 'Có lỗi — kiểm tra chi tiết')
    elif report.get("input_hold") is True or rebind.get("held") is True:
        label = "Tạm dừng — chờ phục hồi"
    else:
        label = STAGES.get(stage, "Đang chờ dữ liệu")
    return {
        "quality": "FRESH", "label": label, "receive_age_s": age,
        "reason": task.get("reason") or report.get("reason"),
        "fire_confirmed": report.get("fire_confirmed") if type(report.get("fire_confirmed")) is bool else None,
        "centered": report.get("centered") if type(report.get("centered")) is bool else None,
        "ir_counts": report.get("ir_counts") if isinstance(report.get("ir_counts"), dict) else None,
        "ir_source_age_ms": report.get("ir_source_age_ms"),
        "confirm_samples": report.get("range_confirm_samples"),
        "confirm_required": report.get("ir_confirm_required"),
        "shutdown": shutdown,
    }
