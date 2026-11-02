#!/usr/bin/env python3
"""Generate the importable TripGuard ThingsBoard dashboard."""

from __future__ import annotations

import json
from pathlib import Path


DEVICE_NAME = "Raspberry Pi Demo Device"
ALIAS_ID = "35e75c1a-d9a6-4bbd-92c8-ec18f5bea001"
WIDGET_ID = "35e75c1a-d9a6-4bbd-92c8-ec18f5bea002"
MAP_WIDGET_ID = "35e75c1a-d9a6-4bbd-92c8-ec18f5bea003"
CAMERA_WIDGET_ID = "35e75c1a-d9a6-4bbd-92c8-ec18f5bea004"


SECTIONS = {
    "Tổng quan": [
        ("systemState", "Trạng thái hệ thống"),
        ("event", "Sự kiện mới nhất"),
        ("activationSource", "Nguồn kích hoạt"),
        ("systemArmed", "Hệ thống đã kích hoạt"),
        ("powerPresent", "Nguồn chính"),
        ("stationDetected", "Trạng thái trạm"),
        ("telemetry_seq", "Số thứ tự telemetry"),
        ("uptime_s", "Thời gian chạy", "s"),
    ],
    "Rear Confirm và radar": [
        ("rearConfirmed", "Rear Confirm"),
        ("rear_confirm_count", "Số lần xác nhận"),
        ("rear_confirm_sequence", "Sequence mới nhất"),
        ("rear_confirm_node", "Node Rear Pod"),
        ("radarEnabled", "Cho phép radar"),
        ("radar_online", "Radar online"),
        ("person_detected", "Phát hiện người"),
        ("radar_distance_cm", "Khoảng cách", "cm"),
        ("radar_moving_distance_cm", "Khoảng cách chuyển động", "cm"),
        ("radar_still_distance_cm", "Khoảng cách tĩnh", "cm"),
        ("radar_moving_energy", "Năng lượng chuyển động"),
        ("radar_still_energy", "Năng lượng tĩnh"),
        ("radar_alert_count", "Số cảnh báo radar"),
        ("radar_valid_frames", "Khung radar hợp lệ"),
    ],
    "SOS và cảnh báo": [
        ("sos_pending", "SOS đang chờ xử lý"),
        ("sos_button_latched", "SOS đã chốt"),
        ("sos_button_raw", "Mức nút SOS"),
        ("sos_count", "Số lần SOS"),
        ("driver_ack_count", "Số lần tài xế ACK"),
        ("alert_active", "Cảnh báo đang chạy"),
        ("alert_kind", "Loại cảnh báo hiện tại"),
        ("alert_phase", "Giai đoạn cảnh báo"),
        ("alert_last_kind", "Loại cảnh báo gần nhất"),
        ("alert_last_success", "Cảnh báo gần nhất thành công"),
        ("sms_sent_count", "SMS đã gửi"),
        ("call_started_count", "Cuộc gọi đã bắt đầu"),
    ],
    "Raspberry Pi và camera": [
        ("piOnline", "Pi online"),
        ("piReady", "Pi sẵn sàng"),
        ("cameraEnabled", "Cho phép camera"),
        ("cameraPerson", "Camera phát hiện người"),
        ("cameraConfidencePermille", "Độ tin cậy camera", "‰"),
        ("piScanSessionId", "Phiên quét camera"),
        ("piScanCommandSentCount", "Lệnh quét đã gửi"),
        ("piScanCommandAckCount", "Lệnh quét đã ACK"),
        ("piScanCompleteCount", "Phiên quét hoàn tất"),
        ("piScanLastResult", "Kết quả quét gần nhất"),
        ("cameraAlertArmed", "Cảnh báo camera đã bật"),
        ("cameraAlertCount", "Số cảnh báo camera"),
    ],
    "GPS và vị trí": [
        ("gps_state", "Trạng thái GPS"),
        ("gps_link_alive", "Liên kết GPS"),
        ("gpsFixValid", "GPS fix hợp lệ"),
        ("position_available", "Có vị trí"),
        ("position_current", "Vị trí hiện hành"),
        ("latitude", "Vĩ độ"),
        ("longitude", "Kinh độ"),
        ("satellites", "Số vệ tinh"),
        ("hdop", "HDOP"),
        ("speed_kmh", "Tốc độ", "km/h"),
        ("altitude", "Độ cao", "m"),
        ("location_age_s", "Tuổi dữ liệu vị trí", "s"),
    ],
    "Gateway và đường truyền": [
        ("espGatewayOnline", "ESP Gateway online"),
        ("gateway_spi_ok", "SPI Gateway ↔ STM32"),
        ("connectivityOwner", "Bộ điều khiển kết nối"),
        ("piRxFrames", "Pi RX frames"),
        ("piTxFrames", "Pi TX frames"),
        ("piLastPacketAgeMs", "Tuổi gói tin Pi", "ms"),
        ("mqtt_connected", "MQTT kết nối"),
        ("mqtt_state", "Trạng thái MQTT"),
        ("mqtt_publish_count", "MQTT publish"),
        ("remote_command_count", "Lệnh remote"),
        ("rpc_command_count", "Lệnh RPC"),
        ("gatewayTimestampMs", "Timestamp của Pi", "ms"),
    ],
    "Modem, âm thanh và servo": [
        ("modem_ready", "Modem sẵn sàng"),
        ("network_registered", "Đăng ký mạng"),
        ("pdp_active", "PDP active"),
        ("modem_state", "Trạng thái modem"),
        ("cereg_status", "CEREG"),
        ("audio_busy_raw", "JQ8900 BUSY"),
        ("audio_track", "Track đang phát"),
        ("driving_state", "Trạng thái hành trình"),
        ("servo_angle_deg", "Góc servo", "°"),
        ("servo_moving", "Servo đang chạy"),
        ("servo_tracking_radar", "Servo bám radar"),
        ("servo_move_count", "Số lần servo di chuyển"),
    ],
    "Chẩn đoán và lỗi": [
        ("lastActivationReason", "Lý do kích hoạt gần nhất"),
        ("lastError", "Lỗi hệ thống gần nhất"),
        ("selfTestComplete", "Self-test hoàn tất"),
        ("selfTestMask", "Self-test mask"),
        ("selfTestRetries", "Self-test retries"),
        ("rear_duplicate_count", "Rear duplicate"),
        ("rear_invalid_count", "Rear invalid"),
        ("radar_invalid_frames", "Radar invalid frames"),
        ("radar_uart_errors", "Radar UART errors"),
        ("piCrcErrors", "Pi CRC errors"),
        ("piUartErrors", "Pi UART errors"),
        ("mqtt_error_count", "MQTT errors"),
        ("rpc_receive_error_count", "RPC receive errors"),
        ("remote_rejected_count", "Remote rejected"),
        ("audio_error_count", "Audio errors"),
        ("alert_error_count", "Alert errors"),
        ("telemetry_dropped_count", "Telemetry dropped"),
        ("telemetry_period_ms", "Chu kỳ telemetry", "ms"),
    ],
}


def unique_keys() -> list[str]:
    result: list[str] = []
    for rows in SECTIONS.values():
        for row in rows:
            if row[0] not in result:
                result.append(row[0])
    return result


def build_html() -> str:
    cards: list[str] = []
    boolean_keys = {
        "systemArmed", "powerPresent", "stationDetected", "rearConfirmed",
        "radarEnabled", "radar_online", "person_detected", "sos_pending",
        "sos_button_latched", "sos_button_raw", "alert_active",
        "alert_last_success", "piOnline", "piReady", "cameraEnabled",
        "cameraPerson", "cameraAlertArmed", "gps_link_alive", "gpsFixValid",
        "position_available", "position_current", "espGatewayOnline",
        "gateway_spi_ok", "mqtt_connected", "modem_ready",
        "network_registered", "pdp_active", "servo_moving",
        "servo_tracking_radar", "selfTestComplete",
    }
    for title, rows in SECTIONS.items():
        rendered_rows: list[str] = []
        for row in rows:
            key, label = row[0], row[1]
            unit = row[2] if len(row) > 2 else ""
            value_class = "tg-value tg-flag" if key in boolean_keys else "tg-value"
            rendered_rows.append(
                f'<div class="tg-row"><span class="tg-label">{label}</span>'
                f'<span class="{value_class}" data-v="${{{key}}}">${{{key}}}'
                f'{(" " + unit) if unit else ""}</span></div>'
            )
        cards.append(
            '<section class="tg-card">'
            f'<h2>{title}</h2>'
            + "".join(rendered_rows)
            + "</section>"
        )

    return (
        '<main class="tg-shell">'
        '<header class="tg-header">'
        '<div><div class="tg-eyebrow">TRIPGUARD • LIVE SYSTEM</div>'
        '<h1>Trung tâm giám sát an toàn</h1>'
        '<p>ESP32 Mesh • STM32F411 • Raspberry Pi • ThingsBoard</p></div>'
        '<div class="tg-hero-state" data-v="${systemState}">'
        '<span>HỆ THỐNG</span><strong>${systemState}</strong></div>'
        '</header>'
        '<div class="tg-alert-strip">'
        '<div><small>Rear Confirm</small><b class="tg-flag" data-v="${rearConfirmed}">${rearConfirmed}</b></div>'
        '<div><small>Người trong xe</small><b class="tg-flag" data-v="${person_detected}">${person_detected}</b></div>'
        '<div><small>SOS</small><b class="tg-flag" data-v="${sos_pending}">${sos_pending}</b></div>'
        '<div><small>Gateway</small><b class="tg-flag" data-v="${espGatewayOnline}">${espGatewayOnline}</b></div>'
        '<div><small>Raspberry Pi</small><b class="tg-flag" data-v="${piOnline}">${piOnline}</b></div>'
        '<div><small>Radar</small><b class="tg-flag" data-v="${radar_online}">${radar_online}</b></div>'
        '</div>'
        '<div class="tg-grid">' + "".join(cards) + "</div>"
        '<footer>Thiết bị: Raspberry Pi Demo Device • Dữ liệu tự động cập nhật từ telemetry</footer>'
        '</main>'
    )


CSS = r"""
.tg-shell{min-height:100%;box-sizing:border-box;padding:22px;background:linear-gradient(145deg,#07111f 0%,#0b1b2d 45%,#10263b 100%);color:#e8f2ff;font-family:Inter,Segoe UI,Arial,sans-serif}
.tg-header{display:flex;align-items:center;justify-content:space-between;gap:20px;padding:22px 24px;margin-bottom:14px;border:1px solid #27445f;border-radius:18px;background:linear-gradient(110deg,rgba(14,42,67,.95),rgba(8,25,43,.92));box-shadow:0 15px 40px rgba(0,0,0,.28)}
.tg-eyebrow{font-size:11px;font-weight:800;letter-spacing:.2em;color:#55d6be}.tg-header h1{margin:5px 0 2px;font-size:30px;line-height:1.1}.tg-header p{margin:0;color:#91a9c0;font-size:13px}
.tg-hero-state{min-width:185px;padding:13px 18px;border-radius:14px;text-align:right;background:#132c43;border:1px solid #33526e}.tg-hero-state span{display:block;font-size:10px;letter-spacing:.18em;color:#8aa4bc}.tg-hero-state strong{display:block;margin-top:3px;font-size:21px;color:#5ce1c2}.tg-hero-state[data-v="SYSTEM_ALERT"],.tg-hero-state[data-v="ALERT"]{border-color:#ff6278;background:#401a28}.tg-hero-state[data-v="SYSTEM_ALERT"] strong,.tg-hero-state[data-v="ALERT"] strong{color:#ff7185}
.tg-alert-strip{display:grid;grid-template-columns:repeat(6,minmax(110px,1fr));gap:10px;margin-bottom:14px}.tg-alert-strip>div{padding:12px 14px;border-radius:13px;background:#102338;border:1px solid #263f59}.tg-alert-strip small{display:block;color:#8fa7bd;font-size:11px}.tg-alert-strip b{display:block;margin-top:5px;font-size:15px}
.tg-grid{display:grid;grid-template-columns:repeat(4,minmax(245px,1fr));gap:14px}.tg-card{overflow:hidden;border:1px solid #27445f;border-radius:16px;background:rgba(11,29,47,.92);box-shadow:0 10px 30px rgba(0,0,0,.2)}.tg-card h2{margin:0;padding:13px 16px;border-bottom:1px solid #27445f;background:rgba(24,57,83,.62);font-size:14px;color:#7de3cf;letter-spacing:.02em}.tg-row{display:flex;align-items:center;justify-content:space-between;gap:14px;padding:8px 15px;border-bottom:1px solid rgba(49,75,99,.38);font-size:12px}.tg-row:last-child{border-bottom:0}.tg-label{color:#9fb3c7}.tg-value{max-width:58%;overflow:hidden;text-overflow:ellipsis;text-align:right;color:#f2f7fb;font-weight:700;white-space:nowrap}.tg-flag{color:#ffd166}.tg-flag[data-v="true"],.tg-flag[data-v="1"]{color:#55d6be}.tg-flag[data-v="false"],.tg-flag[data-v="0"]{color:#ff7185}
footer{padding:16px 4px 2px;text-align:right;color:#708ba4;font-size:11px}
@media(max-width:1300px){.tg-grid{grid-template-columns:repeat(3,minmax(235px,1fr))}.tg-alert-strip{grid-template-columns:repeat(3,1fr)}}
@media(max-width:900px){.tg-grid{grid-template-columns:repeat(2,minmax(220px,1fr))}.tg-header{align-items:flex-start}.tg-header h1{font-size:24px}}
@media(max-width:620px){.tg-shell{padding:10px}.tg-grid{grid-template-columns:1fr}.tg-alert-strip{grid-template-columns:repeat(2,1fr)}.tg-header{display:block}.tg-hero-state{margin-top:14px;text-align:left}.tg-value{max-width:52%}}
""".strip()


CAMERA_HTML = r"""
<main class="tg-camera">
  <header>
    <div><div class="eyebrow">TRIPGUARD • CAMERA</div><h2>Camera trong xe</h2></div>
    <span class="pi-state" data-v="${piOnline}">PI ${piOnline}</span>
  </header>
  <div class="frame-wrap">
    <img src="${evidenceImage}" alt="Chưa có ảnh camera. Ảnh sẽ xuất hiện khi phát hiện người.">
  </div>
  <div class="camera-grid">
    <div><small>Phát hiện người</small><b>${cameraPerson}</b></div>
    <div><small>Độ tin cậy realtime</small><b>${cameraConfidencePermille} ‰</b></div>
    <div><small>Evidence</small><b>${eventType}</b></div>
    <div><small>Confidence ảnh</small><b>${confidence}</b></div>
    <div><small>Sequence</small><b>${eventSeq}</b></div>
    <div><small>Dung lượng</small><b>${evidenceBytes} bytes</b></div>
  </div>
  <footer>Ảnh gần nhất do Raspberry Pi gửi khi camera xác nhận người</footer>
</main>
""".strip()


CAMERA_CSS = r"""
.tg-camera{height:100%;box-sizing:border-box;padding:18px;background:linear-gradient(145deg,#07111f,#0d2237);color:#e8f2ff;font-family:Inter,Segoe UI,Arial,sans-serif;overflow:auto}
.tg-camera header{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:14px}.tg-camera h2{margin:5px 0 0;font-size:23px}.eyebrow{color:#55d6be;font-size:11px;font-weight:800;letter-spacing:.18em}.pi-state{padding:9px 13px;border-radius:999px;background:#263443;color:#ff7185;font-weight:800}.pi-state[data-v="true"],.pi-state[data-v="1"]{background:#123d36;color:#65e3c8}
.frame-wrap{display:flex;align-items:center;justify-content:center;min-height:250px;border:1px solid #28445d;border-radius:14px;background:#03080e;overflow:hidden}.frame-wrap img{display:block;width:100%;height:auto;max-height:470px;object-fit:contain}
.camera-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin-top:12px}.camera-grid div{padding:10px 12px;border:1px solid #28445d;border-radius:11px;background:#102338}.camera-grid small{display:block;color:#90a9c0}.camera-grid b{display:block;margin-top:4px;overflow:hidden;text-overflow:ellipsis;color:#f3f8fc;white-space:nowrap}.tg-camera footer{padding-top:12px;color:#718da5;font-size:11px;text-align:right}
@media(max-width:700px){.camera-grid{grid-template-columns:repeat(2,1fr)}}
""".strip()


def data_key(name: str, index: int) -> dict:
    colors = ["#55d6be", "#5aa9ff", "#ffd166", "#ff7185", "#b59cff"]
    return {
        "name": name,
        "type": "timeseries",
        "label": name,
        "color": colors[index % len(colors)],
        "settings": {},
        "_hash": (index + 1) / 1000.0,
        "units": None,
        "decimals": None,
        "funcBody": None,
        "usePostProcessing": None,
        "postFuncBody": None,
    }


def build_camera_widget() -> dict:
    names = [
        "evidenceImage", "eventType", "eventSeq", "cameraConfidencePermille",
        "confidence", "evidenceBytes", "cameraPerson", "piOnline",
    ]
    return {
        "type": "latest",
        "sizeX": 12,
        "sizeY": 8,
        "config": {
            "datasources": [{
                "type": "entity",
                "name": None,
                "entityAliasId": ALIAS_ID,
                "filterId": None,
                "dataKeys": [data_key(name, i) for i, name in enumerate(names)],
            }],
            "showTitle": False,
            "backgroundColor": "#07111f",
            "color": "#e8f2ff",
            "padding": "0px",
            "settings": {"cardHtml": CAMERA_HTML, "cardCss": CAMERA_CSS},
            "title": "Camera trong xe",
            "dropShadow": False,
            "enableFullscreen": True,
            "widgetStyle": {},
            "titleStyle": {"fontSize": "16px", "fontWeight": 500},
            "useDashboardTimewindow": False,
            "showLegend": False,
            "actions": {},
            "showTitleIcon": False,
            "titleIcon": None,
            "enableDataExport": False,
            "displayTimewindow": False,
        },
        "id": CAMERA_WIDGET_ID,
        "typeFullFqn": "system.cards.html_value_card",
    }


def map_key(name: str, label: str, color: str) -> dict:
    return {
        "name": name,
        "label": label,
        "type": "timeseries",
        "settings": {},
        "color": color,
        "aggregationType": None,
        "units": None,
        "decimals": 7 if name in {"latitude", "longitude"} else None,
        "usePostProcessing": None,
        "postFuncBody": None,
    }


def build_map_widget() -> dict:
    marker = {
        "dsType": "entity",
        "dsLabel": "TripGuard Device",
        "dsDeviceId": None,
        "dsEntityAliasId": ALIAS_ID,
        "dsFilterId": None,
        "additionalDataKeys": [
            map_key("speed_kmh", "speed_kmh", "#ffd166"),
            map_key("gps_state", "gps_state", "#ff7185"),
        ],
        "label": {"show": True, "type": "pattern", "pattern": "${entityName}"},
        "tooltip": {
            "show": True,
            "trigger": "click",
            "autoclose": True,
            "type": "pattern",
            "pattern": (
                "<b>${entityName}</b><br/><br/><b>Vĩ độ:</b> ${latitude:7}"
                "<br/><b>Kinh độ:</b> ${longitude:7}"
                "<br/><b>Tốc độ:</b> ${speed_kmh} km/h"
                "<br/><b>GPS:</b> ${gps_state}"
            ),
            "offsetX": 0,
            "offsetY": -1,
        },
        "groups": None,
        "xKey": map_key("latitude", "latitude", "#55d6be"),
        "yKey": map_key("longitude", "longitude", "#5aa9ff"),
        "markerType": "icon",
        "markerShape": {
            "shape": "markerShape1",
            "size": 34,
            "color": {"type": "constant", "color": "#55d6be"},
        },
        "markerIcon": {
            "icon": "directions_car",
            "size": 40,
            "color": {"type": "constant", "color": "#55d6be"},
        },
        "markerImage": {"type": "image", "image": "", "imageSize": 34},
        "markerOffsetX": 0.5,
        "markerOffsetY": 1,
        "markerClustering": {
            "enable": False,
            "zoomOnClick": True,
            "maxZoom": None,
            "maxClusterRadius": 80,
            "zoomAnimation": True,
            "showCoverageOnHover": True,
            "spiderfyOnMaxZoom": False,
            "chunkedLoad": False,
            "lazyLoad": True,
            "useClusterMarkerColorFunction": False,
            "clusterMarkerColorFunction": None,
        },
    }
    return {
        "type": "latest",
        "sizeX": 12,
        "sizeY": 8,
        "config": {
            "datasources": [],
            "showTitle": True,
            "backgroundColor": "rgba(0, 0, 0, 0)",
            "color": "rgba(0, 0, 0, 0.87)",
            "padding": "0px",
            "settings": {
                "mapType": "geoMap",
                "markers": [marker],
                "polygons": [],
                "circles": [],
                "additionalDataSources": [],
            },
            "title": "Vị trí GPS TripGuard",
            "showTitleIcon": False,
            "titleTooltip": "",
            "dropShadow": True,
            "enableFullscreen": True,
            "widgetStyle": {},
            "widgetCss": "",
            "titleStyle": {"fontSize": "16px", "fontWeight": 400},
            "pageSize": 1024,
            "noDataDisplayMessage": "Đang chờ GPS fix hợp lệ...",
            "configMode": "basic",
            "titleFont": None,
            "titleColor": None,
            "margin": "0px",
            "borderRadius": "0px",
            "iconSize": "24px",
            "titleIcon": "map",
            "iconColor": "#55d6be",
            "actions": {},
        },
        "id": MAP_WIDGET_ID,
        "typeFullFqn": "system.map",
    }


def build_dashboard() -> dict:
    keys = unique_keys()
    widget = {
        "type": "latest",
        "sizeX": 24,
        "sizeY": 18,
        "config": {
            "datasources": [{
                "type": "entity",
                "name": None,
                "entityAliasId": ALIAS_ID,
                "filterId": None,
                "dataKeys": [data_key(name, i) for i, name in enumerate(keys)],
            }],
            "timewindow": {
                "displayValue": "",
                "selectedTab": 0,
                "realtime": {
                    "realtimeType": 1,
                    "interval": 1000,
                    "timewindowMs": 60000,
                    "quickInterval": "CURRENT_DAY",
                },
                "history": {
                    "historyType": 0,
                    "interval": 1000,
                    "timewindowMs": 60000,
                    "fixedTimewindow": {"startTimeMs": 0, "endTimeMs": 0},
                    "quickInterval": "CURRENT_DAY",
                },
                "aggregation": {"type": "NONE", "limit": 25000},
            },
            "showTitle": False,
            "backgroundColor": "#07111f",
            "color": "#e8f2ff",
            "padding": "0px",
            "settings": {"cardHtml": build_html(), "cardCss": CSS},
            "title": "TripGuard Full System",
            "dropShadow": False,
            "enableFullscreen": True,
            "widgetStyle": {},
            "titleStyle": {"fontSize": "16px", "fontWeight": 500},
            "useDashboardTimewindow": False,
            "showLegend": False,
            "actions": {},
            "showTitleIcon": False,
            "titleIcon": None,
            "iconColor": "#55d6be",
            "iconSize": "24px",
            "titleTooltip": "",
            "enableDataExport": True,
            "displayTimewindow": False,
        },
        "id": WIDGET_ID,
        "typeFullFqn": "system.cards.html_value_card",
    }
    map_widget = build_map_widget()
    camera_widget = build_camera_widget()

    return {
        "title": "TripGuard - Full System Monitor",
        "image": None,
        "mobileHide": False,
        "mobileOrder": None,
        "configuration": {
            "description": (
                "Giám sát toàn bộ hệ thống TripGuard, vị trí GPS và "
                "ảnh camera evidence theo thời gian thực."
            ),
            "widgets": {
                WIDGET_ID: widget,
                MAP_WIDGET_ID: map_widget,
                CAMERA_WIDGET_ID: camera_widget,
            },
            "states": {
                "default": {
                    "name": "TripGuard Live",
                    "root": True,
                    "layouts": {
                        "main": {
                            "widgets": {
                                WIDGET_ID: {
                                    "sizeX": 24,
                                    "sizeY": 18,
                                    "row": 0,
                                    "col": 0,
                                },
                                MAP_WIDGET_ID: {
                                    "sizeX": 12,
                                    "sizeY": 8,
                                    "row": 18,
                                    "col": 0,
                                },
                                CAMERA_WIDGET_ID: {
                                    "sizeX": 12,
                                    "sizeY": 8,
                                    "row": 18,
                                    "col": 12,
                                },
                            },
                            "gridSettings": {
                                "backgroundColor": "#07111f",
                                "color": "#e8f2ff",
                                "columns": 24,
                                "margin": 0,
                                "backgroundSizeMode": "100%",
                                "autoFillHeight": False,
                                "backgroundImageUrl": None,
                                "mobileAutoFillHeight": False,
                                "mobileRowHeight": 70,
                                "outerMargin": False,
                            },
                        }
                    },
                }
            },
            "entityAliases": {
                ALIAS_ID: {
                    "id": ALIAS_ID,
                    "alias": "TripGuard Device",
                    "filter": {
                        "type": "deviceType",
                        "resolveMultiple": False,
                        "deviceType": "default",
                        "deviceNameFilter": DEVICE_NAME,
                    },
                }
            },
            "filters": {},
            "timewindow": {
                "displayValue": "",
                "hideInterval": False,
                "hideAggregation": False,
                "hideAggInterval": False,
                "hideTimezone": False,
                "selectedTab": 0,
                "realtime": {
                    "realtimeType": 0,
                    "interval": 1000,
                    "timewindowMs": 300000,
                    "quickInterval": "CURRENT_DAY",
                },
                "history": {
                    "historyType": 0,
                    "interval": 1000,
                    "timewindowMs": 300000,
                    "fixedTimewindow": {"startTimeMs": 0, "endTimeMs": 0},
                    "quickInterval": "CURRENT_DAY",
                },
                "aggregation": {"type": "NONE", "limit": 25000},
            },
            "settings": {
                "stateControllerId": "entity",
                "showTitle": True,
                "showDashboardsSelect": False,
                "showEntitiesSelect": False,
                "showDashboardTimewindow": False,
                "showDashboardExport": True,
                "toolbarAlwaysOpen": True,
                "titleColor": "#e8f2ff",
                "showFilters": False,
                "showDashboardLogo": False,
                "dashboardLogoUrl": None,
                "showUpdateDashboardImage": False,
            },
        },
        "name": "TripGuard - Full System Monitor",
        "resources": [],
    }


def main() -> None:
    output = Path(__file__).with_name("TripGuard_Full_System_Dashboard.json")
    output.write_text(
        json.dumps(build_dashboard(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(output.name)


if __name__ == "__main__":
    main()
