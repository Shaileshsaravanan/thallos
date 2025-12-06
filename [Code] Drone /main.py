#!/usr/bin/env python3

import os
import csv
import json
import math
import time
import uuid
import queue
import atexit
import signal
import socket
import random
import string
import datetime
from pathlib import Path
from threading import Thread, Event, Lock

import serial
import requests

import Adafruit_DHT
from picamera import PiCamera

import board
import busio
import adafruit_mlx90614
import adafruit_sgp30

from dataclasses import dataclass, asdict, field

DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 4

I2C_BUS = busio.I2C(board.SCL, board.SDA)

MODEM_AT_PORT = "/dev/ttyUSB2"
GPS_NMEA_PORT = "/dev/ttyUSB1"
MODEM_BAUDRATE = 115200
GPS_BAUDRATE = 9600

LOG_DIR = Path("/home/pi/drone_logs")
CSV_PATH = LOG_DIR / "sensor_log.csv"
IMAGE_DIR = LOG_DIR / "images"
CACHE_PATH = LOG_DIR / "upload_cache.jsonl"

SERVER_URL = "https://your-server.example.com/api/telemetry"

SAMPLE_INTERVAL_SEC = 5
UPLOAD_WORKER_INTERVAL_SEC = 3
GPS_READ_INTERVAL_SEC = 0.5

NETWORK_TEST_HOST = "8.8.8.8"
NETWORK_TEST_PORT = 53
NETWORK_TEST_TIMEOUT = 1.0


@dataclass
class TelemetryRecord:
    record_id: str
    timestamp_iso: str
    gps_lat_deg: float | None
    gps_lon_deg: float | None
    gps_alt_m: float | None
    gps_fix_quality: int | None
    gps_num_satellites: int | None
    gps_hdop: float | None
    flight_distance_m: float | None
    flight_bearing_deg: float | None
    flight_ground_speed_mps: float | None
    dht_temp_c: float | None
    dht_humidity_pct: float | None
    vpd_kpa: float | None
    mlx_canopy_temp_c: float | None
    sgp30_eco2_ppm: int | None
    sgp30_tvoc_ppb: int | None
    image_path: str | None
    upload_status: str | None = None
    upload_http_code: int | None = None
    upload_attempts: int = 0
    local_sequence: int = 0
    node_id: str = ""
    battery_level_pct: float | None = None
    rssi_dbm: float | None = None
    extra_meta: dict = field(default_factory=dict)


@dataclass
class GPSFix:
    timestamp: float
    lat: float | None
    lon: float | None
    alt: float | None
    fix_quality: int | None
    num_sats: int | None
    hdop: float | None


@dataclass
class FlightState:
    last_fix: GPSFix | None = None
    total_distance_m: float = 0.0
    last_distance_increment_m: float = 0.0
    last_bearing_deg: float | None = None
    last_ground_speed_mps: float | None = None


class CSVLogger:
    def __init__(self, path: Path):
        self.path = path
        self.lock = Lock()
        self._ensure_directory()
        self._init_csv()

    def _ensure_directory(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def _init_csv(self):
        if not self.path.exists():
            with self.path.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "record_id",
                    "timestamp_iso",
                    "gps_lat_deg",
                    "gps_lon_deg",
                    "gps_alt_m",
                    "gps_fix_quality",
                    "gps_num_satellites",
                    "gps_hdop",
                    "flight_distance_m",
                    "flight_bearing_deg",
                    "flight_ground_speed_mps",
                    "dht_temp_c",
                    "dht_humidity_pct",
                    "vpd_kpa",
                    "mlx_canopy_temp_c",
                    "sgp30_eco2_ppm",
                    "sgp30_tvoc_ppb",
                    "image_path",
                    "upload_status",
                    "upload_http_code",
                    "upload_attempts",
                    "local_sequence",
                    "node_id",
                    "battery_level_pct",
                    "rssi_dbm",
                    "extra_meta_json",
                ])

    def write(self, rec: TelemetryRecord):
        data = asdict(rec)
        with self.lock:
            with self.path.open("a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    data["record_id"],
                    data["timestamp_iso"],
                    data["gps_lat_deg"],
                    data["gps_lon_deg"],
                    data["gps_alt_m"],
                    data["gps_fix_quality"],
                    data["gps_num_satellites"],
                    data["gps_hdop"],
                    data["flight_distance_m"],
                    data["flight_bearing_deg"],
                    data["flight_ground_speed_mps"],
                    data["dht_temp_c"],
                    data["dht_humidity_pct"],
                    data["vpd_kpa"],
                    data["mlx_canopy_temp_c"],
                    data["sgp30_eco2_ppm"],
                    data["sgp30_tvoc_ppb"],
                    data["image_path"],
                    data["upload_status"],
                    data["upload_http_code"],
                    data["upload_attempts"],
                    data["local_sequence"],
                    data["node_id"],
                    data["battery_level_pct"],
                    data["rssi_dbm"],
                    json.dumps(data["extra_meta"], separators=(",", ":")),
                ])


class CacheWriter:
    def __init__(self, path: Path):
        self.path = path
        self.lock = Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, rec: TelemetryRecord):
        line = json.dumps(asdict(rec), separators=(",", ":"))
        with self.lock:
            with self.path.open("a") as f:
                f.write(line + "\n")

    def load_all(self):
        if not self.path.exists():
            return []
        with self.lock:
            with self.path.open("r") as f:
                lines = [l.strip() for l in f if l.strip()]
        out = []
        for line in lines:
            try:
                obj = json.loads(line)
                out.append(obj)
            except Exception:
                continue
        return out

    def rewrite(self, list_of_dicts):
        tmp = self.path.with_suffix(".tmp")
        with self.lock:
            with tmp.open("w") as f:
                for obj in list_of_dicts:
                    f.write(json.dumps(obj, separators=(",", ":")) + "\n")
            tmp.replace(self.path)


class NodeIdentity:
    def __init__(self):
        self.node_id = self._compute_node_id()

    def _compute_node_id(self):
        hostname = socket.gethostname()
        rand = "".join(random.choices(string.ascii_lowercase + string.digits, k=6))
        return f"{hostname}-{rand}"

    def get_id(self):
        return self.node_id


class DHT11Reader:
    def __init__(self, sensor, pin):
        self.sensor = sensor
        self.pin = pin

    def read(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.pin)
        if humidity is None or temperature is None:
            return None, None
        return float(temperature), float(humidity)


class MLX90614Reader:
    def __init__(self):
        self.sensor = adafruit_mlx90614.MLX90614(I2C_BUS)

    def read(self):
        try:
            return float(self.sensor.object_temperature)
        except Exception:
            return None


class SGP30Reader:
    def __init__(self):
        self.sensor = adafruit_sgp30.Adafruit_SGP30(I2C_BUS)
        self.sensor.iaq_init()
        for _ in range(15):
            time.sleep(1)

    def read(self):
        try:
            eco2, tvoc = self.sensor.iaq_measure()
            return int(eco2), int(tvoc)
        except Exception:
            return None, None


class GPSReader(Thread):
    def __init__(self, at_port, gps_port, at_baud, gps_baud, stop_event: Event):
        super().__init__(daemon=True)
        self.at_port_name = at_port
        self.gps_port_name = gps_port
        self.at_baud = at_baud
        self.gps_baud = gps_baud
        self.stop_event = stop_event
        self.at_ser = None
        self.gps_ser = None
        self.lock = Lock()
        self.latest_fix: GPSFix | None = None

    def run(self):
        self._open_ports()
        while not self.stop_event.is_set():
            fix = self._read_one_fix()
            if fix is not None:
                with self.lock:
                    self.latest_fix = fix
            time.sleep(GPS_READ_INTERVAL_SEC)
        self._close_ports()

    def _open_ports(self):
        try:
            self.at_ser = serial.Serial(self.at_port_name, self.at_baud, timeout=1)
            self.at_ser.write(b"AT+QGPS=1\r")
            time.sleep(0.5)
            _ = self.at_ser.read(1024)
        except Exception:
            self.at_ser = None
        try:
            self.gps_ser = serial.Serial(self.gps_port_name, self.gps_baud, timeout=1)
        except Exception:
            self.gps_ser = None

    def _close_ports(self):
        try:
            if self.at_ser is not None and self.at_ser.is_open:
                self.at_ser.close()
        except Exception:
            pass
        try:
            if self.gps_ser is not None and self.gps_ser.is_open:
                self.gps_ser.close()
        except Exception:
            pass

    def _read_one_fix(self) -> GPSFix | None:
        if self.gps_ser is None:
            return None
        end_time = time.time() + GPS_READ_INTERVAL_SEC
        while time.time() < end_time:
            try:
                line = self.gps_ser.readline().decode(errors="ignore").strip()
            except Exception:
                return None
            if not line:
                continue
            if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                parts = line.split(",")
                if len(parts) < 14:
                    continue
                lat_str = parts[2]
                lat_dir = parts[3]
                lon_str = parts[4]
                lon_dir = parts[5]
                fix_quality_str = parts[6]
                num_sats_str = parts[7]
                hdop_str = parts[8]
                alt_str = parts[9]
                lat, lon = self._parse_nmea_lat_lon(lat_str, lat_dir, lon_str, lon_dir)
                fix_quality = self._safe_int(fix_quality_str)
                num_sats = self._safe_int(num_sats_str)
                hdop = self._safe_float(hdop_str)
                alt = self._safe_float(alt_str)
                return GPSFix(
                    timestamp=time.time(),
                    lat=lat,
                    lon=lon,
                    alt=alt,
                    fix_quality=fix_quality,
                    num_sats=num_sats,
                    hdop=hdop,
                )
        return None

    def _parse_nmea_lat_lon(self, lat_str, lat_dir, lon_str, lon_dir):
        if not lat_str or not lon_str or not lat_dir or not lon_dir:
            return None, None
        try:
            lat_deg = float(lat_str[:2])
            lat_min = float(lat_str[2:])
            lat = lat_deg + lat_min / 60.0
            if lat_dir == "S":
                lat = -lat
            lon_deg = float(lon_str[:3])
            lon_min = float(lon_str[3:])
            lon = lon_deg + lon_min / 60.0
            if lon_dir == "W":
                lon = -lon
            return lat, lon
        except Exception:
            return None, None

    def _safe_int(self, s):
        try:
            return int(s) if s else None
        except Exception:
            return None

    def _safe_float(self, s):
        try:
            return float(s) if s else None
        except Exception:
            return None

    def get_latest_fix(self) -> GPSFix | None:
        with self.lock:
            return self.latest_fix


class CameraController:
    def __init__(self, image_dir: Path):
        self.image_dir = image_dir
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.camera = PiCamera()
        self.camera.resolution = (1280, 720)

    def capture(self, timestamp_iso: str, record_id: str) -> str | None:
        safe_ts = timestamp_iso.replace(":", "").replace("-", "").replace("T", "_")
        filename = f"{record_id}_{safe_ts}.jpg"
        img_path = self.image_dir / filename
        try:
            self.camera.capture(str(img_path), format="jpeg")
            return str(img_path)
        except Exception:
            return None

    def close(self):
        try:
            self.camera.close()
        except Exception:
            pass


class FlightComputer:
    def __init__(self):
        self.state = FlightState()

    def update(self, fix: GPSFix | None):
        if fix is None or fix.lat is None or fix.lon is None:
            return None, None, None, 0.0
        if self.state.last_fix is None or self.state.last_fix.lat is None or self.state.last_fix.lon is None:
            self.state.last_fix = fix
            self.state.last_distance_increment_m = 0.0
            self.state.last_bearing_deg = None
            self.state.last_ground_speed_mps = None
            return 0.0, None, None, 0.0
        dt = max(fix.timestamp - self.state.last_fix.timestamp, 1e-6)
        d, bearing = self._haversine_and_bearing(
            self.state.last_fix.lat, self.state.last_fix.lon, fix.lat, fix.lon
        )
        self.state.total_distance_m += d
        self.state.last_distance_increment_m = d
        self.state.last_bearing_deg = bearing
        self.state.last_ground_speed_mps = d / dt
        self.state.last_fix = fix
        return self.state.total_distance_m, bearing, self.state.last_ground_speed_mps, d

    def _haversine_and_bearing(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c
        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
        return d, bearing


class UplinkWorker(Thread):
    def __init__(self, server_url: str, cache_writer: CacheWriter, stop_event: Event):
        super().__init__(daemon=True)
        self.server_url = server_url
        self.cache_writer = cache_writer
        self.stop_event = stop_event
        self.queue: queue.Queue[TelemetryRecord] = queue.Queue(maxsize=1000)
        self.lock = Lock()

    def run(self):
        while not self.stop_event.is_set():
            try:
                self._process_queue_once()
                self._process_cache_once()
            except Exception:
                pass
            time.sleep(UPLOAD_WORKER_INTERVAL_SEC)

    def _process_queue_once(self):
        for _ in range(10):
            if self.queue.empty():
                break
            rec = self.queue.get_nowait()
            ok, code = self._upload_record(rec)
            if not ok:
                rec.upload_status = "fail"
                rec.upload_http_code = code
                rec.upload_attempts += 1
                self.cache_writer.append(rec)
            else:
                rec.upload_status = "ok"
                rec.upload_http_code = code

    def _process_cache_once(self):
        cached = self.cache_writer.load_all()
        if not cached:
            return
        remaining = []
        for obj in cached:
            rec = TelemetryRecord(**obj)
            ok, code = self._upload_record(rec)
            if not ok:
                rec.upload_status = "fail"
                rec.upload_http_code = code
                rec.upload_attempts = rec.upload_attempts + 1
                remaining.append(asdict(rec))
        self.cache_writer.rewrite(remaining)

    def _upload_record(self, rec: TelemetryRecord):
        if not self._network_available():
            return False, None
        payload, files = self._build_payload(rec)
        try:
            resp = requests.post(self.server_url, data=payload, files=files, timeout=10)
            if "image" in files:
                try:
                    files["image"][1].close()
                except Exception:
                    pass
            return 200 <= resp.status_code < 300, resp.status_code
        except Exception:
            if "image" in files:
                try:
                    files["image"][1].close()
                except Exception:
                    pass
            return False, None

    def _build_payload(self, rec: TelemetryRecord):
        d = asdict(rec)
        files = {}
        image_path = d.pop("image_path", None)
        extra_meta = d.pop("extra_meta", {})
        for k, v in list(d.items()):
            if v is None:
                d[k] = ""
            else:
                d[k] = str(v)
        d["extra_meta_json"] = json.dumps(extra_meta, separators=(",", ":"))
        if image_path and os.path.exists(image_path):
            files["image"] = ("frame.jpg", open(image_path, "rb"), "image/jpeg")
        return d, files

    def _network_available(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(NETWORK_TEST_TIMEOUT)
            s.connect((NETWORK_TEST_HOST, NETWORK_TEST_PORT))
            s.close()
            return True
        except Exception:
            return False

    def submit(self, rec: TelemetryRecord):
        try:
            self.queue.put_nowait(rec)
        except queue.Full:
            rec.upload_status = "queue_full"
            rec.upload_http_code = None
            rec.upload_attempts += 1
            self.cache_writer.append(rec)


class DroneTelemetrySystem:
    def __init__(self):
        self.stop_event = Event()
        self.sequence_counter = 0
        self.node_identity = NodeIdentity()
        self.csv_logger = CSVLogger(CSV_PATH)
        self.cache_writer = CacheWriter(CACHE_PATH)
        self.uplink_worker = UplinkWorker(SERVER_URL, self.cache_writer, self.stop_event)
        self.gps_reader = GPSReader(MODEM_AT_PORT, GPS_NMEA_PORT, MODEM_BAUDRATE, GPS_BAUDRATE, self.stop_event)
        self.camera = CameraController(IMAGE_DIR)
        self.dht_reader = DHT11Reader(DHT_SENSOR, DHT_PIN)
        self.mlx_reader = MLX90614Reader()
        self.sgp_reader = SGP30Reader()
        self.flight_computer = FlightComputer()
        self._register_signal_handlers()
        atexit.register(self.shutdown)

    def _register_signal_handlers(self):
        def handler(signum, frame):
            self.stop_event.set()
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    def start(self):
        self.gps_reader.start()
        self.uplink_worker.start()
        self._main_loop()

    def _main_loop(self):
        while not self.stop_event.is_set():
            now = datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc)
            timestamp_iso = now.isoformat()
            record_id = str(uuid.uuid4())
            local_seq = self._next_sequence()
            gps_fix = self.gps_reader.get_latest_fix()
            total_dist_m, bearing_deg, ground_speed_mps, last_increment = self.flight_computer.update(gps_fix)
            dht_temp_c, dht_humidity = self.dht_reader.read()
            vpd_kpa = self._compute_vpd(dht_temp_c, dht_humidity)
            mlx_canopy_temp_c = self.mlx_reader.read()
            sgp_eco2_ppm, sgp_tvoc_ppb = self.sgp_reader.read()
            image_path = self.camera.capture(timestamp_iso, record_id)
            battery_level_pct = self._estimate_battery()
            rssi_dbm = self._estimate_rssi()
            rec = TelemetryRecord(
                record_id=record_id,
                timestamp_iso=timestamp_iso,
                gps_lat_deg=gps_fix.lat if gps_fix else None,
                gps_lon_deg=gps_fix.lon if gps_fix else None,
                gps_alt_m=gps_fix.alt if gps_fix else None,
                gps_fix_quality=gps_fix.fix_quality if gps_fix else None,
                gps_num_satellites=gps_fix.num_sats if gps_fix else None,
                gps_hdop=gps_fix.hdop if gps_fix else None,
                flight_distance_m=total_dist_m,
                flight_bearing_deg=bearing_deg,
                flight_ground_speed_mps=ground_speed_mps,
                dht_temp_c=dht_temp_c,
                dht_humidity_pct=dht_humidity,
                vpd_kpa=vpd_kpa,
                mlx_canopy_temp_c=mlx_canopy_temp_c,
                sgp30_eco2_ppm=sgp_eco2_ppm,
                sgp30_tvoc_ppb=sgp_tvoc_ppb,
                image_path=image_path,
                upload_status=None,
                upload_http_code=None,
                upload_attempts=0,
                local_sequence=local_seq,
                node_id=self.node_identity.get_id(),
                battery_level_pct=battery_level_pct,
                rssi_dbm=rssi_dbm,
                extra_meta={
                    "last_segment_distance_m": last_increment,
                    "loop_interval_s": SAMPLE_INTERVAL_SEC,
                    "firmware_version": "rpi-drone-telemetry-1.0.0",
                },
            )
            self.csv_logger.write(rec)
            self.uplink_worker.submit(rec)
            self._sleep_precise(SAMPLE_INTERVAL_SEC)

    def _compute_vpd(self, temp_c, rh_pct):
        if temp_c is None or rh_pct is None:
            return None
        try:
            es = 0.6108 * math.exp((17.27 * temp_c) / (temp_c + 237.3))
            e = es * (rh_pct / 100.0)
            vpd = es - e
            return round(vpd, 4)
        except Exception:
            return None

    def _next_sequence(self):
        self.sequence_counter += 1
        return self.sequence_counter

    def _sleep_precise(self, interval):
        start = time.time()
        while True:
            if self.stop_event.is_set():
                break
            elapsed = time.time() - start
            if elapsed >= interval:
                break
            time.sleep(0.05)

    def _estimate_battery(self):
        return None

    def _estimate_rssi(self):
        return None

    def shutdown(self):
        self.stop_event.set()
        try:
            if self.gps_reader.is_alive():
                self.gps_reader.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.uplink_worker.is_alive():
                self.uplink_worker.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.camera.close()
        except Exception:
            pass


def main():
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    system = DroneTelemetrySystem()
    system.start()


if __name__ == "__main__":
    main()