#!/usr/bin/env python3
import argparse
import json
import os
import socket
import sys
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
import serial
import pynmea2


def iso_utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_mqtt_client(args) -> mqtt.Client:
    client_id = args.client_id or f"pi-gps-{socket.gethostname()}"
    client = mqtt.Client(client_id=client_id, clean_session=True)

    if args.username:
        client.username_pw_set(args.username, args.password or "")

    # Last will
    client.will_set(
        topic=f"{args.topic_base}/status",
        payload=json.dumps({"status": "offline", "ts": iso_utc_now()}),
        qos=args.qos,
        retain=True,
    )

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            c.publish(
                f"{args.topic_base}/status",
                json.dumps({"status": "online", "ts": iso_utc_now()}),
                qos=args.qos,
                retain=True,
            )
        else:
            print(f"[mqtt] connect failed rc={rc}", file=sys.stderr)

    client.on_connect = on_connect
    return client


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    # Air530 outputs standard NMEA sentences over UART. Default baud commonly 9600. :contentReference[oaicite:1]{index=1}
    return serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
    )


def nmea_latlon_to_float(lat, lat_dir, lon, lon_dir):
    """pynmea2 already gives decimal degrees for many sentences, but keep safe fallback."""
    try:
        # If pynmea2 already parsed to float degrees:
        lat_f = float(lat) if lat not in (None, "") else None
        lon_f = float(lon) if lon not in (None, "") else None
        return lat_f, lon_f
    except (ValueError, TypeError):
        return None, None


def main():
    p = argparse.ArgumentParser(description="Publish Grove GPS (Air530) NMEA fixes over MQTT (JSON).")
    p.add_argument("--host", default=os.getenv("MQTT_HOST", "localhost"))
    p.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    p.add_argument("--topic-base", default=os.getenv("MQTT_TOPIC_BASE", "sensors/gps"))
    p.add_argument("--qos", type=int, choices=[0, 1, 2], default=int(os.getenv("MQTT_QOS", "0")))
    p.add_argument("--retain", action="store_true", default=os.getenv("MQTT_RETAIN", "0") == "1")
    p.add_argument("--client-id", default=os.getenv("MQTT_CLIENT_ID"))
    p.add_argument("--username", default=os.getenv("MQTT_USERNAME"))
    p.add_argument("--password", default=os.getenv("MQTT_PASSWORD"))

    # Serial / GPS
    p.add_argument("--serial", default=os.getenv("GPS_SERIAL", "/dev/serial0"),
                   help="UART device (common: /dev/serial0, /dev/ttyAMA0, /dev/ttyS0, or a USB adapter like /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=int(os.getenv("GPS_BAUD", "9600")),
                   help="Air530 default is typically 9600")
    p.add_argument("--read-timeout", type=float, default=float(os.getenv("GPS_TIMEOUT", "1.0")))
    p.add_argument("--publish-every", type=float, default=float(os.getenv("GPS_PUBLISH_EVERY", "1.0")),
                   help="Seconds. GPS is usually 1Hz by default.")
    p.add_argument("--print", action="store_true", help="Also print each published JSON payload")
    args = p.parse_args()

    hostname = socket.gethostname()

    # MQTT
    client = build_mqtt_client(args)
    client.connect_async(args.host, args.port, keepalive=30)
    client.loop_start()

    # State (we publish a “best known fix” periodically)
    last_fix = {
        "ts": None,
        "lat": None,
        "lon": None,
        "alt_m": None,
        "speed_knots": None,
        "speed_mps": None,
        "track_deg": None,
        "hdop": None,
        "num_sats": None,
        "fix_quality": None,   # GPS quality indicator (GGA)
        "fix_type": None,      # RMC status / GSA mode etc (if available)
    }
    last_publish_t = 0.0

    ser = None

    try:
        while True:
            if ser is None or not ser.is_open:
                try:
                    ser = open_serial(args.serial, args.baud, args.read_timeout)
                    # flush any partial line
                    ser.reset_input_buffer()
                    print(f"[gps] opened {args.serial} @ {args.baud} baud")
                except Exception as e:
                    print(f"[gps] failed to open serial: {e}", file=sys.stderr)
                    time.sleep(2)
                    continue

            try:
                line = ser.readline()
                if not line:
                    # no data this cycle
                    pass
                else:
                    try:
                        s = line.decode("ascii", errors="ignore").strip()
                        if not s.startswith("$"):
                            continue
                        msg = pynmea2.parse(s)

                        # GGA: time, lat/lon, fix quality, num satellites, HDOP, altitude
                        if msg.sentence_type == "GGA":
                            lat, lon = nmea_latlon_to_float(msg.latitude, msg.lat_dir, msg.longitude, msg.lon_dir)
                            last_fix["lat"] = lat
                            last_fix["lon"] = lon
                            last_fix["fix_quality"] = int(msg.gps_qual) if msg.gps_qual not in (None, "") else None
                            last_fix["num_sats"] = int(msg.num_sats) if msg.num_sats not in (None, "") else None
                            last_fix["hdop"] = float(msg.horizontal_dil) if msg.horizontal_dil not in (None, "") else None
                            last_fix["alt_m"] = float(msg.altitude) if msg.altitude not in (None, "") else None

                        # RMC: time/date, lat/lon, speed (knots), course (track), status A/V
                        elif msg.sentence_type == "RMC":
                            lat, lon = nmea_latlon_to_float(msg.latitude, msg.lat_dir, msg.longitude, msg.lon_dir)
                            last_fix["lat"] = lat
                            last_fix["lon"] = lon
                            if msg.spd_over_grnd not in (None, ""):
                                spd_kn = float(msg.spd_over_grnd)
                                last_fix["speed_knots"] = spd_kn
                                last_fix["speed_mps"] = spd_kn * 0.514444
                            if msg.true_course not in (None, ""):
                                last_fix["track_deg"] = float(msg.true_course)
                            # status: A = valid, V = void
                            last_fix["fix_type"] = getattr(msg, "status", None)

                        # You can extend with GSA/GSV if you want.
                    except pynmea2.ParseError:
                        pass

                # Publish periodically (even if only partial info is available)
                now = time.monotonic()
                if now - last_publish_t >= args.publish_every:
                    payload = {
                        "ts": iso_utc_now(),
                        "host": hostname,
                        "sensor": "Air530",
                        "serial": args.serial,
                        "baud": args.baud,
                        "fix": last_fix,
                    }
                    topic = f"{args.topic_base}/fix"
                    msg = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)

                    client.publish(topic, msg, qos=args.qos, retain=args.retain)
                    if args.print:
                        print(msg)
                    last_publish_t = now

            except (serial.SerialException, OSError) as e:
                print(f"[gps] serial error: {e} (reopening)", file=sys.stderr)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            client.publish(
                f"{args.topic_base}/status",
                json.dumps({"status": "offline", "ts": iso_utc_now()}),
                qos=args.qos,
                retain=True,
            )
        except Exception:
            pass
        client.loop_stop()
        client.disconnect()
        try:
            if ser:
                ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
