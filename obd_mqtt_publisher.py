#!/usr/bin/env python3
import argparse
import json
import os
import socket
import sys
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
import obd  # python-OBD


def iso_utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_mqtt_client(args) -> mqtt.Client:
    client_id = args.client_id or f"pi-obd-{socket.gethostname()}"
    client = mqtt.Client(client_id=client_id, clean_session=True)

    if args.username:
        client.username_pw_set(args.username, args.password or "")

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


def response_to_value(resp):
    """
    Convert python-OBD Response -> JSON-safe value.
    - If no data, return None.
    - If value has magnitude + units, emit {value, unit}.
    - Else try float/str.
    """
    if resp is None or resp.is_null():
        return None

    v = resp.value
    try:
        # Pint Quantity-like from python-OBD: v.magnitude + v.units
        mag = getattr(v, "magnitude", None)
        unit = getattr(v, "units", None)
        if mag is not None and unit is not None:
            # magnitude may be numpy type; float() makes it JSON-clean
            return {"value": float(mag), "unit": str(unit)}
    except Exception:
        pass

    # fallback: try numeric, else string
    try:
        return float(v)
    except Exception:
        return str(v)


def main():
    p = argparse.ArgumentParser(description="Publish USB ELM327 OBD2 data over MQTT (JSON).")
    p.add_argument("--host", default=os.getenv("MQTT_HOST", "localhost"))
    p.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    p.add_argument("--topic-base", default=os.getenv("MQTT_TOPIC_BASE", "sensors/obd"))
    p.add_argument("--qos", type=int, choices=[0, 1, 2], default=int(os.getenv("MQTT_QOS", "0")))
    p.add_argument("--retain", action="store_true", default=os.getenv("MQTT_RETAIN", "0") == "1")
    p.add_argument("--client-id", default=os.getenv("MQTT_CLIENT_ID"))
    p.add_argument("--username", default=os.getenv("MQTT_USERNAME"))
    p.add_argument("--password", default=os.getenv("MQTT_PASSWORD"))

    # OBD settings
    p.add_argument("--device", default=os.getenv("OBD_DEVICE", ""),
                   help="Serial device like /dev/ttyUSB0 (leave empty for auto)")
    p.add_argument("--baud", type=int, default=int(os.getenv("OBD_BAUD", "0")),
                   help="Force baud (0 = auto). ELM327 defaults are commonly 9600 or 38400.")
    p.add_argument("--fast", action="store_true", help="Enable faster querying (may reduce reliability on some cars/adapters)")
    p.add_argument("--rate-hz", type=float, default=float(os.getenv("OBD_RATE_HZ", "2.0")),
                   help="Publish rate for the payload (queries happen each loop).")
    p.add_argument("--print", action="store_true", help="Also print each published JSON payload")

    args = p.parse_args()

    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be > 0")
    period = 1.0 / args.rate_hz

    hostname = socket.gethostname()

    # MQTT
    client = build_mqtt_client(args)
    client.connect_async(args.host, args.port, keepalive=30)
    client.loop_start()

    # OBD connection
    # python-OBD can auto-detect port + baud in many cases. :contentReference[oaicite:1]{index=1}
    obd_kwargs = {}
    if args.device:
        obd_kwargs["portstr"] = args.device
    if args.baud and args.baud > 0:
        obd_kwargs["baudrate"] = args.baud
    obd_kwargs["fast"] = bool(args.fast)

    connection = None

    # Commands to query (common ones; availability depends on vehicle)
    cmds = {
        "rpm": obd.commands.RPM,
        "speed": obd.commands.SPEED,
        "coolant_temp": obd.commands.COOLANT_TEMP,
        "engine_load": obd.commands.ENGINE_LOAD,
        "throttle_pos": obd.commands.THROTTLE_POS,
        "intake_temp": obd.commands.INTAKE_TEMP,
        "maf": obd.commands.MAF,
        "fuel_level": obd.commands.FUEL_LEVEL,
        "battery_voltage": obd.commands.CONTROL_MODULE_VOLTAGE,
    }

    try:
        while True:
            if connection is None or not connection.is_connected():
                try:
                    connection = obd.OBD(**obd_kwargs)  # may take a moment (protocol detection)
                    if connection.is_connected():
                        print(f"[obd] connected: port={connection.port_name()} protocol={connection.protocol_name()}")
                    else:
                        print("[obd] not connected (ignition ON? correct port?)", file=sys.stderr)
                        time.sleep(2)
                        continue
                except Exception as e:
                    print(f"[obd] connect error: {e}", file=sys.stderr)
                    time.sleep(2)
                    continue

            data = {}
            for name, cmd in cmds.items():
                try:
                    resp = connection.query(cmd)
                    data[name] = response_to_value(resp)
                except Exception as e:
                    data[name] = None

            payload = {
                "ts": iso_utc_now(),
                "host": hostname,
                "sensor": "ELM327",
                "device": args.device or "auto",
                "baud": args.baud or "auto",
                "protocol": connection.protocol_name() if connection else None,
                "data": data,
            }

            topic = f"{args.topic_base}/live"
            msg = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
            client.publish(topic, msg, qos=args.qos, retain=args.retain)

            if args.print:
                print(msg)

            time.sleep(period)

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
            if connection is not None:
                connection.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
