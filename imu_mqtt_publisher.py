#!/usr/bin/env python3
import argparse
import json
import os
import socket
import sys
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
from smbus2 import SMBus


def iso_utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class LSM6DS3:
    """
    Minimal LSM6DS3 I2C driver (accel + gyro + temp).
    Default: accel ±2g, gyro ±245 dps, ODR 104 Hz.
    WHO_AM_I should be 0x69. :contentReference[oaicite:1]{index=1}
    """

    # Common I2C addresses (depends on SA0/SDO pin)
    ADDR_LO = 0x6A
    ADDR_HI = 0x6B

    # Registers
    WHO_AM_I = 0x0F
    CTRL1_XL = 0x10
    CTRL2_G  = 0x11
    CTRL3_C  = 0x12

    OUT_TEMP_L = 0x20  # temp L, temp H at 0x21
    OUTX_L_G   = 0x22  # gyro 6 bytes to 0x27
    OUTX_L_XL  = 0x28  # accel 6 bytes to 0x2D :contentReference[oaicite:2]{index=2}

    def __init__(self, bus: int = 1, address: int = ADDR_LO):
        self.bus_num = bus
        self.addr = address
        self.bus = SMBus(bus)

        who = self._read_u8(self.WHO_AM_I)
        if who != 0x69:
            raise RuntimeError(
                f"LSM6DS3 not found at 0x{address:02X} on /dev/i2c-{bus} "
                f"(WHO_AM_I=0x{who:02X}, expected 0x69)."
            )

        # CTRL3_C: IF_INC=1 (auto-increment register address for multi-byte reads)
        self._write_u8(self.CTRL3_C, 0x04)

        # CTRL1_XL: ODR_XL=104 Hz (0b0100 << 4), FS_XL=±2g (00), BW_XL=100 Hz (00)
        self._write_u8(self.CTRL1_XL, 0x40)

        # CTRL2_G: ODR_G=104 Hz (0b0100 << 4), FS_G=245 dps (00)
        self._write_u8(self.CTRL2_G, 0x40)

        # Scale factors for chosen full-scales (from datasheet typical sensitivities)
        # accel ±2g: 0.061 mg/LSB  -> 0.000061 g/LSB
        # gyro  245 dps: 8.75 mdps/LSB -> 0.00875 dps/LSB
        self.accel_g_per_lsb = 0.000061
        self.gyro_dps_per_lsb = 0.00875

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _read_u8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

    def _write_u8(self, reg: int, val: int):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    @staticmethod
    def _to_int16(lo: int, hi: int) -> int:
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def _read_block(self, start_reg: int, length: int) -> list[int]:
        return list(self.bus.read_i2c_block_data(self.addr, start_reg, length))

    def read_accel_g(self) -> dict:
        b = self._read_block(self.OUTX_L_XL, 6)
        x = self._to_int16(b[0], b[1]) * self.accel_g_per_lsb
        y = self._to_int16(b[2], b[3]) * self.accel_g_per_lsb
        z = self._to_int16(b[4], b[5]) * self.accel_g_per_lsb
        return {"x": x, "y": y, "z": z}

    def read_gyro_dps(self) -> dict:
        b = self._read_block(self.OUTX_L_G, 6)
        x = self._to_int16(b[0], b[1]) * self.gyro_dps_per_lsb
        y = self._to_int16(b[2], b[3]) * self.gyro_dps_per_lsb
        z = self._to_int16(b[4], b[5]) * self.gyro_dps_per_lsb
        return {"x": x, "y": y, "z": z}

    def read_temp_c(self) -> float:
        b = self._read_block(self.OUT_TEMP_L, 2)
        raw = self._to_int16(b[0], b[1])
        # LSM6DS3 temp: 16 LSB/°C, 0 at 25°C (per datasheet family convention)
        return 25.0 + (raw / 16.0)


def build_mqtt_client(args) -> mqtt.Client:
    client_id = args.client_id or f"pi-imu-{socket.gethostname()}"
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


def main():
    p = argparse.ArgumentParser(description="Publish Grove LSM6DS3 IMU data over MQTT (JSON).")
    p.add_argument("--host", default=os.getenv("MQTT_HOST", "localhost"))
    p.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    p.add_argument("--topic-base", default=os.getenv("MQTT_TOPIC_BASE", "sensors/imu"))
    p.add_argument("--rate-hz", type=float, default=float(os.getenv("IMU_RATE_HZ", "20")))
    p.add_argument("--qos", type=int, choices=[0, 1, 2], default=int(os.getenv("MQTT_QOS", "0")))
    p.add_argument("--retain", action="store_true", default=os.getenv("MQTT_RETAIN", "0") == "1")
    p.add_argument("--client-id", default=os.getenv("MQTT_CLIENT_ID"))
    p.add_argument("--username", default=os.getenv("MQTT_USERNAME"))
    p.add_argument("--password", default=os.getenv("MQTT_PASSWORD"))

    # LSM6DS3 options
    p.add_argument("--i2c-bus", type=int, default=int(os.getenv("IMU_I2C_BUS", "1")))
    p.add_argument("--i2c-addr", default=os.getenv("IMU_I2C_ADDR", "0x6A"),
                   help="LSM6DS3 I2C address, usually 0x6A or 0x6B")

    p.add_argument("--print", action="store_true", help="Also print each JSON payload to stdout")
    args = p.parse_args()

    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be > 0")
    period = 1.0 / args.rate_hz

    i2c_addr = int(args.i2c_addr, 16)

    imu = LSM6DS3(bus=args.i2c_bus, address=i2c_addr)

    client = build_mqtt_client(args)
    client.connect_async(args.host, args.port, keepalive=30)
    client.loop_start()

    hostname = socket.gethostname()

    try:
        next_t = time.monotonic()
        while True:
            payload = {
                "ts": iso_utc_now(),
                "host": hostname,
                "sensor": "LSM6DS3",
                "rate_hz": args.rate_hz,
                "accel_g": imu.read_accel_g(),
                "gyro_dps": imu.read_gyro_dps(),
                "temp_c": imu.read_temp_c(),
            }

            topic = f"{args.topic_base}/raw"
            msg = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
            client.publish(topic, msg, qos=args.qos, retain=args.retain)

            if args.print:
                print(msg)

            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

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
        imu.close()


if __name__ == "__main__":
    main()
