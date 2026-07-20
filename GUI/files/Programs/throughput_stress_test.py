#!/usr/bin/env python3
"""
Stress-test throughput protokol Modbus TCP/IP (TERPISAH dari log siklus N=100).

Tujuan: mengukur KAPASITAS ASLI protokol Modbus dengan menembak transaksi
read back-to-back (secepat mungkin) tanpa menunggu robot. Menghasilkan metrik
gaya tabel skripsi:

    Avg / Min / Max Response Time (ms)
    Throughput (tps)
    Packet Loss Rate (%)

Ini memakai frame Modbus SUNGGUHAN (function code 01/02/03/04) via pymodbus,
sama seperti yang dipakai aplikasi di Commander_feature_adapters.py.

Contoh pemakaian
----------------
    # 1000 transaksi read coil address 0 (trigger_pick), secepat mungkin:
    python throughput_stress_test.py 192.168.1.10

    # Read holding register, 500 transaksi, tulis hasil ke Excel:
    python throughput_stress_test.py 192.168.1.10 --fn holding --address 0 -n 500 \
        --out hasil_throughput

    # Mode "paced" 10 Hz untuk meniru persis tabel di gambar (Trigger Interval ~0,1 s):
    python throughput_stress_test.py 192.168.1.10 --mode paced --interval 0.1 -n 100

Catatan penting
---------------
- Harus konek ke PLC NYATA. Jangan pakai IP "MOCK" -> hasil tak bermakna.
- Satu "transaksi" = SATU function code read (round-trip request->response).
- Throughput dihitung: jumlah_transaksi / total_waktu_dinding (detik).
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from datetime import datetime

try:
    from pymodbus.client import ModbusTcpClient
except Exception as exc:  # pragma: no cover
    print(f"[FATAL] pymodbus tidak tersedia: {exc}", file=sys.stderr)
    sys.exit(2)


# Peta --fn ke nama method read pymodbus. Semua read 1 register/bit.
READ_METHODS = {
    "coil": "read_coils",                # FC 01
    "discrete": "read_discrete_inputs",  # FC 02
    "holding": "read_holding_registers", # FC 03
    "input": "read_input_registers",     # FC 04
}


def call_modbus(method, kwargs: dict, slave_id: int):
    """Panggil method read dengan keyword slave-id yang cocok lintas versi pymodbus.

    Sama seperti _call_modbus() di aplikasi: pymodbus lama pakai 'unit'/'slave',
    3.x pakai 'device_id'. Coba berurutan sampai ada yang diterima.
    """
    for device_keyword in ("device_id", "slave", "unit"):
        try:
            return method(**kwargs, **{device_keyword: slave_id})
        except TypeError as exc:
            msg = str(exc).lower()
            if not (device_keyword in msg and "unexpected keyword" in msg):
                raise
    return method(**kwargs)


def run_stress_test(args) -> dict:
    method_name = READ_METHODS[args.fn]
    read_kwargs = {"address": args.address, "count": args.read_count}

    client = ModbusTcpClient(host=args.ip, port=args.port, timeout=args.timeout)
    if not client.connect():
        raise ConnectionError(f"Gagal konek ke {args.ip}:{args.port}")
    print(f"[OK] Terhubung ke {args.ip}:{args.port} (slave_id={args.slave})")
    print(f"     Transaksi: {method_name}(address={args.address}, count={args.read_count})")
    print(f"     Mode: {args.mode}"
          + (f", interval target={args.interval}s" if args.mode == "paced" else " (secepat mungkin)"))

    method = getattr(client, method_name)

    # Warm-up: buang beberapa transaksi pertama (koneksi TCP baru, cache PLC).
    for _ in range(max(0, args.warmup)):
        try:
            call_modbus(method, read_kwargs, args.slave)
        except Exception:
            pass

    latencies_ms: list[float] = []   # RT tiap transaksi (hanya yang mengembalikan)
    intervals_s: list[float] = []    # jarak antar-awal request (Trigger Interval)
    statuses: list[str] = []         # "OK" / "Loss" / "Timeout"
    n = args.n

    print(f"[..] Menjalankan {n} transaksi...")
    prev_start = None
    t_wall_start = time.perf_counter()

    for i in range(n):
        if args.mode == "paced" and prev_start is not None:
            # Jaga interval target (mis. 0,1 s) untuk meniru poll 10 Hz.
            sleep_left = args.interval - (time.perf_counter() - prev_start)
            if sleep_left > 0:
                time.sleep(sleep_left)

        t0 = time.perf_counter()
        if prev_start is not None:
            intervals_s.append(t0 - prev_start)
        prev_start = t0

        try:
            result = call_modbus(method, read_kwargs, args.slave)
            t1 = time.perf_counter()
            if result is None or (hasattr(result, "isError") and result.isError()):
                statuses.append("Loss")
            else:
                statuses.append("OK")
                latencies_ms.append((t1 - t0) * 1000.0)
        except Exception:
            # timeout / socket error / link drop = paket hilang
            statuses.append("Timeout")

    t_wall_end = time.perf_counter()
    client.close()

    elapsed_s = max(t_wall_end - t_wall_start, 1e-9)
    ok_count = statuses.count("OK")
    loss_count = n - ok_count
    throughput_tps = n / elapsed_s
    packet_loss_pct = loss_count / n * 100.0 if n else 0.0

    return {
        "args": args,
        "method_name": method_name,
        "n": n,
        "elapsed_s": elapsed_s,
        "ok_count": ok_count,
        "loss_count": loss_count,
        "throughput_tps": throughput_tps,
        "packet_loss_pct": packet_loss_pct,
        "avg_rt": statistics.fmean(latencies_ms) if latencies_ms else 0.0,
        "min_rt": min(latencies_ms) if latencies_ms else 0.0,
        "max_rt": max(latencies_ms) if latencies_ms else 0.0,
        "avg_interval": statistics.fmean(intervals_s) if intervals_s else 0.0,
        "latencies_ms": latencies_ms,
        "intervals_s": intervals_s,
        "statuses": statuses,
    }


def _crit(passed: bool) -> str:
    return "OK" if passed else "TIDAK MEMENUHI"


def print_report(res: dict, args) -> None:
    print("\n" + "=" * 64)
    print("  RINGKASAN PERFORMA PROTOKOL KOMUNIKASI MODBUS TCP/IP")
    print("=" * 64)
    print(f"  Total transaksi        : {res['n']}")
    print(f"  Berhasil / Hilang      : {res['ok_count']} / {res['loss_count']}")
    print(f"  Total waktu            : {res['elapsed_s']:.3f} s")
    print(f"  Rata-rata interval     : {res['avg_interval']*1000:.2f} ms "
          f"({res['avg_interval']:.4f} s)")
    print("-" * 64)
    print(f"  {'Metrik':<22}{'Nilai':<16}Kriteria")
    print("-" * 64)
    print(f"  {'Avg Response Time':<22}{res['avg_rt']:>7.2f} ms      "
          f"<= 100 ms   [{_crit(res['avg_rt'] <= 100)}]")
    print(f"  {'Min Response Time':<22}{res['min_rt']:>7.2f} ms      -")
    print(f"  {'Max Response Time':<22}{res['max_rt']:>7.2f} ms      "
          f"<= 100 ms   [{_crit(res['max_rt'] <= 100)}]")
    print(f"  {'Throughput':<22}{res['throughput_tps']:>7.2f} tps     "
          f">= 10 tps   [{_crit(res['throughput_tps'] >= 10)}]")
    print(f"  {'Packet Loss Rate':<22}{res['packet_loss_pct']:>7.2f} %       "
          f"<= 1 %      [{_crit(res['packet_loss_pct'] <= 1)}]")
    print("=" * 64)

    # Tabel contoh per-trial (10 sampel merata) -- gaya tabel kedua di gambar.
    trials = min(args.trials, res["n"])
    if trials > 0 and res["latencies_ms"]:
        print(f"\n  Contoh {trials} trial (sampel merata):")
        print(f"  {'Trial':<7}{'Response Time (ms)':<20}{'Status':<10}{'Interval (s)':<12}")
        step = max(1, res["n"] // trials)
        shown = 0
        for idx in range(0, res["n"], step):
            if shown >= trials:
                break
            rt = ""
            if idx < len(res["latencies_ms"]):
                rt = f"{res['latencies_ms'][idx]:.1f}"
            interval = ""
            if idx < len(res["intervals_s"]):
                interval = f"{res['intervals_s'][idx]:.3f}"
            status = res["statuses"][idx] if idx < len(res["statuses"]) else ""
            print(f"  {idx+1:<7}{rt:<20}{status:<10}{interval:<12}")
            shown += 1


def write_excel(res: dict, out_prefix: str) -> str | None:
    try:
        from openpyxl import Workbook
    except Exception as exc:
        print(f"[warn] openpyxl tidak tersedia, lewati tulis Excel: {exc}")
        return None

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = f"{out_prefix}_{stamp}.xlsx"
    wb = Workbook()

    ws = wb.active
    ws.title = "Data"
    ws.append(["No.", "Response Time (ms)", "Packet Status", "Interval (s)"])
    lat = res["latencies_ms"]
    iv = res["intervals_s"]
    st = res["statuses"]
    # Baris per-transaksi; RT hanya untuk yang OK (yang Loss/Timeout kosong).
    li = 0
    for i in range(res["n"]):
        status = st[i] if i < len(st) else ""
        rt = ""
        if status == "OK" and li < len(lat):
            rt = round(lat[li], 3)
            li += 1
        interval = round(iv[i - 1], 4) if 0 < i <= len(iv) else ""
        ws.append([i + 1, rt, status, interval])

    rs = wb.create_sheet("Ringkasan")
    rs.append(["Metrik", "Nilai", "Kriteria"])
    rs.append(["Avg Response Time", f"{res['avg_rt']:.2f} ms", "<= 100 ms"])
    rs.append(["Min Response Time", f"{res['min_rt']:.2f} ms", "-"])
    rs.append(["Max Response Time", f"{res['max_rt']:.2f} ms", "<= 100 ms"])
    rs.append(["Throughput", f"{res['throughput_tps']:.2f} tps", ">= 10 tps"])
    rs.append(["Packet Loss Rate", f"{res['packet_loss_pct']:.2f} %", "<= 1 %"])
    rs.append(["Total Transaksi", res["n"], "-"])
    rs.append(["Total Waktu (s)", round(res["elapsed_s"], 3), "-"])

    wb.save(path)
    return path


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Stress-test throughput protokol Modbus TCP/IP (terpisah dari log siklus).",
    )
    p.add_argument("ip", help="IP PLC (mis. 192.168.1.10). JANGAN 'MOCK'.")
    p.add_argument("--port", type=int, default=502, help="Port Modbus TCP (default 502)")
    p.add_argument("--slave", type=int, default=1, help="Slave/Unit ID (default 1)")
    p.add_argument("-n", type=int, default=1000, help="Jumlah transaksi (default 1000)")
    p.add_argument("--fn", choices=list(READ_METHODS), default="coil",
                   help="Jenis read: coil/discrete/holding/input (default coil)")
    p.add_argument("--address", type=int, default=0,
                   help="Alamat register/coil yang dibaca (default 0 = trigger_pick)")
    p.add_argument("--read-count", type=int, default=1,
                   help="Jumlah register/bit per transaksi (default 1)")
    p.add_argument("--timeout", type=float, default=3.0, help="Timeout per transaksi, detik (default 3.0)")
    p.add_argument("--warmup", type=int, default=5, help="Transaksi warm-up yang dibuang (default 5)")
    p.add_argument("--mode", choices=["burst", "paced"], default="burst",
                   help="burst=secepat mungkin (default); paced=jaga interval tetap")
    p.add_argument("--interval", type=float, default=0.1,
                   help="Target interval antar-request untuk mode paced, detik (default 0.1)")
    p.add_argument("--trials", type=int, default=10, help="Jumlah sampel di tabel contoh (default 10)")
    p.add_argument("--out", default=None,
                   help="Prefix nama file Excel hasil (mis. hasil_throughput). Kosong = tidak menulis file.")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if str(args.ip).strip().upper() == "MOCK":
        print("[FATAL] IP 'MOCK' tidak valid untuk stress-test. Pakai IP PLC nyata.",
              file=sys.stderr)
        return 2
    try:
        res = run_stress_test(args)
    except Exception as exc:
        print(f"[FATAL] {exc}", file=sys.stderr)
        return 1

    print_report(res, args)

    if args.out:
        path = write_excel(res, args.out)
        if path:
            print(f"\n[OK] Hasil ditulis ke: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
