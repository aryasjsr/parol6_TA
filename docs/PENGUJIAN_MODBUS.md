# Pengujian Modbus — Blok A & Blok B

Dokumen ini berisi panduan langkah demi langkah untuk menjalankan dua skenario uji performa Modbus yang tersedia di aplikasi `Source controller.py` (branch `comsoft`):

- **Blok A — Protocol Performance Test**: mengukur karakteristik komunikasi Modbus TCP/IP pada level protokol (response time, throughput, packet loss).
- **Blok B — Cycle Time Test**: mengukur durasi end-to-end satu siklus kerja yang di-trigger oleh sinyal Modbus (mis. PLC memberi trigger pick → robot menjalankan program → PLC menerima sinyal done/error).

Hasil tiap iterasi/cycle dicatat di **logging table** pada tab Modbus dan dapat diekspor ke berkas `.xlsx` (2 sheet: `Samples` + `Summary`) atau `.csv`. Catatan: data **session-only** — jika aplikasi ditutup tanpa export, riwayat hilang.

---

## Persiapan Umum

1. Pastikan dependency sudah terpasang. Dari root repo:
   ```
   pip install -r PAROL6_app/requirements.txt
   ```
   Paket relevan: `pymodbus>=3.6.0`, `openpyxl>=3.1.2`.
2. Konfigurasi awal disimpan di `PAROL6_app/config.json` di bagian `modbus.block_a` dan `modbus.block_b`. Field yang diisi di UI akan otomatis tersimpan kembali ke file ini saat test dimulai.
3. **Mode MOCK vs PLC fisik:**
   - `IP = MOCK`: simulasi internal, tidak menyentuh jaringan — cocok untuk dev/dry-run.
   - `IP = <alamat PLC>`: koneksi nyata via `pymodbus`. Pastikan PLC aktif, firewall membuka port 502, dan `slave_id` cocok.
4. Jalankan aplikasi dari direktori `PAROL6_app/GUI/files/`:
   ```
   python GUI_PAROL_latest.py
   ```
   Buka tab **Modbus** pada window utama.

---

## Pengujian Blok A — Protocol Performance Test

Tujuan: ukur waktu respons, throughput, dan packet loss saat melakukan request Modbus berulang.

### Langkah

1. Di tab Modbus, perluas section **Blok A - Protocol Performance Test**.
2. Isi field koneksi:
   - **IP** — `MOCK` untuk simulasi, atau IP PLC (mis. `192.168.1.10`).
   - **Port** — biasanya `502`.
   - **Slave** — slave id (mis. `1`).
   - **Timeout ms** — batas waktu per request (default `1000`).
3. Pilih **Function** dari dropdown:
   - `read_holding_register`, `read_input_register`, `read_coil`, `read_discrete_input` untuk uji baca.
   - `write_register`, `write_coil` untuk uji tulis.
4. Isi parameter request:
   - **Address** — alamat register/coil tujuan (mis. `100`).
   - **Count** — jumlah register/coil per request (umumnya `1`).
   - **N** — total iterasi test (mis. `1000`).
5. Klik **Start Block A**. Pantau:
   - Label `progress` berubah jadi `RUNNING` dengan counter `completed/target`.
   - Metrik live: `avg_rt_ms`, `throughput_rps`, `packet_loss_pct`, `success_count`, `failed_count`, `timeout_count`.
   - Tabel di bawah menampilkan tiap iterasi: `N`, `Time`, `RT (ms)`, `Status`, `Detail`.
6. (Opsional) Klik **Stop** untuk menghentikan sebelum N tercapai. Data yang sudah masuk tabel tetap dipertahankan.
7. Setelah test selesai/dihentikan, klik **Export XLSX**. Pilih lokasi simpan (default: `modbus_block_a_<tanggal>_<jam>.xlsx`).
8. Buka file hasil export:
   - Sheet **Block A Samples** — semua iterasi (kolom: N, Timestamp ISO, Response (ms), Status, Detail).
   - Sheet **Summary** — metadata test (started_at, finished_at, target, completed, stopped_early, stat.*, setting.*).

### Interpretasi Hasil

- **avg_rt_ms tinggi & std besar** → ada jitter di jaringan/PLC; cek beban switch atau ganti kabel.
- **throughput_rps rendah jauh dari ekspektasi** → bottleneck di PLC scan cycle atau timeout terlalu besar.
- **packet_loss_pct > 0 saat mode MOCK** → indikasi bug di aplikasi (laporkan).
- **packet_loss_pct tinggi saat PLC fisik** → naikkan `Timeout ms`, cek konfigurasi PLC, periksa kabel/switch.
- **timeout_count > 0** sementara `failed_count = 0` → koneksi ada tapi lambat; turunkan `Count` atau frekuensi.

### Tip Reproducibility

Catat saat eksperimen: versi pymodbus, OS host, IP PLC, jenis switch/cable, durasi total test. Sheet `Summary` sudah berisi `started_at` & `finished_at` epoch yang ter-konversi ISO untuk audit.

---

## Pengujian Blok B — Cycle Time Test

Tujuan: ukur durasi penuh satu siklus kerja yang di-trigger dari Modbus (mis. PLC memerintahkan robot pick → robot menyelesaikan program → status done/error dikirim balik ke PLC).

### Prasyarat

- Tab **Address Mapping** sudah berisi tiga sinyal yang akan dipakai:
  - **trigger** (default name `trigger_pick`) — sinyal masuk dari PLC, biasanya coil RW=read.
  - **done** (default `cycle_done`) — sinyal keluar untuk konfirmasi sukses, coil RW=write.
  - **error** (default `error_flag`) — sinyal keluar untuk error, coil RW=write.
- Program robot yang akan dieksekusi sudah di-load (tombol Execute Program pada tab lain berfungsi normal).
- Sumber trigger sudah siap (PLC nyata, atau emulator coil yang dapat di-toggle).

### Langkah

1. Pastikan koneksi Modbus aktif (klik **Connect** di section Modbus Connection Setup, atau biarkan Blok B yang memicu koneksi otomatis).
2. Perluas section **Blok B - Cycle Time Test**.
3. Isi field:
   - **N** — jumlah cycle yang ingin diukur (mis. `100`).
   - **Trigger** — nama signal trigger (sesuai entry di Address Mapping).
   - **Done** — nama signal "done".
   - **Error** — nama signal "error".
4. Klik **Start Block B**. Label `progress` menjadi `RUNNING`, sistem menunggu rising edge pada signal trigger.
5. Trigger satu cycle:
   - **PLC fisik:** PLC men-set coil trigger → `True`. Aplikasi mendeteksi rising edge, menjalankan program robot. Saat program selesai, aplikasi men-set coil done (atau error jika gagal).
   - **Manual/emulator:** toggle coil trigger di tool eksternal (mis. modpoll, qmodbus) atau langsung edit di tab Modbus Address Mapping bila bisa write coil.
6. Pantau di UI:
   - Label `active` menjadi `True` selama cycle berjalan.
   - Counter `success_count` / `failed_count` bertambah saat cycle selesai.
   - Statistik live: `avg_s`, `min_s`, `max_s`, `std_s` (dalam detik).
   - Tabel di bawah menambahkan baris baru per cycle: `Idx`, `Time`, `Status`, `Duration (s)`, `Note`.
7. Lanjutkan trigger sampai counter `completed` mencapai N (otomatis stop), atau klik **Stop** untuk berhenti awal.
8. Klik **Export XLSX**. File berisi:
   - Sheet **Block B Samples** — tiap cycle (Index, Timestamp, Status, Duration (s), Note).
   - Sheet **Summary** — metadata + stats agregat.

### Interpretasi Hasil

- **avg_s konsisten, std_s kecil** → siklus deterministik, baik.
- **std_s besar / max_s ≫ avg_s** → ada outlier. Cek apakah salah satu cycle terganggu (program error, robot wait input, jaringan jeda).
- **failed_count > 0** → kolom `Note` di tabel berisi state program saat dianggap gagal (mis. `ERROR`, `STOP_REQUESTED`). Inspeksi log untuk root cause.
- **Tidak ada cycle terdeteksi** padahal PLC ngirim trigger:
  - Cek `Trigger` name persis cocok dengan entry di Address Mapping.
  - Pastikan signal trigger berupa **rising edge** (dari `False` → `True`); kalau PLC men-hold di `True`, tidak akan trigger ulang.
  - Pastikan tombol Estop (`Buttons[7]`) tidak aktif.

### Tip Reproducibility

Catat: identitas program robot yang dieksekusi, payload trigger (coil/register), kondisi awal robot, dan parameter motion. Untuk perbandingan antar konfigurasi (mis. variasi kecepatan robot), jalankan minimal 30 cycle agar `std_s` representatif.

---

## Troubleshooting Umum

| Gejala | Cek |
|---|---|
| Tombol Start tidak merespon | Field input kosong/ill-formed. Lihat console output. |
| "Belum ada data" saat klik Export | Test belum pernah dijalankan di sesi ini, atau test gagal sebelum sample pertama tercatat. |
| File `.xlsx` tidak terbuat | `openpyxl` tidak terpasang. Jalankan `pip install openpyxl>=3.1.2`. |
| Koneksi gagal mode non-MOCK | Cek IP/port/slave_id; coba ping PLC; verifikasi firewall di OS host. |
| `packet_loss_pct = 100%` segera | PLC unreachable atau `pymodbus` versi tidak kompatibel. Cek `requirements.txt`. |
| Block B tidak detect trigger | Mapping signal salah, atau trigger tidak rising edge. |

---

## Catatan Implementasi

- Logging table di UI dibatasi tampilan oleh tinggi widget (~6 baris terlihat, sisanya bisa di-scroll).
- Sample lengkap (full history) disimpan di memori `ModbusManager` dan **direset setiap kali Start dipencet ulang**.
- Untuk arsip jangka panjang, **selalu Export XLSX sebelum menjalankan test berikutnya**.
- Field `Note` Blok B berisi state program robot (`IDLE`, `ERROR`, dll.) — semantic note ini ditetapkan oleh handler `block_b_cycle_finished` di `Commander_feature_adapters.py`.
