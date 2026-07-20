# Panduan Pengujian Modbus PAROL6 App

Dokumen ini menjelaskan langkah pengujian Modbus pada PAROL6 App. Pengujian
dilakukan sebagai **satu alur terpadu berbasis siklus** (per-siklus): setiap
trigger dari PLC memicu satu siklus kerja robot, dan aplikasi mencatat data
komunikasi Modbus sekaligus data cycle time dalam satu tabel.

## 1. Ringkasan Pengujian

Pada setiap siklus (satu trigger PLC), aplikasi mencatat satu baris data:

| Kolom | Arti |
| --- | --- |
| `No. Siklus` | Nomor urut siklus |
| `Timestamp` | Waktu siklus dimulai |
| `Response Time (ms)` | Waktu respons komunikasi Modbus saat trigger |
| `Packet Status` | `OK`, `Timeout`, atau `Loss` |
| `Cycle Time (s)` | Durasi end-to-end satu siklus robot |
| `Outcome` | `Sukses` atau `Gagal` |

Ringkasan (dihitung otomatis, tanpa min/max/outlier):

- `avg_rt_ms`: rata-rata response time komunikasi.
- `throughput_rps`: jumlah siklus per detik.
- `packet_loss_pct`: persentase paket gagal/timeout.
- `avg_cycle_time_s`: rata-rata cycle time.
- `jumlah_data`: jumlah siklus tercatat.

Catatan:

- Mode `MOCK` dipakai untuk dry-run tanpa jaringan PLC.
- Mode IP PLC nyata dipakai untuk uji komunikasi sebenarnya.
- Jika robot tidak connect, program yang hanya berisi `ModbusRead`,
  `ModbusWrite`, `Delay`, `timestamp`, `print`, atau `vision` masih bisa
  berjalan offline. Program yang berisi `MoveJoint`, `MoveCart`, `Gripper`,
  `Output`, atau command fisik lain akan gagal karena membutuhkan robot.

## 2. Persiapan Umum

### 2.1 Siapkan aplikasi

Jalankan aplikasi dari root project:

```bash
cd /home/arya/TA/parol6_TA
./run_linux.sh
```

Alternatif manual:

```bash
cd /home/arya/TA/parol6_TA/GUI/files
../../.venv/bin/python Serial_sender_good_latest.py
```

Menjalankan dari `GUI/files` penting jika memakai mode manual, karena beberapa
asset dan program runtime memakai path relatif.

### 2.2 Siapkan robot PAROL6

Langkah ini wajib jika program melakukan gerakan fisik.

1. Hubungkan power robot dan board kontrol.
2. Hubungkan USB serial robot ke komputer.
3. Cek port serial:

   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB*
   ```

4. Jika robot muncul sebagai `/dev/ttyACM0`, masukkan `0` pada field port di
   app. Jika muncul sebagai device lain, masukkan path penuh, misalnya
   `/dev/ttyUSB0`.
5. Pastikan status koneksi di app berubah menjadi `CONNECTED`.
6. Jalankan program pick-place secara manual satu kali tanpa Modbus untuk
   memastikan robot, homing, workspace, gripper, dan program sudah aman.

### 2.3 Siapkan PLC atau emulator Modbus

Gunakan salah satu:

- PLC fisik sebagai Modbus TCP server.
- Emulator Modbus TCP.
- Mode internal `MOCK` di aplikasi untuk dry-run.

Untuk PLC fisik:

1. Pastikan komputer dan PLC berada pada jaringan yang sama.
2. Catat IP PLC.
3. Pastikan port Modbus TCP terbuka, biasanya `502`.
4. Pastikan `slave_id` sesuai konfigurasi PLC.
5. Pastikan firewall komputer tidak memblokir koneksi.

### 2.4 Buka tab Modbus

1. Di app, klik tab **Modbus**.
2. Pada section **Modbus Connection Setup**, isi:
   - `IP`: isi `MOCK` untuk simulasi, atau IP PLC nyata.
   - `Port`: biasanya `502`.
   - `Slave ID`: biasanya `1`, sesuaikan PLC.
3. Pada **Address Mapping**, pastikan minimal ada signal berikut:

| Name | Type | Address | RW | Fungsi |
| --- | --- | ---: | --- | --- |
| `trigger_pick` | `coil` | `0` | `read` | Trigger dari PLC ke app |
| `cycle_done` | `coil` | `1` | `write` | Sinyal sukses dari app ke PLC |
| `error_flag` | `coil` | `2` | `write` | Sinyal error dari app ke PLC |

4. Klik **Save Config** untuk menyimpan konfigurasi.

Catatan penyimpanan:

- Perubahan IP, port, slave ID, dan address mapping tidak langsung tersimpan
  saat diketik.
- Perubahan tersimpan saat klik **Save Config**, **Connect**, atau **Start**.
- Jika IP diganti saat Modbus masih connected, lakukan **Disconnect** lalu
  **Connect** lagi supaya client memakai IP baru.

## 3. Pengujian Per Siklus

### 3.1 Tujuan

Mengukur, per siklus, durasi end-to-end satu siklus robot sekaligus performa
komunikasi Modbus yang memicunya:

1. PLC atau emulator mengirim trigger Modbus.
2. App mendeteksi rising edge `trigger_pick`.
3. App menjalankan program robot yang sedang dimuat.
4. Setelah program selesai, app menulis:
   - `cycle_done = True` jika sukses.
   - `error_flag = True` jika gagal.
5. App menyimpan satu baris data (response time, packet status, cycle time,
   outcome).

### 3.2 Prasyarat

Sebelum menjalankan pengujian, pastikan:

1. Robot sudah connect ke app jika program berisi gerakan fisik.
2. Program pick-place sudah diuji manual lewat tombol execute biasa.
3. Program tidak menabrak limit, workspace, objek, atau fixture.
4. Modbus mapping memiliki `trigger_pick`, `cycle_done`, dan `error_flag`.
5. PLC atau emulator dapat mengubah nilai trigger dari `False` ke `True`.
6. Trigger dapat dikembalikan ke `False` setelah satu siklus agar rising edge
   berikutnya bisa terdeteksi.

### 3.3 Contoh struktur program robot

Gunakan program pick-place yang sudah divalidasi di robot. Struktur minimalnya:

```text
Begin()
; isi gerakan pick-place yang sudah aman
End()
```

Trigger utama sudah ditangani oleh handler pengujian. Skenario yang paling rapi:

- PLC menulis `trigger_pick`.
- App mendeteksi trigger.
- App menjalankan program pick-place.
- App menulis `cycle_done` atau `error_flag` setelah program selesai.

Hindari menggandakan mekanisme trigger di dalam script (`ModbusRead`) kecuali
memang sedang menguji script `ModbusRead` secara khusus.

### 3.4 Konfigurasi

Pada section **Pengujian Modbus (Per Siklus)**, isi:

| Field | Nilai awal yang disarankan | Keterangan |
| --- | --- | --- |
| `N` | `100` | Jumlah siklus |
| `Trigger` | `trigger_pick` | Nama signal trigger |
| `Done` | `cycle_done` | Nama signal selesai |
| `Error` | `error_flag` | Nama signal error |

Pastikan nama tersebut sama persis dengan kolom `Name` di Address Mapping.

### 3.5 Langkah menjalankan

1. Jalankan app.
2. Connect robot sampai status app menunjukkan `CONNECTED` (jika program berisi
   gerakan fisik).
3. Buka atau tulis program pick-place di editor program.
4. Jalankan program sekali secara manual tanpa Modbus.
5. Jika program manual sukses, kembali ke tab **Modbus**.
6. Isi **Modbus Connection Setup**:
   - `IP = MOCK` untuk dry-run.
   - `IP = <alamat PLC>` untuk uji nyata.
   - `Port = 502`.
   - `Slave ID` sesuai PLC.
7. Klik **Connect**.
8. Pastikan status Modbus menunjukkan `MOCK` atau `<ip>:<port>`.
9. Isi konfigurasi pengujian:
   - `N = 100` atau sesuai rancangan pengujian.
   - `Trigger = trigger_pick`.
   - `Done = cycle_done`.
   - `Error = error_flag`.
10. Klik **Start**.
11. Pastikan label `progress` berubah menjadi `RUNNING`.
12. Dari PLC atau emulator, set `trigger_pick` dari `False` ke `True`.
13. App akan menjalankan program robot.
14. Setelah program mulai, kembalikan `trigger_pick` ke `False` agar trigger
    berikutnya bisa terbaca sebagai rising edge baru.
15. Tunggu robot menyelesaikan pick-place.
16. Cek hasil:
    - Jika sukses, `cycle_done` ditulis `True`.
    - Jika gagal, `error_flag` ditulis `True`.
    - Tabel data menampilkan status dan cycle time siklus tersebut.
17. Ulangi trigger sampai `completed = N`.
18. Setelah selesai, klik **Export Data**.
19. Simpan file hasil dengan nama yang jelas, misalnya:

    ```text
    modbus_pick_place_n100_YYYYMMDD.xlsx
    ```

File XLSX berisi dua sheet:

- **Data**: tabel mentah per-siklus (No. Siklus, Timestamp, Response Time,
  Packet Status, Cycle Time, Outcome).
- **Ringkasan**: `avg_rt_ms`, `throughput_rps`, `packet_loss_pct`,
  `avg_cycle_time_s`, `jumlah_data`.

Tombol **Clear Data** mengosongkan tabel untuk memulai sesi pengujian baru.

### 3.6 Validasi hasil

Hasil dianggap valid jika:

1. Jumlah sample sama dengan jumlah siklus yang direncanakan.
2. `success_count + failed_count = completed`.
3. `failed_count` rendah atau sesuai toleransi penelitian.
4. `avg_cycle_time_s` masuk akal terhadap observasi fisik.
5. Setiap trigger menghasilkan tepat satu siklus, bukan nol siklus atau lebih
   dari satu siklus.
6. `cycle_done` dan `error_flag` terbaca benar oleh PLC.
7. `packet_loss_pct` rendah atau 0% untuk jaringan stabil, dan `avg_rt_ms`
   masih sesuai batas penelitian.

Jika trigger tidak terdeteksi:

- Pastikan signal yang diubah adalah `trigger_pick`.
- Pastikan type address adalah `coil`.
- Pastikan address sesuai mapping PLC.
- Pastikan nilai berubah dari `False` ke `True`, bukan tetap `True`.
- Pastikan pengujian sudah dalam kondisi `RUNNING`.
- Pastikan Modbus polling sedang connected.

Jika `packet_loss_pct = 100%`, cek:

- IP PLC salah.
- Port `502` tertutup.
- Slave ID salah.
- Firewall memblokir koneksi.
- PLC belum aktif sebagai Modbus TCP server.

## 4. Urutan Pengujian yang Disarankan

### 4.1 Tahap 1 - Dry-run Modbus internal

Tujuan: memastikan UI, konfigurasi, logging, dan export berjalan.

1. Set `IP = MOCK`.
2. Klik **Save Config**, lalu **Connect**.
3. Jalankan pengujian dengan program non-fisik atau program sederhana yang aman.
4. Export Data.

Catatan: Jika robot tidak connect dan program berisi command gerak, siklus akan
gagal. Untuk dry-run tanpa robot, gunakan program software-only.

### 4.2 Tahap 2 - Uji komunikasi PLC tanpa robot

Tujuan: memastikan koneksi TCP/IP PLC stabil.

1. Set `IP = <alamat PLC>`, set `Port` dan `Slave ID`.
2. Klik **Disconnect** jika sebelumnya sudah connected, lalu **Connect**.
3. Amati status Modbus dan monitor snapshot address mapping.

Jangan lanjut ke pengujian penuh jika koneksi masih gagal total.

### 4.3 Tahap 3 - Validasi program robot manual

Tujuan: memastikan kegagalan pengujian bukan berasal dari program robot.

1. Connect robot.
2. Home atau siapkan posisi awal sesuai prosedur lab.
3. Jalankan program pick-place tanpa trigger PLC.
4. Catat apakah program selesai normal.
5. Perbaiki program sampai stabil sebelum lanjut.

### 4.4 Tahap 4 - Uji per siklus dengan PLC atau emulator

Tujuan: mengukur cycle time dan performa komunikasi end-to-end.

1. Pastikan koneksi Modbus stabil dan robot connect.
2. Pastikan program pick-place sudah stabil.
3. Klik **Start**.
4. Trigger `trigger_pick` sebanyak `N` siklus.
5. Export Data.
6. Simpan catatan kondisi eksperimen.

## 5. Format Data yang Perlu Dicatat

Selain export XLSX dari app, catat metadata eksperimen berikut:

| Item | Contoh |
| --- | --- |
| Tanggal pengujian | `2026-06-26` |
| Operator | Nama penguji |
| Mode | `MOCK` atau `PLC nyata` |
| IP PLC | `192.168.1.10` |
| Port | `502` |
| Slave ID | `1` |
| N siklus | `100` |
| Program robot | Nama file program |
| Kondisi objek | Jenis dan posisi objek |
| Catatan error | Timeout, failed trigger, gagal grip, dll. |

## 6. Kriteria Keberhasilan

Pengujian berhasil jika:

- App mendeteksi trigger Modbus.
- Robot menjalankan satu program per satu trigger.
- App menulis `cycle_done` untuk siklus sukses.
- App menulis `error_flag` untuk siklus gagal.
- `packet_loss_pct` dan response time berada dalam batas penelitian.
- Data (tabel per-siklus dan ringkasan) dapat diexport.

## 7. Troubleshooting

| Gejala | Penyebab umum | Tindakan |
| --- | --- | --- |
| Status Modbus tetap disconnected | IP/port salah, PLC mati, firewall | Cek IP, port 502, firewall, koneksi kabel |
| `packet_loss_pct` tinggi | Banyak request gagal/timeout | Cek slave ID, koneksi, dan PLC |
| Pengujian tidak mulai siklus | Tidak ada rising edge trigger | Set trigger `False` dulu, lalu `True` |
| Trigger hanya jalan sekali | Trigger tidak dikembalikan ke `False` | Reset trigger setelah program mulai |
| Program gagal saat robot tidak connect | Program berisi command fisik | Connect robot atau pakai program software-only |
| IP sudah diganti tapi masih ke target lama | Client Modbus lama masih aktif | Klik **Disconnect**, lalu **Connect** lagi |
| Address mapping berubah tapi tidak tersimpan | Belum klik save/start | Klik **Save Config** atau **Start** |
| Export kosong/hilang | Test belum jalan atau app ditutup | Export segera setelah test selesai |

## 8. Checklist Sebelum Mengambil Data Final

- [ ] App berjalan tanpa error.
- [ ] Robot connect untuk pengujian fisik.
- [ ] PLC/emulator Modbus siap.
- [ ] IP, port, slave ID sudah benar.
- [ ] Address mapping sudah benar.
- [ ] Konfigurasi sudah disimpan.
- [ ] Jika IP baru dipakai, Modbus sudah reconnect.
- [ ] Program pick-place sudah diuji manual.
- [ ] Koneksi Modbus sudah stabil.
- [ ] Jumlah siklus (`N`) sudah diset benar.
- [ ] File XLSX diexport setelah test selesai.
- [ ] Metadata eksperimen dicatat.

## 9. Catatan untuk Flowchart Penelitian

Struktur alur pengujian yang disarankan:

1. Konfigurasi sistem (IP, port, slave ID, address mapping).
2. Connect Modbus dan robot.
3. Klik **Start**.
4. Trigger `trigger_pick` sebanyak `N` siklus; tiap siklus mencatat response
   time, packet status, cycle time, dan outcome.
5. Export Data (tabel per-siklus + ringkasan).
6. Analisis hasil:
   - rata-rata response time,
   - throughput,
   - packet loss,
   - rata-rata cycle time,
   - success/fail per siklus.
7. Pembahasan dan kesimpulan.

Dengan satu alur ini, hasil pengujian lebih sesuai dengan implementasi app dan
lebih mudah dipertanggungjawabkan dalam laporan.
