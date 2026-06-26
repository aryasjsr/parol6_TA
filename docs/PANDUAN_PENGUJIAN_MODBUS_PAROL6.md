# Panduan Pengujian Modbus PAROL6 App

Dokumen ini menjelaskan langkah pengujian Modbus pada PAROL6 App secara
terpisah untuk:

- Blok A: Protocol Performance Test.
- Blok B: Cycle Time Test.

Keduanya sengaja dipisahkan karena fungsi yang diuji berbeda. Blok A mengukur
performa komunikasi Modbus TCP/IP. Blok B mengukur durasi siklus kerja robot
yang dipicu oleh sinyal Modbus.

## 1. Ringkasan Pengujian

| Blok | Fokus uji | Robot wajib connect? | PLC wajib ada? | Output utama |
| --- | --- | --- | --- | --- |
| Blok A | Response time, throughput, packet loss | Tidak | Tidak jika `MOCK`; ya jika IP PLC nyata | Statistik protokol |
| Blok B | Trigger Modbus ke siklus pick-place | Ya, jika program berisi gerakan robot | Tidak jika `MOCK`; ya untuk uji integrasi PLC nyata | Cycle time, success/fail |

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

Langkah ini wajib untuk Blok B jika program melakukan gerakan fisik.

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
- Perubahan tersimpan saat klik **Save Config**, **Connect**, atau
  **Start Block B**.
- Jika IP diganti saat Modbus masih connected, lakukan **Disconnect** lalu
  **Connect** lagi supaya client memakai IP baru.

## 3. Pengujian Blok A - Protocol Performance Test

### 3.1 Tujuan

Blok A digunakan untuk mengukur performa komunikasi Modbus TCP/IP tanpa
menjalankan robot. Data utama:

- `avg_rt_ms`: rata-rata response time.
- `throughput_rps`: jumlah request per detik.
- `packet_loss_pct`: persentase request gagal atau timeout.
- `success_count`: jumlah request sukses.
- `failed_count`: jumlah request gagal.
- `timeout_count`: jumlah request timeout.

### 3.2 Kapan Blok A dilakukan

Lakukan Blok A sebelum Blok B untuk memastikan jalur komunikasi stabil.
Urutan yang disarankan:

1. Dry-run dengan `IP = MOCK`.
2. Uji PLC nyata dengan `IP = <alamat PLC>`.
3. Ulangi jika ada perubahan jaringan, timeout, slave ID, atau mapping
   address.

### 3.3 Konfigurasi Blok A

Pada section **Blok A - Protocol Performance Test**, isi:

| Field | Nilai awal yang disarankan | Keterangan |
| --- | --- | --- |
| `IP` | `MOCK` atau IP PLC | Target koneksi Modbus untuk Blok A |
| `Port` | `502` | Port Modbus TCP |
| `Slave` | `1` | Slave/unit ID |
| `Timeout ms` | `1000` | Batas tunggu response |
| `Function` | `read_coil` atau sesuai PLC | Function code yang diuji |
| `Address` | address yang valid | Contoh `0` untuk coil trigger |
| `Count` | `1` | Jumlah register/coil per request |
| `N` | `1000` | Jumlah request untuk uji protokol |

Jika memakai mapping default, untuk uji sederhana dapat memakai:

- `Function = read_coil`
- `Address = 0`
- `Count = 1`
- `N = 1000`

Jika PLC hanya menyediakan holding register, gunakan:

- `Function = read_holding_register`
- `Address = address register yang valid`
- `Count = 1`

### 3.4 Langkah menjalankan Blok A

1. Pastikan app sudah terbuka dan tab **Modbus** aktif.
2. Isi konfigurasi Blok A sesuai target uji.
3. Klik **Start Block A**.
4. Amati label progress sampai selesai:
   - `progress`
   - `avg_rt_ms`
   - `throughput_rps`
   - `packet_loss_pct`
   - `success_count`
   - `failed_count`
   - `timeout_count`
5. Jika ingin menghentikan sebelum selesai, klik **Stop**.
6. Setelah selesai, klik **Export XLSX**.
7. Simpan file hasil dengan nama yang jelas, misalnya:

   ```text
   block_a_plc_read_coil_addr0_n1000_YYYYMMDD.xlsx
   ```

### 3.5 Validasi hasil Blok A

Hasil Blok A dianggap layak untuk lanjut ke Blok B jika:

1. `packet_loss_pct` rendah atau 0% untuk jaringan stabil.
2. `timeout_count` tidak muncul terus-menerus.
3. `avg_rt_ms` masih sesuai batas penelitian. Jika memakai target 100 ms,
   bandingkan `avg_rt_ms` terhadap 100 ms.
4. Tidak ada error koneksi seperti `Unable to connect`.

Jika `packet_loss_pct = 100%`, cek:

- IP PLC salah.
- Port `502` tertutup.
- Slave ID salah.
- Address/function tidak valid untuk PLC.
- Firewall memblokir koneksi.
- PLC belum aktif sebagai Modbus TCP server.

## 4. Pengujian Blok B - Cycle Time Test

### 4.1 Tujuan

Blok B digunakan untuk mengukur durasi end-to-end satu siklus robot:

1. PLC atau emulator mengirim trigger Modbus.
2. App mendeteksi rising edge `trigger_pick`.
3. App menjalankan program robot yang sedang dimuat.
4. Setelah program selesai, app menulis:
   - `cycle_done = True` jika sukses.
   - `error_flag = True` jika gagal.
5. App menyimpan data cycle time dan status siklus.

### 4.2 Prasyarat Blok B

Sebelum menjalankan Blok B, pastikan:

1. Robot sudah connect ke app jika program berisi gerakan fisik.
2. Program pick-place sudah diuji manual lewat tombol execute biasa.
3. Program tidak menabrak limit, workspace, objek, atau fixture.
4. Modbus mapping memiliki `trigger_pick`, `cycle_done`, dan `error_flag`.
5. PLC atau emulator dapat mengubah nilai trigger dari `False` ke `True`.
6. Trigger dapat dikembalikan ke `False` setelah satu siklus agar rising edge
   berikutnya bisa terdeteksi.

### 4.3 Contoh struktur program robot

Gunakan program pick-place yang sudah divalidasi di robot. Struktur minimalnya:

```text
Begin()
; isi gerakan pick-place yang sudah aman
End()
```

Jika ingin program menunggu trigger di dalam script, dapat memakai:

```text
Begin()
ModbusRead(trigger_pick, HIGH, timeout=5)
; isi gerakan pick-place yang sudah aman
ModbusWrite(cycle_done, HIGH)
End()
```

Namun untuk Blok B, trigger utama sudah ditangani oleh handler Blok B. Karena
itu, skenario yang paling rapi adalah:

- PLC menulis `trigger_pick`.
- Blok B mendeteksi trigger.
- App menjalankan program pick-place.
- App menulis `cycle_done` atau `error_flag` setelah program selesai.

Hindari menggandakan mekanisme trigger kecuali memang sedang menguji script
ModbusRead secara khusus.

### 4.4 Konfigurasi Blok B

Pada section **Blok B - Cycle Time Test**, isi:

| Field | Nilai awal yang disarankan | Keterangan |
| --- | --- | --- |
| `N` | `100` | Jumlah siklus |
| `Trigger` | `trigger_pick` | Nama signal trigger |
| `Done` | `cycle_done` | Nama signal selesai |
| `Error` | `error_flag` | Nama signal error |

Pastikan nama tersebut sama persis dengan kolom `Name` di Address Mapping.

### 4.5 Langkah menjalankan Blok B

1. Jalankan app.
2. Connect robot sampai status app menunjukkan `CONNECTED`.
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
9. Isi konfigurasi Blok B:
   - `N = 100` atau sesuai rancangan pengujian.
   - `Trigger = trigger_pick`.
   - `Done = cycle_done`.
   - `Error = error_flag`.
10. Klik **Start Block B**.
11. Pastikan label `progress` berubah menjadi `RUNNING`.
12. Dari PLC atau emulator, set `trigger_pick` dari `False` ke `True`.
13. App akan menjalankan program robot.
14. Setelah program mulai, kembalikan `trigger_pick` ke `False` agar trigger
    berikutnya bisa terbaca sebagai rising edge baru.
15. Tunggu robot menyelesaikan pick-place.
16. Cek hasil:
    - Jika sukses, `cycle_done` ditulis `True`.
    - Jika gagal, `error_flag` ditulis `True`.
    - Log table Blok B menampilkan status dan `duration_s`.
17. Ulangi trigger sampai `completed = N`.
18. Setelah selesai, klik **Export XLSX**.
19. Simpan file hasil dengan nama yang jelas, misalnya:

    ```text
    block_b_pick_place_n100_YYYYMMDD.xlsx
    ```

### 4.6 Validasi hasil Blok B

Hasil Blok B dianggap valid jika:

1. Jumlah sample sama dengan jumlah siklus yang direncanakan.
2. `success_count + failed_count = completed`.
3. `failed_count` rendah atau sesuai toleransi penelitian.
4. `avg_s`, `min_s`, `max_s`, dan `std_s` masuk akal terhadap observasi fisik.
5. Setiap trigger menghasilkan tepat satu siklus, bukan nol siklus atau lebih
   dari satu siklus.
6. `cycle_done` dan `error_flag` terbaca benar oleh PLC.

Jika trigger tidak terdeteksi:

- Pastikan signal yang diubah adalah `trigger_pick`.
- Pastikan type address adalah `coil`.
- Pastikan address sesuai mapping PLC.
- Pastikan nilai berubah dari `False` ke `True`, bukan tetap `True`.
- Pastikan Blok B sudah dalam kondisi `RUNNING`.
- Pastikan Modbus polling sedang connected.

## 5. Urutan Pengujian yang Disarankan

Gunakan urutan berikut agar masalah lebih mudah dilacak.

### 5.1 Tahap 1 - Dry-run Modbus internal

Tujuan: memastikan UI, konfigurasi, logging, dan export berjalan.

1. Set `IP = MOCK`.
2. Klik **Save Config**.
3. Jalankan Blok A dengan `N = 100`.
4. Export XLSX.
5. Jalankan Blok B dengan program non-fisik atau program sederhana yang aman.
6. Export XLSX.

Catatan: Jika robot tidak connect dan program berisi command gerak, Blok B akan
gagal. Untuk dry-run tanpa robot, gunakan program software-only.

### 5.2 Tahap 2 - Uji komunikasi PLC tanpa robot

Tujuan: memastikan koneksi TCP/IP PLC stabil.

1. Set `IP = <alamat PLC>`.
2. Set `Port` dan `Slave ID`.
3. Klik **Disconnect** jika sebelumnya sudah connected.
4. Klik **Connect**.
5. Jalankan Blok A dengan address/function yang valid di PLC.
6. Export XLSX.
7. Cek `packet_loss_pct`, `timeout_count`, dan `avg_rt_ms`.

Jangan lanjut ke Blok B jika Blok A masih gagal total.

### 5.3 Tahap 3 - Validasi program robot manual

Tujuan: memastikan kegagalan Blok B bukan berasal dari program robot.

1. Connect robot.
2. Home atau siapkan posisi awal sesuai prosedur lab.
3. Jalankan program pick-place tanpa trigger PLC.
4. Catat apakah program selesai normal.
5. Perbaiki program sampai stabil sebelum lanjut.

### 5.4 Tahap 4 - Uji Blok B dengan PLC atau emulator

Tujuan: mengukur cycle time end-to-end.

1. Pastikan Blok A sudah stabil.
2. Pastikan robot connect.
3. Pastikan program pick-place sudah stabil.
4. Klik **Start Block B**.
5. Trigger `trigger_pick` sebanyak `N` siklus.
6. Export XLSX.
7. Simpan catatan kondisi eksperimen.

### 5.5 Tahap 5 - Analisis gabungan

Gabungkan hasil Blok A dan Blok B di tahap analisis, bukan di loop pengujian.

Gunakan Blok A untuk:

- Rata-rata response time.
- Throughput.
- Packet loss.
- Timeout.

Gunakan Blok B untuk:

- Rata-rata cycle time.
- Minimum cycle time.
- Maximum cycle time.
- Standar deviasi cycle time.
- Success/fail per siklus.

Jika ingin menguji korelasi, sejajarkan data berdasarkan waktu pengambilan,
kondisi jaringan, dan skenario trigger.

## 6. Format Data yang Perlu Dicatat

Selain export XLSX dari app, catat metadata eksperimen berikut:

| Item | Contoh |
| --- | --- |
| Tanggal pengujian | `2026-06-26` |
| Operator | Nama penguji |
| Mode | `MOCK` atau `PLC nyata` |
| IP PLC | `192.168.1.10` |
| Port | `502` |
| Slave ID | `1` |
| Function Blok A | `read_coil` |
| Address Blok A | `0` |
| N Blok A | `1000` |
| N Blok B | `100` |
| Program robot | Nama file program |
| Kondisi objek | Jenis dan posisi objek |
| Catatan error | Timeout, failed trigger, gagal grip, dll. |

## 7. Kriteria Keberhasilan

### 7.1 Blok A

Pengujian Blok A berhasil jika:

- App dapat menyelesaikan N request.
- Data sample dan summary dapat diexport.
- `packet_loss_pct` dan `timeout_count` berada dalam batas penelitian.
- Tidak ada error koneksi berulang.

### 7.2 Blok B

Pengujian Blok B berhasil jika:

- App mendeteksi trigger Modbus.
- Robot menjalankan satu program per satu trigger.
- App menulis `cycle_done` untuk siklus sukses.
- App menulis `error_flag` untuk siklus gagal.
- Data cycle time dapat diexport.

## 8. Troubleshooting

| Gejala | Penyebab umum | Tindakan |
| --- | --- | --- |
| Status Modbus tetap disconnected | IP/port salah, PLC mati, firewall | Cek IP, port 502, firewall, koneksi kabel |
| `packet_loss_pct = 100%` | Semua request gagal/timeout | Cek function, address, slave ID, dan PLC |
| Blok B tidak mulai siklus | Tidak ada rising edge trigger | Set trigger `False` dulu, lalu `True` |
| Trigger hanya jalan sekali | Trigger tidak dikembalikan ke `False` | Reset trigger setelah program mulai |
| Program gagal saat robot tidak connect | Program berisi command fisik | Connect robot atau pakai program software-only |
| IP sudah diganti tapi masih ke target lama | Client Modbus lama masih aktif | Klik **Disconnect**, lalu **Connect** lagi |
| Address mapping berubah tapi tidak tersimpan | Belum klik save/start | Klik **Save Config** atau **Connect** |
| Export kosong/hilang | Test belum jalan atau app ditutup | Export segera setelah test selesai |

## 9. Checklist Sebelum Mengambil Data Final

Gunakan checklist ini sebelum data final dicatat.

- [ ] App berjalan tanpa error.
- [ ] Robot connect untuk Blok B fisik.
- [ ] PLC/emulator Modbus siap.
- [ ] IP, port, slave ID sudah benar.
- [ ] Address mapping sudah benar.
- [ ] Konfigurasi sudah disimpan.
- [ ] Jika IP baru dipakai, Modbus sudah reconnect.
- [ ] Program pick-place sudah diuji manual.
- [ ] Blok A sudah diuji dan hasil komunikasi stabil.
- [ ] Blok B sudah diset ke jumlah siklus yang benar.
- [ ] File XLSX diexport setelah test selesai.
- [ ] Metadata eksperimen dicatat.

## 10. Catatan untuk Flowchart Penelitian

Untuk flowchart metodologi, jangan mencampur Blok A dan Blok B dalam satu loop
yang sama.

Struktur yang disarankan:

1. Konfigurasi sistem.
2. Jalankan Blok A untuk uji protokol.
3. Analisis awal hasil Blok A.
4. Jalankan Blok B untuk uji cycle time.
5. Export dan rekap hasil Blok B.
6. Analisis gabungan:
   - response time,
   - throughput,
   - packet loss,
   - cycle time,
   - success/fail,
   - korelasi jika diperlukan.
7. Pembahasan dan kesimpulan.

Dengan struktur ini, hasil pengujian lebih sesuai dengan implementasi app dan
lebih mudah dipertanggungjawabkan dalam laporan.
