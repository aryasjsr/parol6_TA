# PAROL6 TA Commander

Fork dari [PAROL Commander Software](https://github.com/PCrnjak/PAROL-commander-software) untuk pengembangan dan pengujian sistem robot PAROL6 pada tugas akhir. Aplikasi desktop ini menggabungkan kontrol robot, simulator, vision, komunikasi Modbus TCP, dan pencatatan data penelitian dalam satu antarmuka.

![Tampilan PAROL6 Commander](Images/screen_2.png)

## Fitur utama

- Jogging enam joint dan gerakan Cartesian pada world/tool reference frame.
- Telemetri posisi, status koneksi, I/O, error, dan kontrol gripper.
- Editor program robot dengan perintah gerak, I/O, delay, loop, Modbus, dan vision.
- Simulator PAROL6 bawaan.
- Deteksi objek dari kamera atau sumber `MOCK` untuk dry-run.
- Kalibrasi intrinsik kamera dan transformasi camera-to-base.
- Validasi workspace, safe-pick, perhitungan Tool-Z, dan placeholder `$vision.*`.
- Modbus TCP untuk integrasi PLC, termasuk mode `MOCK` tanpa PLC.
- Logging metrik penelitian dan ekspor hasil pengujian.
- Tema antarmuka `Dark`, `Light`, dan `Gray`; pilihan terakhir disimpan otomatis.

## Persyaratan

- Python **3.10.x**. Beberapa dependensi yang dipin belum kompatibel dengan Python 3.12.
- Linux atau Windows dengan dukungan GUI/Tk.
- Board kontrol PAROL6 untuk menjalankan robot fisik.
- Kamera dan PLC hanya diperlukan untuk pengujian nyata; mode `MOCK` tersedia untuk pengujian perangkat lunak.

Model ONNX tidak disimpan di Git karena ukurannya besar. Untuk deteksi berbasis model, letakkan model pada path yang dipilih di GUI (default: `vision/models/best.onnx`).

> [!CAUTION]
> Uji program terlebih dahulu dengan simulator atau mode `MOCK`. Sebelum mengaktifkan robot fisik, pastikan area kerja kosong, batas joint benar, tombol emergency stop dapat dijangkau, dan kecepatan awal dibuat rendah.

## Instalasi cepat di Linux

Contoh berikut ditujukan untuk Ubuntu dan turunannya.

```bash
sudo apt update
sudo apt install -y git build-essential python3-tk python3-pil python3-pil.imagetk

git clone https://github.com/aryasjsr/parol6_TA.git
cd parol6_TA
git checkout comsoft

python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools
python -m pip install wheel==0.42.0
python -m pip install -r requirements.txt
```

Jalankan aplikasi dari root repository:

```bash
chmod +x run_linux.sh
./run_linux.sh
```

`run_linux.sh` selalu memakai `.venv/bin/python` dan menjalankan entry point dari direktori yang benar agar asset GUI serta import lokal dapat ditemukan.

Panduan Linux yang lebih lengkap, termasuk instalasi Python melalui `pyenv`, tersedia di [Linux_install.md](Linux_install.md). Untuk Windows, lihat [Windows_install.md](Windows_install.md).

## Menjalankan secara manual

Jika launcher tidak digunakan:

```bash
cd GUI/files
../../.venv/bin/python Serial_sender_good_latest.py
```

Jangan menjalankan entry point dari root repository karena beberapa asset lama masih memakai path relatif terhadap `GUI/files`.

## Konfigurasi awal

Saat pertama dijalankan, aplikasi membuat `config.json` di root repository. File ini bersifat lokal dan tidak dilacak Git. Pengaturan utama dapat diubah dari GUI:

1. **Robot/Serial** — pilih port board, misalnya `COM3` di Windows atau `/dev/ttyACM0` di Linux.
2. **Modbus** — gunakan IP PLC dan port `502`, atau isi IP dengan `MOCK` untuk dry-run.
3. **Vision** — pilih `MOCK`, indeks kamera, atau sumber video; kemudian atur metode deteksi dan kalibrasi.
4. **Workspace** — periksa batas X/Y/Z dan margin sebelum menjalankan gerakan otomatis.
5. **Theme** — pilih `Dark`, `Light`, atau `Gray` dari header. Pilihan disimpan ke `config.json`.

Data runtime seperti `runtime_state.json`, log, snapshot kalibrasi, dan model lokal juga diabaikan oleh Git.

## Alur penggunaan yang disarankan

1. Jalankan aplikasi dan pastikan status GUI normal.
2. Uji Modbus dan vision dengan mode `MOCK`.
3. Buka simulator untuk memeriksa arah serta jangkauan gerak.
4. Hubungkan board PAROL6 dan pastikan status berubah menjadi `CONNECTED`.
5. Lakukan homing/kalibrasi sesuai prosedur perangkat keras.
6. Jalankan program dengan kecepatan rendah, lalu pantau response log dan emergency stop.

Contoh program tersedia di [`GUI/files/Programs`](GUI/files/Programs). Dokumentasi khusus fitur penelitian:

- [Panduan pengujian Modbus PAROL6](docs/PANDUAN_PENGUJIAN_MODBUS_PAROL6.md)
- [Panduan Vision Tool-Z](docs/VISION_TOOL_Z.md)
- [Rangkuman IK, MoveCart, dan vision](docs/RANGKUMAN_IK_MOVECART_VISION.md)

## Pengujian

Install `pytest` pada virtual environment pengembangan, lalu jalankan seluruh unit test dari root repository:

```bash
source .venv/bin/activate
python -m pip install pytest
python -m pytest -q
```

Tes mencakup kontrol program, kalibrasi kamera, transformasi camera-to-base, validasi safe-pick, akurasi pick, runtime Tool-Z, dan helper tema UI. Tes unit tidak menggantikan validasi gerak pada simulator maupun robot fisik.

## Struktur repository

```text
GUI/files/                    GUI, komunikasi serial, simulator, dan program contoh
backend/                      konfigurasi, kontrol program, logging, dan helper tema
vision/                       deteksi, kalibrasi, workspace, safe-pick, dan Tool-Z
tests/                        unit test
docs/                         panduan pengujian dan catatan teknis
tools/Camera/param/           parameter intrinsik kamera
run_linux.sh                  launcher Linux
requirements.txt              dependensi Python
```

Entry point aplikasi adalah `GUI/files/Serial_sender_good_latest.py`. Modul tersebut menjalankan komunikasi robot dan menghubungkan GUI dengan adapter Modbus/vision di `GUI/files/Commander_feature_adapters.py`.

## Troubleshooting singkat

- **GUI gagal dibuka:** pastikan Python 3.10, Tk, dan semua dependensi pada `requirements.txt` terpasang di `.venv`.
- **Port serial ditolak di Linux:** tambahkan user ke grup `dialout` dengan `sudo usermod -aG dialout "$USER"`, lalu logout dan login kembali.
- **Robot tidak terhubung:** periksa kabel, daya, firmware, port serial, dan izin device.
- **Modbus masih memakai target lama:** klik **Disconnect**, simpan konfigurasi, lalu **Connect** kembali.
- **Deteksi model gagal:** periksa path model ONNX dan kecocokan runtime CUDA. Gunakan `MOCK` atau metode non-model untuk isolasi masalah.
- **Asset tidak ditemukan:** jalankan melalui `./run_linux.sh` atau mulai Python dari direktori `GUI/files`.

## Sumber dan lisensi

Proyek ini dikembangkan dari PAROL Commander Software milik [Source Robotics](https://source-robotics.com). Dokumentasi dan desain mekanik PAROL6 tersedia di [PAROL6 Desktop Robot Arm](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm) serta [PAROL Documentation](https://source-robotics.github.io/PAROL-docs/).

Kode didistribusikan di bawah lisensi [GPLv3](LICENSE). Perangkat lunak masih dalam tahap pengembangan dan digunakan atas risiko pengguna.
