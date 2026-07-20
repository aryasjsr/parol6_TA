# Rangkuman Analisis: IK, MoveCart, dan Kompensasi Vision (x_comp)

> Hasil sesi analisis 12 Juli 2026 — verifikasi program pick-and-place PAROL6
> menggunakan model robot asli (`PAROL6_ROBOT.py`) dan replika pipeline
> `MoveCartRelTRF`/`MoveCart` dari `Serial_sender_good_latest.py`
> (fkine → ctraj → subdivision `ikine_LMS` → cek limit).

---

## 1. Konsep Kunci

### 1.1 Frame TRF vs Base — sumber kebingungan utama

`MoveCartRelTRF(x,y,z,rx,ry,rz)` bergerak di **Tool Reference Frame** (mm, derajat),
**bukan** frame base/dunia. Pada pose kerja `MoveJoint(90,-88,174,0,3,177)`:

| Sumbu tool | Arah di dunia (base) | Artinya |
|---|---|---|
| tool **X** | `[0.05, 0.09, **0.99**]` | hampir lurus **ke atas** dunia |
| tool Y | `[0.99, 0.00, -0.05]` | ke arah +X dunia |
| tool **Z** | `[0.00, **0.99**, -0.087]` | hampir horizontal, **maju +Y dunia**, miring turun 8.7% |

Konsekuensi:
- `z` tool = gerak **maju/mundur** (bukan naik/turun!), dengan efek samping
  vertikal kecil `-0.087 × z`.
- `x` tool = gerak **naik/turun** dunia (hampir 1:1).
- Kasus nyata: `MoveCartRelTRF(0,0,-130,...)` terlihat "mengangkat" karena
  `Z=-130` = mundur 129.5 mm + **naik 11.3 mm** (komponen miring 8.7%).

### 1.2 Rumus kompensasi vertikal (khusus pose ini)

Agar perpindahan vertikal dunia = 0 untuk perintah `[x, 0, z]` tool:

```
dZ_dunia = 0.9948·x − 0.087·z  =>  x = 0.0875 · z
```

Contoh: z=−130 → x=−11.37; z=+60 → x=+5.25.
**Rasio 0.0875 hanya berlaku di orientasi wrist ini** — pose lain punya matriks
rotasi berbeda.

### 1.3 MoveCartRelTRF (relatif) vs MoveCart (absolut)

| | `MoveCartRelTRF` | `MoveCart` |
|---|---|---|
| Target | relatif dari posisi sekarang (frame tool) | **absolut di frame base** (`SE3.RPY` + t, [Serial_sender_good_latest.py:1952-1955]) |
| Jika step sebelumnya bervariasi (vision) | titik akhir ikut bergeser | **titik akhir selalu sama** |
| Kegunaan | approach/retract mengikuti vision | titik place yang harus konsisten |

Temuan menarik: formulasi relatif `z_minus = -(z_plus+70)` **sudah hampir konstan**
secara aljabar (step2 `+z_plus` dan step3 `-(z_plus+70)` saling batal, sisa net
`tool(-20,0,-70)`), sebaran endpoint hanya <0.04 mm. `MoveCart` membuatnya eksak
dan menghapus kebutuhan `z_minus` sama sekali.

---

## 2. Hasil Verifikasi Keselamatan

### 2.1 Limit joint — AMAN ✅

Limit robot (`PAROL6_ROBOT.Joint_limits_degree`):
J1 ±123°, J2 [−145°,−3.4°], J3 [108°,288°], J4 ±105.5°, J5 ±90°, J6 [0°,360°].

Semua varian program yang diuji lolos limit di seluruh waypoint (300–400 per gerak).
Joint paling ketat selalu J2 (mencapai −112° pada retract dalam; margin ≥ 33°).

### 2.2 IK — konvergen ✅

`ikine_LMS` + subdivision berhasil di semua waypoint untuk seluruh rentang
z_plus yang diizinkan. Reach puncak 0.41 m < batas 0.44 m
(`calculate_configuration_dependent_max_reach`; berkurang ke ~0.395 bila J5 dekat ±90°).

### 2.3 Singularitas — ⚠️ PERINGATAN TETAP BERLAKU

Pose kerja `MoveJoint(90,-88,174,0,3,177)` punya manipulabilitas **0.000357**,
di bawah ambang `singularity_threshold = 0.001`. Seluruh lintasan bekerja di zona
dekat-singular (min ~0.00019–0.00027). Dampak:
- IK otomatis pakai toleransi longgar (1e-7) → akurasi Cartesian sedikit berkurang;
- potensi gerak tersentak; **jalankan pertama kali dengan kecepatan rendah**.
- Belum diperbaiki; rencana: cari MoveJoint alternatif dekat nilai sekarang
  dengan manipulabilitas lebih tinggi (tertunda).

### 2.4 Rentang z_plus yang diizinkan

**Tanpa kompensasi** (step2 = `tool(-20, 0, z_plus)`):

| Rentang | Status |
|---|---|
| 0–95 mm | ✅ aman penuh |
| 96–107 mm | ⚠️ lolos, tapi manipulabilitas anjlok (0.00019→0.00002) |
| ≥108 mm | ❌ limit joint dilanggar (≥120: reach limit) |

**Dengan kompensasi x_comp** (step2 = `tool(-20+0.0875·z_plus, 0, z_plus)`):

| Rentang | Status |
|---|---|
| 0–80 mm | ✅ aman & nyaman |
| 81–93 mm | ⚠️ 90+ nyaris singular (manip 0.000079 @90, 0.000025 @92) |
| ≥94 mm | ❌ IK GAGAL |

→ `z_plus_max_mm` di config diturunkan **105 → 85** (batas keras kompensasi = 93).

### 2.5 Efek kompensasi terhadap dZ dunia

| | Tanpa kompensasi | Dengan x_comp |
|---|---|---|
| Turun step 2 | −24.3 … −28.6 mm (variasi ~4.4 mm ikut z_plus) | **−19.9 mm konstan** |
| Tinggi grip dunia | 234→230 mm (beda 4 mm) | **238.5 mm konstan** |
| Angkat step 3 (MoveCart) | +10 … +15 mm | **+6.1 mm konstan** |

Step 3 (retract ke titik place) **monoton tanpa dip/overshoot** — naik halus,
bukan menukik.

---

## 3. Fitur Baru: `$vision.x_comp`

Placeholder aritmatika (`-20 + 0.0875*$vision.z_plus`) **tidak bisa** di-parse
regex command — solusinya vision runtime menyediakan nilai jadi.

Perubahan yang diimplementasikan (semua terverifikasi, 7/7 tes lama lulus):

| File | Perubahan |
|---|---|
| `vision/tool_z_runtime.py` | pattern `+x_comp`; `ToolZResult.x_comp_mm`; `calculate_tool_z(... x_comp_base_mm=-20, x_comp_slope=0.0875)`; resolver dengan error jelas bila runtime lama |
| `GUI/files/Commander_feature_adapters.py` | `x_comp_mm` masuk runtime; parameter dari config; `Xcomp=` di log vision & research logger |
| `GUI/files/Serial_sender_good_latest.py:2744` | guard pose referensi juga aktif untuk `$vision.x_comp` |
| `config.json` (`vision.tool_z`) | `+x_comp_base_mm: -20.0`, `+x_comp_slope: 0.0875`, `z_plus_max_mm: 105 → 85` |
| `GUI/files/Programs/tes_vision_terbaru.txt` | program contoh (baru) |

`print("X comp = $vision.x_comp mm")` bekerja (jalur print memakai resolver yang sama).

---

## 4. Program Rekomendasi Final

`GUI/files/Programs/tes_vision_terbaru.txt`:

```
Begin()
print("pick start")
Output(1,LOW)
Output(2,HIGH)
MoveJoint(90,-88,174,0,3,177,t=4)
vision()
print("Z+ Tool = $vision.z_plus mm")
print("X comp = $vision.x_comp mm")
Delay(1)
MoveCartRelTRF($vision.x_comp,0,$vision.z_plus,0,0,0,t=4)
Delay(1)
Output(1,HIGH)
Output(2,LOW)
MoveCart(-3.4,197.5,244.6,-87.0,0.0,-95.0,t=4)
Delay(1)
End()
```

- **Step 2** (approach): maju `z_plus` mm ke objek, turun konstan 19.9 mm
  berkat `x_comp` — tinggi grip identik untuk semua jarak objek.
- **Step 3** (place): `MoveCart` absolut → titik place **selalu sama persis**
  (`(-3.4, 197.5, 244.6)` mm, RPY `(-87, 0, -95)°`), identik dengan endpoint versi
  relatif lama (selisih <0.04 mm) — drop-in replacement, `z_minus` tak dipakai lagi.

## 5. Pekerjaan Tertunda

1. **Cari MoveJoint alternatif anti-singularitas** dekat `(90,-88,174,0,3,177)` —
   satu-satunya kelemahan tersisa (manipulabilitas < ambang di seluruh lintasan).
2. Uji di robot nyata dengan kecepatan rendah; amati sentakan di step retract.
