# Vision Tool-Z

## Kontrak Koordinat

Alur koordinat yang digunakan:

```text
Camera Pixel (u,v)
-> undistorted pixel
-> Camera-to-Base homography
-> target (Xbase,Ybase)
-> FK pose referensi
-> perpindahan Z pada Tool Frame
```

Kamera tidak menghasilkan koordinat Tool. Kamera menghasilkan pixel yang
diubah menjadi Base XY. FK pose referensi kemudian memproyeksikan target Base
XY ke sumbu Z Tool.

Pose referensi:

```text
[90, -88, 182.259, 0, 3, 180] deg
```

## Kalibrasi Kamera ke Base

1. Pasang kamera fixed overhead dan jangan mengubah posisi, zoom, atau resolusi.
2. Jalankan intrinsic camera calibration dengan minimal tiga snapshot
   chessboard yang valid.
3. Pasang pointer pada TCP robot. Ujung pointer harus mewakili endpoint Tool
   yang dipakai oleh FK dan `MoveCartRelTRF`.
4. Pertahankan ujung pointer pada bidang kerja yang sama dengan pick point.
5. Buka panel Vision dan hidupkan kamera.
6. Klik satu titik fisik pada live feed.
7. Jog robot sampai ujung pointer tepat menyentuh titik fisik yang diklik.
8. Tekan `Add TCP/Base Point`. GUI menyimpan pixel dan FK TCP Base XY.
9. Ulangi sampai minimal sembilan titik tersebar merata di seluruh area kerja.
10. Tekan `Solve Camera-to-Base`.
11. Kalibrasi diterima bila RMS maksimal 3 mm dan error maksimum 5 mm.

Perubahan posisi kamera, resolusi, atau zoom mewajibkan kalibrasi ulang.

## Diagnosis Offline

Gunakan `GUI/files/Programs/vision_diagnostic.txt`:

```text
Begin()
print("pick start")
vision()
print("$vision.z_minus")
print("$vision.z_plus")
End()
```

Robot dan simulator boleh tidak aktif. Kamera, model YOLO, intrinsic
calibration, dan Camera-to-Base calibration tetap harus tersedia.

Offset yang dapat disetel di panel Vision:

- `Offset X`: offset target Base X.
- `Offset Y`: offset target Base Y.
- `Z Tool offset`: koreksi hasil proyeksi sepanjang Z Tool.
