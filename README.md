# 🚁 WSN Tabanlı Çok Ajanlı Drone Alan Tarama Sistemi

Webots simülasyonunda merkez istasyon + 3 otonom MAVIC 2 PRO drone ile lawn-mower deseninde alan tarama, RGB görüntü işleme ile kırmızı nesne tespiti, GPS tabanlı konum kilitleme ve merkeze kablosuz ALERT bildirimi.

## 🏗️ Sistem Mimarisi
┌─────────────────────────────────────────────┐
│         MERKEZ İSTASYON (Base Station)      │
│         Supervisor (Emitter/Receiver)       │
│    ┌──────────────┐    ┌────────────────┐   │
│    │ ALERT Parse  │    │ PROGRESS Takip │   │
│    │ Koordinat    │    │ Harita Üzerinde│   │
│    │ Kaydet       │    │ İlerlemeyi     │   │
│    └──────────────┘    └────────────────┘   │
└──────────────┬──────────────────────────────┘
│ WSN (Emitter/Receiver)
┌──────────┴──────────┬───────────────────┐
▼                     ▼                   ▼
┌─────────┐        ┌─────────┐        ┌─────────┐
│ Drone-1 │◄──────►│ Drone-2 │◄──────►│ Drone-3 │
│camera1  │        │camera2  │        │camera3  │
│GPS/IMU  │        │GPS/IMU  │        │GPS/IMU  │
│emitter1 │        │emitter2 │        │emitter3 │
│receiver1│        │receiver2│        │receiver3│
└─────────┘        └─────────┘        └─────────┘

## ✨ Özellikler

- ✅ Otonom lawn-mower alan tarama (X-Y şerit deseni)
- ✅ Çok ajanlı senkronize sistem (3 bağımsız drone)
- ✅ Gerçek zamanlı RGB thresholding ile kırmızı nesne tespiti
- ✅ GPS tabanlı dünya koordinatlarına çevirim ve konum kilitleme
- ✅ WSN protokolü ile merkeze canlı ALERT & PROGRESS bildirimi
- ✅ PID-benzeri irtifa + roll/pitch/yaw stabilizasyonu
- ✅ X/Y sınır güvenliği ve otomatik düzeltme

## 🛠️ Teknolojiler

Webots R2023b+ | Python 3.8+ | GPS/IMU/Gyro | Camera/Display | Emitter/Receiver | RGB Thresholding

## ⚙️ Kurulum & Çalıştırma

```bash
git clone https://https://github.com/Bakipisirici/mavic-wsn--robotik-object-detection.git
cd wsn-drone-surveillance
# Webots'ta worlds/surveillance_world.wbt açın
# Controller'ları ilgili .py dosyalarına atayın, ▶ Play
