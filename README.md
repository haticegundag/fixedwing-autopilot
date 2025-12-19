# ATABEY UAV Fixed-Wing Autopilot

Bu depo, **Atabey İHA** projesi kapsamında geliştirilen, **STM32F411CEU6 (Black Pill) tabanlı sabit kanat (fixed-wing) otonom uçuş kontrol yazılımını** içermektedir. Sistem; sensör okuma, durum kestirimi, kontrol algoritmaları, görev yürütme ve yer istasyonu haberleşmesini **modüler ve genişletilebilir** bir mimariyle ele alır.

---

## 🎯 Proje Hedefleri

- Sabit kanat İHA için **tam otonom uçuş yeteneği**
- STM32 üzerinde **bare-metal / HAL tabanlı** gerçek zamanlı kontrol
- **Modüler mimari** (sürücüler, kontrol, haberleşme, görev mantığı)
- MATLAB/Simulink ile **sistem modelleme ve doğrulama**
- Yer istasyonu (GCS) ile **telemetri ve görev entegrasyonu**

---

## 🧠 Sistem Mimarisi (Özet)

Autopilot yazılımı aşağıdaki ana katmanlardan oluşur:

- **Drivers**  
  Donanım bağımlı katman (IMU, barometre, GPS, UART, I2C, SPI vb.)

- **Core**  
  Zamanlayıcı (Scheduler), görev döngüsü ve sistem altyapısı

- **Control**  
  PID tabanlı kontrol algoritmaları (Roll, Pitch, Yaw, hız vb.)

- **Comms**  
  Yer istasyonu / telemetri haberleşmesi (MAVLink benzeri yapı)

- **App (Autopilot)**  
  Sistem durum makinesi, uçuş modları ve üst seviye mantık

---

## 📁 Dizin Yapısı
```
fixedwing-autopilot/
│
├─ firmware/
│ └─ stm32/
│ ├─ App/
│ │ ├─ app/ # Autopilot ana kütüphanesi
│ │ ├─ core/ # Scheduler ve çekirdek sistemi
│ │ ├─ control/ # PID ve kontrol algoritmaları
│ │ ├─ comms/ # Haberleşme katmanı
│ │ ├─ drivers/ # Sensör ve donanım sürücüleri
│ │ └─ config/ # Kart ve sistem konfigürasyonları
│ │
│ ├─ Drivers/ # STM32 HAL sürücüleri
│ ├─ linker scripts/ # Flash / RAM linker dosyaları
│ └─ startup & system # Startup ve sistem dosyaları
│
├─ MATLAB/
│ ├─ scripts/ # Simülasyon ve analiz scriptleri
│ ├─ simulinkModelleri/ # Simulink sistem modelleri
│ └─ sistemDinamikleri/ # Uçak dinamiği ve matematiksel modeller
│
├─ LICENSE
└─ README.md
```

---

## 🔁 Çalışma Mantığı (Yüksek Seviye)

1. **MCU Boot**
2. Sensör ve haberleşme birimlerinin başlatılması
3. Scheduler üzerinden periyodik görevlerin çalıştırılması
4. Sensör verilerinin okunması
5. Durum kestirimi (attitude, hız, irtifa)
6. Kontrol algoritmalarının çalıştırılması
7. Aktüatör komutlarının üretilmesi
8. Telemetri verilerinin GCS’ye gönderilmesi

---

## 📊 MATLAB & Simulink

Bu depo, gömülü yazılım ile **aynı sistemin matematiksel modelini** de içerir:

- Uçak dinamikleri (longitudinal / lateral)
- PID kontrolcü tasarımı
- Simulink tabanlı sistem doğrulama
- Uçuş senaryosu simülasyonları

Bu sayede:
- Gerçek uçuş öncesi kontrolcü ayarları test edilir.
- Embedded kod ile teorik model arasında tutarlılık sağlanır.

---

## 🛠 Donanım Hedefi

- **MCU:** STM32F411 / STM32F401 (BlackPill uyumlu)
- **Sensörler:** IMU (MPU6050/9250), Barometre, GPS (u-blox)
- **Haberleşme:** UART / LoRa / RC
- **Aktüatörler:** Servo yüzeyler + ESC

---

## 🚀 Geliştirme Notları

- Kod yapısı **STM32** mimarisine yöneliktir.
- Her alt sistem (kütüphane) bağımsız geliştirilebilir.
- Scheduler tabanlı yapı dolayısıyla sonradan RTOS’a geçişe uygundur.
- PID ve kontrol katmanı kolayca genişletilebilir.

---

## 📌 Yol Haritası (Özet)

- [ ] Sensör Driverleri
- [ ] Durum kestirimi (EKF)
- [ ] Uçuş modları (AUTO, MANUAL)
- [ ] MAVLink uyumluluğu
- [ ] Donanım-in-the-loop (HIL) testleri
- [ ] Fail-safe ve güvenlik katmanları

---

## 👥 Katkı

Bu proje **Atabey İHA Elektronik Birimi** tarafından geliştirilmektedir.  
Katkı sağlamak için:

1. Fork oluştur
2. Feature branch aç
3. Temiz ve dokümante edilmiş PR gönder

---

