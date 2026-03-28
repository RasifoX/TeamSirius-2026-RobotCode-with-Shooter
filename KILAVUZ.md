# FRC Team Sirius 2026 — Practice Günü Kılavuzu

> **İnternet yok, sorun değil. Bu dosya her şeyi içeriyor.**

---

## 📋 Değiştirilen Dosyalar (Deploy Edilecekler)

| Dosya | Ne Değişti |
|---|---|
| `DriveSubsystem.java` | CAN bus spam fix — `m_lastCurrentLimit` cache eklendi |
| `RobotContainer.java` | X butonu `ConditionalCommand`'a çevrildi, deprecated `schedule()` kaldırıldı |

Diğer tüm dosyalar aynı. Sadece bu 2 dosyayı değiştir, deploy et.

---

## 🔧 ADIM 1 — PİT: Deploy Öncesi Kontrol (Robot Kapalı)

### Constants.java'da Kontrol Et

```java
// Hangi gyro varsa:
DriveConstants.kUseNavX = false;   // Pigeon2 için false, NavX için true
DriveConstants.kPigeonID = 13;     // Fiziksel CAN ID ile eşleşiyor mu?

// Shooter motor CAN ID'leri — fiziksel etiketle birebir mi?
ShooterConstants.ShooterMotor1ID  = 14;
ShooterConstants.ShooterMotor2ID  = 15;
ShooterConstants.ShooterFeeder3ID = 16;
ShooterConstants.ShooterMotor4ID  = 17;
ShooterConstants.ShooterFeeder5ID = 18;
ShooterConstants.IndexerMotorID   = 28;
```

Deploy et. **BUILD SUCCESSFUL** gördükten sonra robotu aç.

---

## 🔧 ADIM 2 — PİT: CANcoder Offset Alma (Tekerlekler Havada)

> ⚠️ Bu adımı atlama. Atlassan robot yana gider veya döner.

### Yapılacaklar

1. Tekerleklerin hepsini **elle çevir**, düz ileri baksın (cetvel koy önlerine)
2. Shuffleboard'u aç → şu değerleri gör:

```
CANcoder/FL rot  →  örn: 0.2341
CANcoder/FR rot  →  örn: -0.1823
CANcoder/RL rot  →  örn: 0.4102
CANcoder/RR rot  →  örn: -0.3271
```

3. `saveOffsets()` tetikle — geçici olarak Start tuşuna bağla:

```java
// RobotContainer.java → configureBindings() içine ekle (sadece bugün)
new JoystickButton(m_controller, XboxController.Button.kStart.value)
    .onTrue(new InstantCommand(() -> m_robotDrive.saveOffsets()));
```

4. Start'a bas → konsola şunu yazacak:

```
[OFSETLERİ KAYDET] FL=0.2341  FR=-0.1823  RL=0.4102  RR=-0.3271 rot
→ Bu değerleri Constants.java'daki kXxxOffset sabitlerine de kopyala
```

5. **Bu değerleri Constants.java'ya kopyala:**

```java
PortConstants.kFrontLeftOffset  = 0.2341;   // ← konsoldaki FL değeri
PortConstants.kFrontRightOffset = -0.1823;  // ← FR değeri
PortConstants.kRearLeftOffset   = 0.4102;   // ← RL değeri
PortConstants.kRearRightOffset  = -0.3271;  // ← RR değeri
```

6. Tekrar deploy et.

---

## 🏟️ ADIM 3 — SAHA: Temel Sürüş Testleri

Robotu saha ortasına koy, düz ileri baksın.

### Test 1 — İleri/Geri
- Sol stick ileri → robot **düz ileri** gitmeli
- ❌ Yana kayıyorsa → offset yanlış, pit'e dön

### Test 2 — Strafe (Yan)
- Sol stick sağa → robot **sağa kaymalı, dönmemeli**
- ❌ Dönüyorsa → offset sorunu

### Test 3 — Rotasyon
- Sağ stick sağa → robot **yerinde saat yönünde** dönmeli
- ❌ Hem dönüp hem kayıyorsa → bir modülün `DriveInverted` yanlış

### Test 4 — Field Oriented
- Robotu 90° döndür, sol stick ileri it
- Robot **sahaya göre** ileri gitmeli (kendi burnuna göre değil)
- ❌ Kendi burnuna göre gidiyorsa → `kGyroReversed = true` yap, deploy

### Test 5 — Gyro Sıfırlama
- Y tuşuna bas → Shuffleboard'da `Cockpit/Heading deg` **0'a dönmeli**

---

## 🏟️ ADIM 4 — SAHA: Shooter Testleri

SmartDashboard → **"Shooter Modu" → "🔒 SABİT HIZ"** seç.

### Motor Yön Testi
- **B tuşuna bas** → tüm shooter motorları dönmeli
- Top doğru yönde çıkıyor mu?
- ❌ Ters çıkıyorsa → Constants'ta ilgili motorun `Inverted`'ını değiştir

### Indexer Testi
- **RB tuşuna bas** → indexer ters dönmeli (sıkışma açma)

### İnvert Değerleri (gerekirse değiştir)

```java
ShooterConstants.ShooterMotor1Inverted  = false;  // ← değiştir
ShooterConstants.ShooterMotor2Inverted  = true;
ShooterConstants.ShooterFeeder3Inverted = true;
ShooterConstants.ShooterMotor4Inverted  = false;
ShooterConstants.ShooterFeeder5Inverted = false;
ShooterConstants.IndexerMotorInverted   = false;
```

---

## 🏟️ ADIM 5 — SAHA: Limelight Kalibrasyonu

Önce Limelight'ın Hub tag'ini görüp görmediğini kontrol et:

```
Limelight/HasTarget  →  true olmalı
Limelight/TargetID   →  2-5 veya 18-27 arası olmalı (Hub ID'leri)
```

### Montaj Ölçümleri (Kumpasla Ölç)

```java
// Constants.java → LimelightConstants
kLimelightMountAngleDeg = 30.0;  // ← Limelight'ın bakış açısını ölç, buraya yaz
kLimelightHeightMeters  = 0.50;  // ← Lens merkezinin yüksekliğini ölç, buraya yaz
```

### TY Setpoint Bulma

1. Robotu **ideal atış mesafesine** götür (geçen seneki atış noktası veya test edilen mesafe)
2. Limelight Hub tag'i görüyor olsun
3. Shuffleboard → `Limelight/TY` değerine bak → örn: `-8.3`
4. Constants'a yaz:

```java
LimelightConstants.TARGET_TY_SETPOINT = -8.3;  // ← ölçtüğün değer
```

Deploy et.

### Hizalama Testi (X Tuşu)

SmartDashboard → **"Shooter Modu" → "📍 STATİK"** seç.

X tuşuna bas → robot Hub'a doğru hizalanmalı.

```
Align/Durum  →  HİZALANIYOR
Align/TX     →  0'a yaklaşmalı (yatay hizalama)
Align/TY     →  TARGET_TY_SETPOINT'e yaklaşmalı (mesafe)
```

#### Robot Sallanıyorsa (Oscillation)
```java
// Constants.java → kP değerlerini yarıya düşür
LimelightConstants.kP_X = 0.015;  // 0.03'ten düştük
LimelightConstants.kP_Y = 0.015;
```

#### Robot Yavaş Hizalanıyorsa
```java
// kP değerlerini biraz artır
LimelightConstants.kP_X = 0.05;
LimelightConstants.kP_Y = 0.05;
```

---

## 🚨 Anlık Sorun Giderme

| Belirti | Muhtemel Sebep | Çözüm |
|---|---|---|
| Bir tekerlek ters dönüyor | Drive invert yanlış | Constants → o modülün `DriveInverted` değiştir |
| Robot strafe'de dönüyor | Offset yanlış | Pit'e dön, offset tekrar al |
| Gyro heading zıplıyor / saçma değer | Pigeon CAN ID yanlış | `kPigeonID` kontrol et |
| Field oriented çalışmıyor | Gyro ters | `kGyroReversed = true` |
| Align "Hub Tag Değil" yazıyor | Yanlış tag görüyor | Limelight pipeline kontrol et |
| Align "Hedef YOK" yazıyor | Limelight görmüyor | Limelight power/network bağlantısı |
| B tuşunda motorlar dönmüyor | Mod yanlış | SmartDashboard'da modu FIXED seç |
| Robot brownout'ta donuyor | Pil düştü | Konsola `[AKIM LİMİT] 30A` yazar — normaldir, pili değiştir |
| Konsola `[PATHPLANNER] Config yüklenemedi` | PP config dosyası yok | PathPlanner GUI'den robot config oluştur |

---

## 🔑 Kritik Kod Notları

### Neden DriveSubsystem değişti?
Eski kodda `periodic()` her 20ms'de 4 SparkMax'e `configure()` gönderiyordu. Bu CAN bus'ı spamlar, motor controller donabilir. Yeni kodda sadece 10.5V eşiği geçildiğinde gönderiliyor.

### Neden RobotContainer değişti?
Eski X butonu `RunCommand` içinden deprecated `schedule()` çağırıyordu ve her 20ms tetikleniyordu. Yeni `ConditionalCommand` yapısı buton basıldığında modu bir kez kontrol eder, WPILib altyapısı buton bırakılınca otomatik cancel eder.

### Offset Matematiği
```
encoder_rad = (cancoder_rotations - offset_rotations) × 2π
```
Sonuç `[-π, +π]` aralığına normalize edilir. Tekerlek ileri bakarken encoder = 0 rad.

### Wrapping Range
Steer PID wrapping `[-π, +π]` olarak ayarlı. Bu sayede -90° hedefine (sağa strafe) giderken motor 270° uzun yol yerine 90° kısa yoldan döner.

---

## ✅ Practice Bitiş Kontrol Listesi

Ayrılmadan önce bunları işaretle:

- [ ] Offset değerleri Constants.java'ya yazıldı ve deploy edildi
- [ ] 4 yönde sürüş temiz (ileri, geri, sol, sağ, rotasyon)
- [ ] Field oriented çalışıyor (Y sıfırlama dahil)
- [ ] B tuşu atıyor, RB sıkışma açıyor
- [ ] `TARGET_TY_SETPOINT` ideal mesafeden ölçüldü
- [ ] `kLimelightMountAngleDeg` ve `kLimelightHeightMeters` girildi
- [ ] Hizalama (X tuşu) TX→0 ve TY→setpoint'e gidiyor
- [ ] Konsol temiz, CAN error yok

---

## 📁 Dosya Yapısı Referansı

```
src/main/java/frc/robot/
├── Main.java
├── Robot.java
├── RobotContainer.java          ← DEĞİŞTİ
├── Constants.java               ← practice'de doldurulacak
├── commands/
│   ├── AlignToTagCMD.java
│   ├── Autos.java
│   └── SwerveJoystickCmd.java
└── subsystems/
    ├── DriveSubsystem.java      ← DEĞİŞTİ
    ├── SwerveModule.java
    ├── ShooterSubsystem.java
    ├── LimelightSubsystem.java
    ├── GyroIO.java
    ├── Pigeon2IO.java
    └── NavXIO.java
```

---

*Bol şans yarın! 🤖*