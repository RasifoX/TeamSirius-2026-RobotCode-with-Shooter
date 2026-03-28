package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.util.Set;

/**
 * CONSTANTS — REBUILT 2026
 *
 * AprilTag koordinatları ve yükseklikleri resmi FRC 2026 Field Dimension
 * Drawings'den alınmıştır (FE-2026 Rev B, sheet 11, AndyMark Perimeter).
 * Türkiye Bölgesi = AndyMark tip saha (FE-2026 BOM, sheet 2).
 * Kaynak: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
 *
 * Resmi AprilTag yükseklikleri (Z sütunu, karpetten merkeze kadar):
 *   HUB    → Z = 44.25in = 1.1240m  (ID: 2-5, 8-11, 18-21, 24-27)
 *   TRENCH → Z = 35.00in = 0.8890m  (ID: 1,6,7,12,17,22,23,28)
 *   TOWER  → Z = 21.75in = 0.5524m  (ID: 15,16,31,32)
 *   OUTPOST→ Z = 21.75in = 0.5524m  (ID: 13,14,29,30)
 */
public final class Constants {

  // =============================================================================
  // 1. MODÜL AYARLARI
  // =============================================================================
  public static final class ModuleConstants {
    public static final double kNominalVoltage       = 12.0;
    public static final double kWheelDiameterMeters  = Units.inchesToMeters(4.0);
    public static final double kWheelCircumference   = kWheelDiameterMeters * Math.PI;
    public static final double kDriveGearRatio       = 6.75;
    public static final double kSteerGearRatio       = 150.0 / 7.0;
    public static final double kDrivePositionFactor  = (1.0 / kDriveGearRatio) * kWheelCircumference;
    public static final double kDriveVelocityFactor  = kDrivePositionFactor / 60.0;
    public static final double kSteerPositionFactor  = (1.0 / kSteerGearRatio) * (2.0 * Math.PI);
    public static final double kSteerVelocityFactor  = kSteerPositionFactor / 60.0;
    public static final double kTurningP             = 1.0;
    public static final double kTurningI             = 0.0;
    public static final double kTurningD             = 0.05;
    public static final int    kDriveCurrentLimit    = 50;
    public static final int    kSteerCurrentLimit    = 20;
  }

  // =============================================================================
  // 2. ŞASİ AYARLARI
  // =============================================================================
  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase  = Units.inchesToMeters(24.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kUseNavX     = false;
    public static final int     kPigeonID    = 13;
    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed         = 2 * Math.PI;
  }

  // =============================================================================
  // 3. PORT HARİTASI
  // Swerve: Drive 1,4,7,10 | Steer 2,5,8,11 | CANCoder 3,6,9,12
  // Shooter ve Indexer motorları 14+ ID kullanmak ZORUNDA (çakışmaması için)
  // =============================================================================
  public static final class PortConstants {
    // Front Left
    public static final int     kFrontLeftDriveID       = 1;
    public static final int     kFrontLeftSteerID       = 2;
    public static final int     kFrontLeftCANCoderID    = 3;
    public static final double  kFrontLeftOffset        = 0.0; // Pit'te saveOffsets() ile doldur
    public static final boolean kFrontLeftDriveInverted = false;
    public static final boolean kFrontLeftSteerInverted = false;

    // Front Right
    public static final int     kFrontRightDriveID       = 4;
    public static final int     kFrontRightSteerID       = 5;
    public static final int     kFrontRightCANCoderID    = 6;
    public static final double  kFrontRightOffset        = 0.0;
    public static final boolean kFrontRightDriveInverted = true;
    public static final boolean kFrontRightSteerInverted = false;

    // Rear Left
    public static final int     kRearLeftDriveID       = 7;
    public static final int     kRearLeftSteerID       = 8;
    public static final int     kRearLeftCANCoderID    = 9;
    public static final double  kRearLeftOffset        = 0.0;
    public static final boolean kRearLeftDriveInverted = false;
    public static final boolean kRearLeftSteerInverted = false;

    // Rear Right
    public static final int     kRearRightDriveID       = 10;
    public static final int     kRearRightSteerID       = 11;
    public static final int     kRearRightCANCoderID    = 12;
    public static final double  kRearRightOffset        = 0.0;
    public static final boolean kRearRightDriveInverted = true;
    public static final boolean kRearRightSteerInverted = false;
  }

  // =============================================================================
  // 4. OPERATÖR AYARLARI
  // =============================================================================
  public static final class OIConstants {
    public static final int    kDriverControllerPort = 0;
    public static final double kDriveDeadband        = 0.05;
  }

  // =============================================================================
  // 5. SHOOTER AYARLARI
  //
  // ⚠️ CAN ID: Swerve 1-12 arası kullanıyor.
  //    Shooter motorları 14+ olmalı. Fiziksel motorların üzerindeki
  //    etiketle eşleştir, yanlış ID = yanlış motor veya çakışma!
  // =============================================================================
  public static final class ShooterConstants {

    // ── CAN ID'leri ─────────────────────────────────────────────
    // TODO: Fiziksel motor etiketiyle birebir eşleştir
    public static final int ShooterMotor1ID  = 14;
    public static final int ShooterMotor2ID  = 15;
    public static final int ShooterFeeder3ID = 16;
    public static final int ShooterMotor4ID  = 17;
    public static final int ShooterFeeder5ID = 18;
    public static final int IndexerMotorID   = 28;

    // ── Yön Ayarları ────────────────────────────────────────────
    public static final boolean ShooterMotor1Inverted  = false;
    public static final boolean ShooterMotor2Inverted  = true;
    public static final boolean ShooterFeeder3Inverted = true;
    public static final boolean ShooterMotor4Inverted  = false;
    public static final boolean ShooterFeeder5Inverted = false;
    public static final boolean IndexerMotorInverted   = false;

    // ── Temel Hız ───────────────────────────────────────────────
    // ShooterBaseSpeed = 1.0 → hızı aşağıdaki scale'lerle kontrol et
    public static final double ShooterBaseSpeed     = 1.0;
    public static final double IndexerBaseSpeed     = 1.0;
    public static final double IndexerDirectionSign = 1.0; // Ters dönüyorsa -1.0 yap

    // ── MOD 3: Dinamik Hız Tablosu ──────────────────────────────
    // Mesafe sınırları (metre). Practice'de ölçerek doldur.
    // SmartDashboard'da "Shooter/Hesap Mesafe" değerine bakarak tune et.
    public static final double kDistanceClose = 1.5; // 0 – 1.5m
    public static final double kDistanceMid   = 2.5; // 1.5 – 2.5m
    public static final double kDistanceFar   = 3.5; // 2.5 – 3.5m
                                                      // 3.5m+ → kSpeedMax

    // Hız çarpanları (ShooterBaseSpeed * scale = gerçek hız)
    // TODO: Practice'de tune et
    public static final double kSpeedClose    = 0.50;
    public static final double kSpeedMid      = 0.70;
    public static final double kSpeedFar      = 0.85;
    public static final double kSpeedMax      = 1.00;

    // Limelight hedef göremeyince güvenli sabit hız
    public static final double ShooterFallbackScale = 0.70;
  }

  // =============================================================================
  // 6. LIMELIGHT & HİZALAMA AYARLARI
  //
  // Resmi FRC 2026 sahadan alınan HUB yüksekliği:
  //   44.25 inch = 1.1240 m (FE-2026 Rev B, sheet 11, Z sütunu)
  //
  // HUB AprilTag ID'leri (resmi, REBUILT 2026):
  //   Red  Hub: 2, 3, 4, 5, 8, 9, 10, 11
  //   Blue Hub: 18, 19, 20, 21, 24, 25, 26, 27
  // =============================================================================
  public static final class LimelightConstants {
    public static final String LIMELIGHT_NAME = "limelight";

    // ── Geometri (Mesafe Hesabı İçin) ───────────────────────────
    // ⚠️ KUMPASLA ÖLÇEREK GİR — yanlış değer = yanlış mesafe hesabı

    // Limelight'ın yere göre bakış açısı (derece).
    // Tamamen düz montaj = 0°. Robota montaj açını ölç.
    public static final double kLimelightMountAngleDeg = 30.0; // TODO: ölç ve gir

    // Limelight lens merkezinin karpetten yüksekliği (metre).
    public static final double kLimelightHeightMeters = 0.50;  // TODO: ölç ve gir

    // Hub AprilTag yüksekliği — resmi FRC 2026 sahadan (değiştirme!)
    // Kaynak: FE-2026 Rev B, sheet 11 → Z = 44.25 in
    public static final double kTargetHeightMeters = Units.inchesToMeters(44.25); // 1.1240m

    // ── Hizalama Hız Limitleri ───────────────────────────────────
    public static final double maxTurnSpeed    = 1.5;  // rad/s
    public static final double maxStrafeSpeed  = 1.0;  // m/s
    public static final double maxForwardSpeed = 1.0;  // m/s

    // ── PID Ayarları ─────────────────────────────────────────────
    // Practice'de SmartDashboard üzerinden tune et.
    // Robot sallanıyor/aşırı tepki veriyorsa kP'yi düşür.
    // Yavaş hizalanıyorsa kP'yi arttır.
    public static final double kP_X     = 0.03;
    public static final double kI_X     = 0.0;
    public static final double kD_X     = 0.00005;

    public static final double kP_Y     = 0.03;
    public static final double kI_Y     = 0.0;
    public static final double kD_Y     = 0.00005;

    // Theta (dönüş) PID — Hub hizalamada kullanılmaz (heading lock yok)
    // AlignToTagCMD Hub tag gördüğünde heading'e dokunmaz.
    public static final double kP_Theta = 0.04;
    public static final double kI_Theta = 0.0;
    public static final double kD_Theta = 0.00005;

    // ── Hedef Setpointleri ───────────────────────────────────────
    // TARGET_TX_SETPOINT = 0 → tag yatayda tam ortada
    // TARGET_TY_SETPOINT: "Tam bu mesafeden atacağım" noktasının TY değeri.
    //   Practice'de robotu ideal atış mesafesine götür, Limelight TY'ye bak,
    //   o değeri buraya yaz.
    public static final double TARGET_TX_SETPOINT = 0.0;
    public static final double TARGET_TY_SETPOINT = 0.0; // TODO: practice'de bul

    // ── HUB AprilTag ID'leri (Resmi FRC 2026 REBUILT) ───────────
    // Kaynak: FE-2026 Rev B, sheet 11
    // AlignToTagCMD bu tag'leri görünce heading lock YAPMAZ (Hub için gereksiz)
    public static final Set<Integer> kHubRedIDs  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    public static final Set<Integer> kHubBlueIDs = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    public static final Set<Integer> kHubIDs     = Set.of(
        2, 3, 4, 5, 8, 9, 10, 11,
        18, 19, 20, 21, 24, 25, 26, 27);

    // Diğer element ID'leri (referans)
    public static final Set<Integer> kTowerIDs   = Set.of(15, 16, 31, 32);
    public static final Set<Integer> kOutpostIDs = Set.of(13, 14, 29, 30);
    public static final Set<Integer> kTrenchIDs  = Set.of(1, 6, 7, 12, 17, 22, 23, 28);
  }
}