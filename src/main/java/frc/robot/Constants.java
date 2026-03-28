package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.util.Set;

/**
 * CONSTANTS (SABİTLER) - ROBOTUN DNA'SI
 * Burası "Ana Kadran Santrali"dir. Başka hiçbir dosyada el ile sayı girilmez.
 */
public final class Constants {

  // =============================================================================
  // 1. MODÜL AYARLARI (Module Constants)
  // =============================================================================
  public static final class ModuleConstants {
    public static final double kNominalVoltage = 12.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumference   = kWheelDiameterMeters * Math.PI;

    public static final double kDriveGearRatio = 6.75;
    public static final double kSteerGearRatio = 150.0 / 7.0;

    public static final double kDrivePositionFactor = (1.0 / kDriveGearRatio) * kWheelCircumference;
    public static final double kDriveVelocityFactor = kDrivePositionFactor / 60.0;

    public static final double kSteerPositionFactor = (1.0 / kSteerGearRatio) * (2.0 * Math.PI);
    public static final double kSteerVelocityFactor = kSteerPositionFactor / 60.0;

    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.05;

    public static final int kDriveCurrentLimit = 50;
    public static final int kSteerCurrentLimit = 20;
  }

  // =============================================================================
  // 2. ŞASİ AYARLARI (Drive Constants)
  // =============================================================================
  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase  = Units.inchesToMeters(24.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    public static final boolean kUseNavX     = false;
    public static final int     kPigeonID    = 13;
    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed         = 2 * Math.PI;
  }

  // =============================================================================
  // 3. PORT HARİTASI (Port Constants)
  // =============================================================================
  public static final class PortConstants {
    // Front Left
    public static final int    kFrontLeftDriveID    = 1;
    public static final int    kFrontLeftSteerID    = 2;
    public static final int    kFrontLeftCANCoderID = 3;
    public static final double kFrontLeftOffset     = 0.0;
    public static final boolean kFrontLeftDriveInverted  = false;
    public static final boolean kFrontLeftSteerInverted  = false;

    // Front Right
    public static final int    kFrontRightDriveID    = 4;
    public static final int    kFrontRightSteerID    = 5;
    public static final int    kFrontRightCANCoderID = 6;
    public static final double kFrontRightOffset     = 0.0;
    public static final boolean kFrontRightDriveInverted = true;
    public static final boolean kFrontRightSteerInverted = false;

    // Rear Left
    public static final int    kRearLeftDriveID    = 7;
    public static final int    kRearLeftSteerID    = 8;
    public static final int    kRearLeftCANCoderID = 9;
    public static final double kRearLeftOffset     = 0.0;
    public static final boolean kRearLeftDriveInverted = false;
    public static final boolean kRearLeftSteerInverted = false;

    // Rear Right
    public static final int    kRearRightDriveID    = 10;
    public static final int    kRearRightSteerID    = 11;
    public static final int    kRearRightCANCoderID = 12;
    public static final double kRearRightOffset     = 0.0;
    public static final boolean kRearRightDriveInverted = true;
    public static final boolean kRearRightSteerInverted = false;
  }

  // =============================================================================
  // 4. OPERATÖR AYARLARI (OI Constants)
  // =============================================================================
  public static final class OIConstants {
    public static final int    kDriverControllerPort = 0;
    public static final double kDriveDeadband        = 0.05;
  }

  // =============================================================================
  // 5. SHOOTER & INDEXER AYARLARI (Shooter Constants)
  // =============================================================================
  public static final class ShooterConstants {
      // Motor ID'leri (buradan deneyecek)
      public static final int ShooterMotor1ID = 10;
      public static final int ShooterMotor2ID = 2;
      public static final int ShooterFeeder3ID = 8; 
      public static final int ShooterMotor4ID = 4;
      public static final int ShooterFeeder5ID = 19; 
      public static final int IndexerMotorID = 28;

      // Motor Yönleri
      public static final boolean ShooterMotor1Inverted = false;
      public static final boolean ShooterMotor2Inverted = true;
      public static final boolean ShooterFeeder3Inverted = true;
      public static final boolean ShooterMotor4Inverted = false;
      public static final boolean ShooterFeeder5Inverted = false;
      public static final boolean IndexerMotorInverted = false; 

      // Hız Limitleri ve Çarpanlar
      public static final double ShooterBaseSpeed = 0.3;
      public static final double IndexerBaseSpeed = 1.0;
      public static final double IndexerDirectionSign = 1.0; // Ters dönüyorsa -1.0 yap
  }

  // =============================================================================
  // 6. LIMELIGHT & HİZALAMA AYARLARI (Vision Constants)
  // =============================================================================
  public static final class LimelightConstants {
      public static final String LIMELIGHT_NAME = "limelight";
      
      // Hizalama (Align) Hız Limitleri (m/s ve rad/s)
      public static final double maxTurnSpeed = 1.5; 
      public static final double maxStrafeSpeed = 1.0;
      public static final double maxForwardSpeed = 1.0;

      // PID Ayarları - burayı kurcalayarak pürüzsüzlüğü bulacak
      public static final double kP_X = 0.03; // İleri-Geri
      public static final double kI_X = 0.0;
      public static final double kD_X = 0.00005;

      public static final double kP_Y = 0.03; // Sağ-Sol (Strafe)
      public static final double kI_Y = 0.0;
      public static final double kD_Y = 0.00005;

      public static final double kP_Theta = 0.04; // Dönüş
      public static final double kI_Theta = 0.0;
      public static final double kD_Theta = 0.00005;

      // Hedef Setpointleri
      public static final double TARGET_TY_SETPOINT = 0.0; // Mesafe için TY hedefi (0 = merkez)
      public static final double TARGET_TX_SETPOINT = 0.0; // Sağ/Sol için TX hedefi (0 = merkez)

      // AprilTag Odometry ve Açı Yönelimleri
      public static final Set<Integer> kZeroDegreeIDs = Set.of(2, 9, 10, 29, 30);
      public static final Set<Integer> k180DegreeIDs = Set.of(13, 14, 25, 26);
  }
}