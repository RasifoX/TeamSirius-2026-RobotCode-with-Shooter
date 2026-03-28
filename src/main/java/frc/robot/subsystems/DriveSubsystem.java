package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * DRIVE SUBSYSTEM — ROBOTUN OMURİLİĞİ
 *
 * 4 SwerveModule + Gyro (Pigeon2 veya NavX) yönetir.
 * Gyro seçimi: Constants.java → DriveConstants.kUseNavX
 */
public class DriveSubsystem extends SubsystemBase {

  // ===========================================================================
  // MODÜLLER
  // ===========================================================================

  // Ön Sol (Front Left)
  private final SwerveModule m_frontLeft = new SwerveModule(
      PortConstants.kFrontLeftDriveID,
      PortConstants.kFrontLeftSteerID,
      PortConstants.kFrontLeftCANCoderID,
      PortConstants.kFrontLeftDriveInverted,
      PortConstants.kFrontLeftSteerInverted,
      Preferences.getDouble("FL_Offset", PortConstants.kFrontLeftOffset));

  // Ön Sağ (Front Right)
  private final SwerveModule m_frontRight = new SwerveModule(
      PortConstants.kFrontRightDriveID,
      PortConstants.kFrontRightSteerID,
      PortConstants.kFrontRightCANCoderID,
      PortConstants.kFrontRightDriveInverted,
      PortConstants.kFrontRightSteerInverted,
      Preferences.getDouble("FR_Offset", PortConstants.kFrontRightOffset));

  // Arka Sol (Rear Left)
  private final SwerveModule m_rearLeft = new SwerveModule(
      PortConstants.kRearLeftDriveID,
      PortConstants.kRearLeftSteerID,
      PortConstants.kRearLeftCANCoderID,
      PortConstants.kRearLeftDriveInverted,
      PortConstants.kRearLeftSteerInverted,
      Preferences.getDouble("RL_Offset", PortConstants.kRearLeftOffset));

  // Arka Sağ (Rear Right)
  private final SwerveModule m_rearRight = new SwerveModule(
      PortConstants.kRearRightDriveID,
      PortConstants.kRearRightSteerID,
      PortConstants.kRearRightCANCoderID,
      PortConstants.kRearRightDriveInverted,
      PortConstants.kRearRightSteerInverted,
      Preferences.getDouble("RR_Offset", PortConstants.kRearRightOffset));

  // ===========================================================================
  // GYRO
  // ===========================================================================

  private final GyroIO m_gyroIO;

  // ===========================================================================
  // ODOMETRY
  // ===========================================================================

  private final SwerveDriveOdometry m_odometry;

  // ===========================================================================
  // AKIMM LİMİT CACHE — KRİTİK FIX
  // Her 20ms'de configure() göndermemek için son değeri sakla.
  // Sadece eşik geçildiğinde (30↔50) CAN frame gönderilir.
  // ===========================================================================
  private int m_lastCurrentLimit = -1;

  // ===========================================================================
  // CONSTRUCTOR
  // ===========================================================================

  public DriveSubsystem() {

    // ── GYRO SEÇİMİ ─────────────────────────────────────────────────────────
    // ✅ Sabit tabanlı seçim: auto-detect yerine tek değişken kontrolü.
    //    NavX bağlıysa → Constants.java → kUseNavX = true yap.
    if (DriveConstants.kUseNavX) {
      m_gyroIO = new NavXIO();
      System.out.println("[GYRO] NavX aktif.");
    } else {
      m_gyroIO = new Pigeon2IO(DriveConstants.kPigeonID);
      System.out.println("[GYRO] Pigeon 2.0 aktif (ID=" + DriveConstants.kPigeonID + ").");
    }

    // ── ODOMETRY ─────────────────────────────────────────────────────────────
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions());

    // ── PATHPLANNER ──────────────────────────────────────────────────────────
    configurePathPlanner();
  }

  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID
              new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);

    } catch (Exception e) {
      System.err.println("[PATHPLANNER] Config yüklenemedi: " + e.getMessage());
      System.err.println("  → PathPlanner GUI'den robot config dosyası oluşturul.");
    }
  }

  // ===========================================================================
  // PERİYODİK DÖNGÜ (50 Hz)
  // ===========================================================================

  @Override
  public void periodic() {
    // Odometry güncelle
    m_odometry.update(getRotation2d(), getModulePositions());

    // Dashboard
    updateDashboard();

    // Dinamik akım koruması: Pil 10.5V altına düşerse drive akımını kıs.
    // FIX: Cache sayesinde configure() sadece eşik değiştiğinde çağrılır,
    //      her 20ms'de değil. CAN bus spam önlendi.
    int currentLimit = RobotController.getBatteryVoltage() < 10.5 ? 30 : 50;
    updateDriveCurrentLimits(currentLimit);
  }

  private void updateDashboard() {
    SmartDashboard.putBoolean("Cockpit/Gyro OK",       m_gyroIO.isConnected());
    SmartDashboard.putBoolean("Cockpit/DriveStick OK", DriverStation.isJoystickConnected(0));
    SmartDashboard.putNumber("Cockpit/Heading deg",    getHeading());
    SmartDashboard.putNumber("Cockpit/Max Temp C",     getMaxModuleTemp());

    // Offset debug: konsol/shuffleboard'da görmek için
    SmartDashboard.putNumber("CANcoder/FL rot", m_frontLeft.getAbsolutePositionRotations());
    SmartDashboard.putNumber("CANcoder/FR rot", m_frontRight.getAbsolutePositionRotations());
    SmartDashboard.putNumber("CANcoder/RL rot", m_rearLeft.getAbsolutePositionRotations());
    SmartDashboard.putNumber("CANcoder/RR rot", m_rearRight.getAbsolutePositionRotations());
  }

  // ===========================================================================
  // ANA SÜRÜŞ METODU
  // ===========================================================================

  /**
   * @param xSpeed        İleri/Geri (m/s). Pozitif = ileri.
   * @param ySpeed        Sol/Sağ (m/s). Pozitif = sol.
   * @param rot           Dönüş (rad/s). Pozitif = CCW (saat tersi).
   * @param fieldRelative Sahaya göre (true) veya robota göre (false) sür.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Hiçbir tekerlek maksimum hızı aşamaz — oranlı yavaşlatma
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** PathPlanner: robot-relative sürüş. */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          false);
  }

  // ===========================================================================
  // X-STANCE (Savunma Modu)
  // ===========================================================================

  /**
   * Tekerlekleri X şekline kilitler — rakip robot itemez.
   * A tuşuna basılı tuttuğu sürece aktif.
   */
  public void stopModules() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
  }

  // ===========================================================================
  // OFSETLERİ KAYDET (Pit Tool)
  // ===========================================================================

  /**
   * Şu anki CANcoder değerlerini offset olarak RoboRIO hafızasına yazar.
   *
   * Kullanım:
   *   1. Tekerlekleri fiziksel olarak ileri hizala (cetvel kullan).
   *   2. Shuffleboard'dan bu metodu tetikle (veya bir tuşa bağla).
   *   3. Deploy gerektirmez — kalıcı olarak RoboRIO'ya yazılır.
   */
  public void saveOffsets() {
    double fl = m_frontLeft.getAbsolutePositionRotations();
    double fr = m_frontRight.getAbsolutePositionRotations();
    double rl = m_rearLeft.getAbsolutePositionRotations();
    double rr = m_rearRight.getAbsolutePositionRotations();

    Preferences.setDouble("FL_Offset", fl);
    Preferences.setDouble("FR_Offset", fr);
    Preferences.setDouble("RL_Offset", rl);
    Preferences.setDouble("RR_Offset", rr);

    System.out.printf(
        "[OFSETLERİ KAYDET] FL=%.4f  FR=%.4f  RL=%.4f  RR=%.4f rot%n", fl, fr, rl, rr);
    System.out.println("  → Bu değerleri Constants.java'daki kXxxOffset sabitlerine de kopyala (kalıcı yedek).");
  }

  // ===========================================================================
  // GYRO
  // ===========================================================================

  public void zeroHeading() {
    m_gyroIO.reset();
  }

  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  public Rotation2d getRotation2d() {
    Rotation2d raw = m_gyroIO.getRotation2d();
    return DriveConstants.kGyroReversed ? raw.unaryMinus() : raw;
  }

  // ===========================================================================
  // ODOMETRY
  // ===========================================================================

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  // ===========================================================================
  // YARDIMCI METODLAR
  // ===========================================================================

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /**
   * Akım limitini günceller.
   * FIX: m_lastCurrentLimit cache sayesinde configure() sadece değer
   *      değiştiğinde gönderilir. Her 20ms'de CAN spam olmaz.
   */
  private void updateDriveCurrentLimits(int amps) {
    if (amps == m_lastCurrentLimit) return;
    m_lastCurrentLimit = amps;
    m_frontLeft.setDriveCurrentLimit(amps);
    m_frontRight.setDriveCurrentLimit(amps);
    m_rearLeft.setDriveCurrentLimit(amps);
    m_rearRight.setDriveCurrentLimit(amps);
    System.out.println("[AKIM LİMİT] " + amps + "A olarak güncellendi.");
  }

  private double getMaxModuleTemp() {
    return Math.max(
        Math.max(m_frontLeft.getTemp(), m_frontRight.getTemp()),
        Math.max(m_rearLeft.getTemp(),  m_rearRight.getTemp()));
  }
}