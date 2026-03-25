package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

/**
 * SWERVE MODULE - ROBOTUN KAS HÜCRESİ
 *
 * Tek bir tekerlek modülünü temsil eder.
 * 2 Motor (Drive + Steer) + CANcoder içerir.
 *
 * ─── OFFSET MATEMATİĞİ ───────────────────────────────────────────────────────
 *  CANcoder → Phoenix 6 → getAbsolutePosition() → ROTASYON birimi (-0.5 … 0.5)
 *
 *  resetToAbsolute() formülü:
 *    encoderRad = (cancoderRotations - offsetRotations) * 2π
 *
 *  Bu formül, tekerlek fiziksel olarak ileri bakarken encoder'ı 0 rad'a ayarlar.
 *  Sonuç [-π, π] aralığına normalize edilir → wrapping range ile uyumlu.
 * ─────────────────────────────────────────────────────────────────────────────
 */
public class SwerveModule {

  // Donanım
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder            m_driveEncoder;
  private final RelativeEncoder            m_turningEncoder;
  private final SparkClosedLoopController  m_turningPIDController;

  private final CANcoder m_turningCanCoder;

  // Offset: Tuner X'ten okunan ROTASYON değeri (0.0 – 1.0 arası, negatif olabilir)
  private final double m_chassisAngularOffsetRotations;

  /**
   * @param driveMotorId    Sürüş Motoru CAN ID
   * @param turningMotorId  Dönüş Motoru CAN ID
   * @param canCoderId      CANcoder CAN ID
   * @param driveInverted   Sürüş motoru ters mi? (MK4i'de sağ modüller genellikle true)
   * @param steerInverted   Dönüş motoru ters mi? (genellikle false)
   * @param offsetRotations Tekerlek ileri bakarkenn CANcoder'ın rotasyon değeri
   *                        (Tuner X'ten direkt oku, eksi yapma, dereceye çevirme)
   */
  @SuppressWarnings("removal")
  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      int canCoderId,
      boolean driveInverted,
      boolean steerInverted,
      double offsetRotations) {

    m_chassisAngularOffsetRotations = offsetRotations;

    // ── Motorlar ──────────────────────────────────────────────────────────────
    m_driveMotor   = new SparkMax(driveMotorId,   MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

    // ── CANcoder ──────────────────────────────────────────────────────────────
    m_turningCanCoder = new CANcoder(canCoderId);

    // =========================================================================
    // SÜRÜŞ MOTORU KONFİGÜRASYONU
    // =========================================================================
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.inverted(driveInverted);
    driveConfig.voltageCompensation(ModuleConstants.kNominalVoltage);
    driveConfig.smartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

    // Motor devrini metreye çevir
    driveConfig.encoder.positionConversionFactor(ModuleConstants.kDrivePositionFactor); // m/rot
    driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveVelocityFactor); // m/s per RPM

    // PID kapalı döngü için hız kontrolü (PathPlanner bunu kullanabilir)
    driveConfig.closedLoop.pid(0.04, 0.0, 0.0);         // Hız PID — ileride SysId ile tunelanacak
    driveConfig.closedLoop.velocityFF(1.0 / (DriveConstants.kMaxSpeedMetersPerSecond * 60.0 /
        ModuleConstants.kDrivePositionFactor)); // Basit feedforward

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // =========================================================================
    // DÖNÜŞ MOTORU KONFİGÜRASYONU
    // =========================================================================
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.inverted(steerInverted);
    turnConfig.voltageCompensation(ModuleConstants.kNominalVoltage);
    turnConfig.smartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

    // Motor devrini radyana çevir
    turnConfig.encoder.positionConversionFactor(ModuleConstants.kSteerPositionFactor); // rad/rot
    turnConfig.encoder.velocityConversionFactor(ModuleConstants.kSteerVelocityFactor); // rad/s per RPM

    // PID
    turnConfig.closedLoop.pid(
        ModuleConstants.kTurningP,
        ModuleConstants.kTurningI,
        ModuleConstants.kTurningD);

    // ✅ KRİTİK: Wrapping [-π, +π] — WPILib'in Rotation2d aralığıyla TAM UYUMLU.
    //    Eski [0, 2π] aralığı negatif hedeflerde (strafe sağ = -π/2) hatalı
    //    sonuç üretiyordu: motor kısa yol yerine 270° dönüyordu.
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);

    m_turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ── Nesne referansları ────────────────────────────────────────────────────
    m_driveEncoder        = m_driveMotor.getEncoder();
    m_turningEncoder      = m_turningMotor.getEncoder();
    m_turningPIDController = m_turningMotor.getClosedLoopController();

    // ── İlk senkronizasyon ────────────────────────────────────────────────────
    // CANcoder'ın hazır olmasını kısa süre bekle (CAN bus gecikmesi)
    try { Thread.sleep(250); } catch (InterruptedException ignored) {}
    resetToAbsolute();
  }

  // ===========================================================================
  // MUTLAK ENCODER SENKRONİZASYONU
  // ===========================================================================

  /**
   * SparkMax encoder'ını CANcoder'la senkronize eder.
   * Robot her açıldığında çalışır.
   *
   * Formül: encoderRad = (cancoderRotations - offsetRotations) × 2π
   *
   * Sonuç [-π, π] aralığına normalize edilir (wrapping range ile uyumlu).
   * Tekerlek ileri bakıyor iken encoder = 0 rad olur.
   */
  public void resetToAbsolute() {
    // Phoenix 6: getAbsolutePosition() → ROTASYON (-0.5 … 0.5)
    double absoluteRotations = m_turningCanCoder.getAbsolutePosition().getValueAsDouble();

    // Offset'i çıkar → tekerlek "ileri" iken sonuç 0 rotasyon olur
    double positionRotations = absoluteRotations - m_chassisAngularOffsetRotations;

    // Radyana çevir
    double positionRad = positionRotations * 2.0 * Math.PI;

    // [-π, π] aralığına normalize et (wrapping range ile uyumlu)
    positionRad = normalizeAngle(positionRad);

    m_turningEncoder.setPosition(positionRad);
  }

  /** Açıyı [-π, π] aralığına getirir. */
  private static double normalizeAngle(double angleRad) {
    while (angleRad >  Math.PI) angleRad -= 2.0 * Math.PI;
    while (angleRad < -Math.PI) angleRad += 2.0 * Math.PI;
    return angleRad;
  }

  // ===========================================================================
  // MODÜLE EMİR VERME
  // ===========================================================================

  /**
   * Hedeflenen hız ve açıyı ayarla.
   *
   * @param desiredState WPILib'in hesapladığı hedef hız (m/s) ve açı (Rotation2d)
   */
  @SuppressWarnings("removal")
  public void setDesiredState(SwerveModuleState desiredState) {
    // Mevcut açı
    Rotation2d currentAngle = new Rotation2d(m_turningEncoder.getPosition());

    // OPTİMİZASYON: Tekerlek hedefine ≤90° ile ulaşabiliyorsa, uzun yola gitme.
    // Eğer >90° dönmesi gerekiyorsa, motoru ters çevir + 180° döndür.
    desiredState.optimize(currentAngle);

    // Steer PID: hedef açıyı radyan olarak ver
    // desiredState.angle.getRadians() her zaman [-π, π] aralığındadır → wrapping ile uyumlu ✓
    m_turningPIDController.setReference(
        desiredState.angle.getRadians(),
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0);

    // Drive motoru: open-loop hız kontrolü
    // Normalize: [-1, 1] aralığına çevir
    m_driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
  }

  // ===========================================================================
  // DURUM / POZİSYON OKUMA
  // ===========================================================================

  /** Odometry için modül pozisyonu (toplam mesafe + açı). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /** Telemetri için modül durumu (anlık hız + açı). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * CANcoder'ın ham (offset uygulanmamış) mutlak pozisyonu.
   * Dönüş: ROTASYON (-0.5 … 0.5) — offset kalibrasyon için kullanılır.
   */
  public double getAbsolutePositionRotations() {
    return m_turningCanCoder.getAbsolutePosition().getValueAsDouble();
  }

  // ===========================================================================
  // YARDIMCI
  // ===========================================================================

  /** Drive motorunun anlık sıcaklığı (°C). */
  public double getTemp() {
    return m_driveMotor.getMotorTemperature();
  }

  /** Drive akım limitini dinamik olarak günceller (brownout koruması). */
  public void setDriveCurrentLimit(int amps) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(amps);
    m_driveMotor.configure(config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  /** Acil durdurma. */
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
}