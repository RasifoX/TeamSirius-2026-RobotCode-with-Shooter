package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LimelightConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SHOOTER SUBSYSTEM — REBUILT 2026
 *
 * 5 shooter motoru + 1 indexer motoru yönetir.
 * REV SparkMax (Brushless) — REVLib 2026 API kullanır.
 *
 * Üç çalışma modu:
 *   runFixed()    → Sabit hız (ShooterBaseSpeed * 1.0)
 *   runFromTY()   → TY'den mesafe hesaplayıp dinamik hız
 *   stop()        → Tüm motorlar dur
 */
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor1;
    private final SparkMax shooterMotor2;
    private final SparkMax shooterFeeder3;
    private final SparkMax shooterMotor4;
    private final SparkMax shooterFeeder5;
    private final SparkMax indexerMotor;

    // Dashboard debug için son değerler
    private double m_lastShooterSpeed    = 0.0;
    private double m_lastCalcDistanceM   = 0.0;
    private String m_lastMod             = "Durdu";

    public ShooterSubsystem() {
        shooterMotor1  = new SparkMax(ShooterConstants.ShooterMotor1ID,  MotorType.kBrushless);
        shooterMotor2  = new SparkMax(ShooterConstants.ShooterMotor2ID,  MotorType.kBrushless);
        shooterFeeder3 = new SparkMax(ShooterConstants.ShooterFeeder3ID, MotorType.kBrushless);
        shooterMotor4  = new SparkMax(ShooterConstants.ShooterMotor4ID,  MotorType.kBrushless);
        shooterFeeder5 = new SparkMax(ShooterConstants.ShooterFeeder5ID, MotorType.kBrushless);
        indexerMotor   = new SparkMax(ShooterConstants.IndexerMotorID,   MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig s1  = new SparkMaxConfig(); s1.inverted(ShooterConstants.ShooterMotor1Inverted);
        SparkMaxConfig s2  = new SparkMaxConfig(); s2.inverted(ShooterConstants.ShooterMotor2Inverted);
        SparkMaxConfig s3  = new SparkMaxConfig(); s3.inverted(ShooterConstants.ShooterFeeder3Inverted);
        SparkMaxConfig s4  = new SparkMaxConfig(); s4.inverted(ShooterConstants.ShooterMotor4Inverted);
        SparkMaxConfig s5  = new SparkMaxConfig(); s5.inverted(ShooterConstants.ShooterFeeder5Inverted);
        SparkMaxConfig idx = new SparkMaxConfig(); idx.inverted(ShooterConstants.IndexerMotorInverted);

        shooterMotor1 .configure(s1,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2 .configure(s2,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFeeder3.configure(s3,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor4 .configure(s4,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFeeder5.configure(s5,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor  .configure(idx, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =========================================================================
    // TEMEL HIZ KONTROLÜ
    // =========================================================================

    /**
     * Tüm motorları ayarla.
     * @param shooterScale Shooter hız çarpanı (0.0–1.0)
     * @param indexerScale Indexer hız çarpanı (0.0–1.0)
     * @param reverseIndexer true → indexer ters döner (sıkışma açma)
     */
    public void setShooterIndexerSpeed(double shooterScale, double indexerScale, boolean reverseIndexer) {
        double sSpeed = ShooterConstants.ShooterBaseSpeed * shooterScale;
        double iSpeed = ShooterConstants.IndexerBaseSpeed * indexerScale
                        * ShooterConstants.IndexerDirectionSign;

        if (reverseIndexer) {
            iSpeed = -iSpeed;
        }

        shooterMotor1 .set(sSpeed);
        shooterMotor2 .set(sSpeed);
        shooterFeeder3.set(sSpeed);
        shooterMotor4 .set(sSpeed);
        shooterFeeder5.set(sSpeed);
        indexerMotor  .set(iSpeed);

        m_lastShooterSpeed = sSpeed;
    }

    // =========================================================================
    // MOD 1 & 2: SABİT HIZ
    // =========================================================================

    /**
     * Sabit hızla çalıştır.
     * Constants'taki ShooterBaseSpeed * 1.0 = tam hız.
     * @param reverseIndexer true → indexer ters (sıkışma açma)
     */
    public void runFixed(boolean reverseIndexer) {
        m_lastMod = reverseIndexer ? "IndexerTers" : "Sabit";
        setShooterIndexerSpeed(1.0, 1.0, reverseIndexer);
    }

    // =========================================================================
    // MOD 3: DİNAMİK HIZ (TY'den Mesafe Hesabı)
    // =========================================================================

    /**
     * Limelight TY açısından mesafeyi hesaplar, uygun hızı seçer.
     *
     * Formül: d = (h_target - h_limelight) / tan(mountAngle + ty)
     *
     * @param ty        Limelight'tan gelen dikey açı (derece)
     * @param hasTarget Limelight geçerli hedef görüyor mu?
     */
    public void runFromTY(double ty, boolean hasTarget) {
        if (!hasTarget) {
            // Hedef yok → güvenli fallback hız
            m_lastMod           = "Dinamik-HedefYok";
            m_lastCalcDistanceM = -1.0;
            setShooterIndexerSpeed(ShooterConstants.ShooterFallbackScale, 1.0, false);
            return;
        }

        // ── Mesafe Hesabı ─────────────────────────────────────────
        // d = (h_target - h_limelight) / tan(mountAngle + ty)
        //
        // kLimelightMountAngleDeg ve kLimelightHeightMeters'ı
        // robotunuzu kumpasla ölçerek Constants'a girin!
        double mountAngleDeg    = LimelightConstants.kLimelightMountAngleDeg;
        double limelightHeightM = LimelightConstants.kLimelightHeightMeters;
        double targetHeightM    = LimelightConstants.kTargetHeightMeters; // 1.1240m (resmi)

        double angleTotalRad = Math.toRadians(mountAngleDeg + ty);
        double distanceM     = (targetHeightM - limelightHeightM) / Math.tan(angleTotalRad);
        m_lastCalcDistanceM  = distanceM;

        // ── Hız Tablosu ───────────────────────────────────────────
        // Practice'de her mesafe aralığında ateş edip
        // topun hedefe girip girmediğine bakarak tune et.
        // SmartDashboard → "Shooter/Hesap Mesafe (m)" değerine bak.
        double scale;
        if (distanceM < ShooterConstants.kDistanceClose) {
            scale = ShooterConstants.kSpeedClose;
        } else if (distanceM < ShooterConstants.kDistanceMid) {
            scale = ShooterConstants.kSpeedMid;
        } else if (distanceM < ShooterConstants.kDistanceFar) {
            scale = ShooterConstants.kSpeedFar;
        } else {
            scale = ShooterConstants.kSpeedMax;
        }

        m_lastMod = String.format("Dinamik d=%.2fm s=%.0f%%", distanceM, scale * 100);
        setShooterIndexerSpeed(scale, 1.0, false);
    }

    // =========================================================================
    // DURDUR
    // =========================================================================

    public void stop() {
        m_lastMod = "Durdu";
        setShooterIndexerSpeed(0.0, 0.0, false);
    }

    // =========================================================================
    // DASHBOARD
    // =========================================================================

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Son Hiz",          m_lastShooterSpeed);
        SmartDashboard.putNumber("Shooter/Hesap Mesafe (m)", m_lastCalcDistanceM);
        SmartDashboard.putString("Shooter/Mod",              m_lastMod);
    }
}