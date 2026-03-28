package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor1;
    private final SparkMax shooterMotor2;
    private final SparkMax shooterFeeder3;
    private final SparkMax shooterMotor4;
    private final SparkMax shooterFeeder5;
    private final SparkMax indexerMotor;

    public ShooterSubsystem() {
        shooterMotor1 = new SparkMax(ShooterConstants.ShooterMotor1ID, MotorType.kBrushless);
        shooterMotor2 = new SparkMax(ShooterConstants.ShooterMotor2ID, MotorType.kBrushless);
        shooterFeeder3 = new SparkMax(ShooterConstants.ShooterFeeder3ID, MotorType.kBrushless);
        shooterMotor4 = new SparkMax(ShooterConstants.ShooterMotor4ID, MotorType.kBrushless);
        shooterFeeder5 = new SparkMax(ShooterConstants.ShooterFeeder5ID, MotorType.kBrushless);
        indexerMotor = new SparkMax(ShooterConstants.IndexerMotorID, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
// Eski hatalı kısım yerine bunu yapıştır:
SparkMaxConfig s1Cfg = new SparkMaxConfig();
s1Cfg.inverted(ShooterConstants.ShooterMotor1Inverted);

SparkMaxConfig s2Cfg = new SparkMaxConfig();
s2Cfg.inverted(ShooterConstants.ShooterMotor2Inverted);

SparkMaxConfig s3Cfg = new SparkMaxConfig();
s3Cfg.inverted(ShooterConstants.ShooterFeeder3Inverted);

SparkMaxConfig s4Cfg = new SparkMaxConfig();
s4Cfg.inverted(ShooterConstants.ShooterMotor4Inverted);

SparkMaxConfig s5Cfg = new SparkMaxConfig();
s5Cfg.inverted(ShooterConstants.ShooterFeeder5Inverted);

SparkMaxConfig idxCfg = new SparkMaxConfig();  
idxCfg.inverted(ShooterConstants.IndexerMotorInverted);

        shooterMotor1.configure(s1Cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(s2Cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFeeder3.configure(s3Cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor4.configure(s4Cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFeeder5.configure(s5Cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(idxCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // TEK GİRİŞ - TEK ÇIKIŞ: Hızları tek bir metoddan kontrol et
    public void setShooterIndexerSpeed(double shooterScale, double indexerScale, boolean reverseIndexer) {
        double sSpeed = ShooterConstants.ShooterBaseSpeed * shooterScale;
        double iSpeed = ShooterConstants.IndexerBaseSpeed * indexerScale * ShooterConstants.IndexerDirectionSign;
        
        if (reverseIndexer) {
            iSpeed = -iSpeed;
        }

        shooterMotor1.set(sSpeed);
        shooterMotor2.set(sSpeed);
        shooterFeeder3.set(sSpeed);
        shooterMotor4.set(sSpeed);
        shooterFeeder5.set(sSpeed);
        indexerMotor.set(iSpeed);
    }

    public void runFixed(boolean reverseIndexer) {
        setShooterIndexerSpeed(1.0, 1.0, reverseIndexer);
    }

    public void stop() {
        setShooterIndexerSpeed(0.0, 0.0, false);
    }
}