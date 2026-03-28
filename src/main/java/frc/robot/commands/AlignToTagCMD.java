package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignToTagCMD extends Command {
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private final PIDController xController; // İleri/Geri (TY'den beslenir)
    private final PIDController yController; // Sağ/Sol (TX'ten beslenir)
    private final PIDController thetaController; // Dönüş (Heading'den beslenir)

    public AlignToTagCMD(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        this.xController = new PIDController(LimelightConstants.kP_X, LimelightConstants.kI_X, LimelightConstants.kD_X);
        this.yController = new PIDController(LimelightConstants.kP_Y, LimelightConstants.kI_Y, LimelightConstants.kD_Y);
        this.thetaController = new PIDController(LimelightConstants.kP_Theta, LimelightConstants.kI_Theta, LimelightConstants.kD_Theta);

        this.thetaController.enableContinuousInput(-180, 180);

        addRequirements(driveSubsystem); // Drive sistemini meşgul eder
    }

    @Override
    public void execute() {
        // Hedef yoksa zımba gibi dur
        if (!limelightSubsystem.hasTarget()) {
            driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            return;
        }

        // Verileri Çek
        int id = limelightSubsystem.getTagID();
        double tx = limelightSubsystem.getTX(); 
        double ty = limelightSubsystem.getTY(); 
        double currentHeading = driveSubsystem.getHeading();

        // Hedef Açıyı Belirle
        double targetHeading = currentHeading;
        if (LimelightConstants.k180DegreeIDs.contains(id)) {
            targetHeading = 180;
        } else if (LimelightConstants.kZeroDegreeIDs.contains(id)) {
            targetHeading = 0;
        }

        // PID Hesaplamaları
        double rawXSpeed = xController.calculate(ty, LimelightConstants.TARGET_TY_SETPOINT);
        double rawYSpeed = yController.calculate(tx, LimelightConstants.TARGET_TX_SETPOINT);
        double rawRotSpeed = thetaController.calculate(currentHeading, targetHeading);

        // Paranoyakça Limit (Clamp)
        double clampedXSpeed = MathUtil.clamp(rawXSpeed, -LimelightConstants.maxForwardSpeed, LimelightConstants.maxForwardSpeed);
        double clampedYSpeed = MathUtil.clamp(rawYSpeed, -LimelightConstants.maxStrafeSpeed, LimelightConstants.maxStrafeSpeed);
        double clampedRotSpeed = MathUtil.clamp(rawRotSpeed, -LimelightConstants.maxTurnSpeed, LimelightConstants.maxTurnSpeed);

        // YÖN DÜZELTMELERİ (Koordinat Matrisi)
        // Eğer robot ileri yerine geri, sağ yerine sola gidiyorsa buradaki eksi (-) işaretlerini değiştirt.
        double finalXSpeed = clampedXSpeed;   // Ty pozitifse ileri git (yaklaş)
        double finalYSpeed = -clampedYSpeed;  // Tx pozitifse (hedef sağda) sağa git
        double finalRotSpeed = clampedRotSpeed;

        // Drive (Robot Relative - Sahaya göre değil robota göre hizalama yaparız)
        driveSubsystem.driveRobotRelative(new ChassisSpeeds(finalXSpeed, finalYSpeed, finalRotSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }
}