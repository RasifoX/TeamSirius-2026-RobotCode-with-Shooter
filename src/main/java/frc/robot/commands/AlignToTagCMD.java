package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * AlignToTagCMD — HUB AprilTag Hizalama
 *
 * Basılı tutulduğu sürece Limelight'tan TX ve TY alarak robotu
 * Hub'a hizalar. Heading lock YAPILMAZ — Hub sahada sabit olduğu için
 * sadece TX=0, TY=setpoint yeterlidir.
 *
 * Koordinat mantığı:
 *   TX pozitif → hedef robotun sağında → robotu sağa (negatif Y) kayar
 *   TY pozitif → hedef yukarıda (robot yakın) → robot geri çekilir
 *   TY negatif → hedef aşağıda (robot uzak) → robot ileri gider
 */
public class AlignToTagCMD extends Command {

    private final DriveSubsystem     m_drive;
    private final LimelightSubsystem m_limelight;

    private final PIDController m_xController; // İleri/Geri — TY'den beslenir
    private final PIDController m_yController; // Sağ/Sol   — TX'ten beslenir

    public AlignToTagCMD(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.m_drive     = drive;
        this.m_limelight = limelight;

        m_xController = new PIDController(
            LimelightConstants.kP_X,
            LimelightConstants.kI_X,
            LimelightConstants.kD_X);

        m_yController = new PIDController(
            LimelightConstants.kP_Y,
            LimelightConstants.kI_Y,
            LimelightConstants.kD_Y);

        // Heading PID kaldırıldı — Hub için heading lock gerekmez.
        // Hub sahada sabit, TX+TY hizalaması yeterli.

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_yController.reset();
    }

    @Override
    public void execute() {
        // Hedef görünmüyorsa dur
        if (!m_limelight.hasTarget()) {
            m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Align/Durum", "Hedef YOK");
            return;
        }

        int    id = m_limelight.getTagID();
        double tx = m_limelight.getTX();
        double ty = m_limelight.getTY();

        // Sadece Hub tag'lerine tepki ver
        // Yanlış tag'e kilitlenmemek için kontrol et
        if (!LimelightConstants.kHubIDs.contains(id) && id != -1) {
            // -1 = ID henüz okunmadı, o durumda yine de hizalamayı dene
            // Bilinen başka element tag'i ise dur
            m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Align/Durum", "Hub Tag Degil: " + id);
            return;
        }

        // PID hesaplamaları
        // xController: TY → TARGET_TY_SETPOINT'e götür (ileri/geri)
        // yController: TX → 0'a götür (sağ/sol)
        double rawX = m_xController.calculate(ty, LimelightConstants.TARGET_TY_SETPOINT);
        double rawY = m_yController.calculate(tx, LimelightConstants.TARGET_TX_SETPOINT);

        // Hız limitleme
        double xSpeed   = MathUtil.clamp(rawX, -LimelightConstants.maxForwardSpeed, LimelightConstants.maxForwardSpeed);
        double ySpeed   = MathUtil.clamp(rawY, -LimelightConstants.maxStrafeSpeed,  LimelightConstants.maxStrafeSpeed);

        // Yön düzeltmesi:
        //   TY > setpoint → robot yakın → geri çekil (negatif X)
        //   TX > 0        → hedef sağda → sola kayarak ortala (negatif Y)
        // Eğer robot ters yönde gidiyorsa bu işaretleri değiştir.
        double finalX = -xSpeed; // TY azaldıkça (hedef uzaklaştı) ileri git
        double finalY = -ySpeed; // TX pozitifken (hedef sağda) sola kayarak ortala

        m_drive.driveRobotRelative(new ChassisSpeeds(finalX, finalY, 0)); // Heading yok

        // Dashboard debug
        SmartDashboard.putString("Align/Durum",  "HİZALANIYOR");
        SmartDashboard.putNumber("Align/TagID",  id);
        SmartDashboard.putNumber("Align/TX",     tx);
        SmartDashboard.putNumber("Align/TY",     ty);
        SmartDashboard.putNumber("Align/xHiz",   finalX);
        SmartDashboard.putNumber("Align/yHiz",   finalY);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("Align/Durum", "Durdu");
    }

    @Override
    public boolean isFinished() {
        // Komut kendiliğinden bitmez, tuş bırakılınca biter (whileTrue)
        return false;
    }
}