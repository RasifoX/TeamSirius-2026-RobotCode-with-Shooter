package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTagCMD;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

/**
 * ROBOT CONTAINER — REBUILT 2026
 *
 * Shooter Modları (SmartDashboard "Shooter Modu" seçicisinden seç):
 *
 *   🔒 SABİT HIZ   — Limelight olmadan, sabit hızla atar.
 *                    X tuşu hiçbir şey yapmaz.
 *
 *   📍 STATİK      — X tuşu Limelight ile Hub'a hizalar.
 *                    B tuşu hizalandıktan sonra sabit hızla atar.
 *
 *   🎯 DİNAMİK     — X tuşu Limelight ile Hub'a hizalar.
 *                    B tuşu TY'den mesafeyi hesaplayıp hız ayarlar.
 *
 * Tuş Haritası:
 *   Y  → Gyro sıfırla (InstantCommand — bir kere)
 *   A  → X-Stance (savunma modu, basılı tut)
 *   X  → AprilTag hizalama (STATIC/DYNAMIC modda aktif, basılı tut)
 *   B  → Ateş et (basılı tut)
 *   RB → Indexer ters (sıkışma açma, basılı tut)
 */
public class RobotContainer {

  // ── Shooter Modu Enum ─────────────────────────────────────────
  public enum ShooterMode {
    FIXED,    // Limelight yok, sabit hız
    STATIC,   // Limelight ile hizala, sabit hız
    DYNAMIC   // Limelight ile hizala, mesafeye göre dinamik hız
  }

  // ── Alt Sistemler ─────────────────────────────────────────────
  private final DriveSubsystem     m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limelight  = new LimelightSubsystem();
  private final ShooterSubsystem   m_shooter    = new ShooterSubsystem();

  // ── Kumanda ───────────────────────────────────────────────────
  private final XboxController m_controller =
      new XboxController(OIConstants.kDriverControllerPort);

  // ── Seçiciler ─────────────────────────────────────────────────
  private final SendableChooser<Command>     m_autoChooser;
  private final SendableChooser<ShooterMode> m_shooterModeChooser;

  // ── AlignToTagCMD referansı ───────────────────────────────────
  // Tek instance — defalarca yeni nesne oluşturmaktan kaçınır
  private final AlignToTagCMD m_alignCmd =
      new AlignToTagCMD(m_robotDrive, m_limelight);

  // =============================================================================
  public RobotContainer() {

    // Varsayılan sürüş komutu
    m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(
        m_robotDrive,
        () -> -m_controller.getLeftY(),
        () -> -m_controller.getLeftX(),
        () -> -m_controller.getRightX(),
        () -> true));

    // Otonom seçici
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Shooter mod seçici
    m_shooterModeChooser = new SendableChooser<>();
    m_shooterModeChooser.setDefaultOption("🔒 SABİT HIZ (Limelight yok)", ShooterMode.FIXED);
    m_shooterModeChooser.addOption("📍 STATİK (Limelight + sabit hız)", ShooterMode.STATIC);
    m_shooterModeChooser.addOption("🎯 DİNAMİK (Limelight + mesafe hesabı)", ShooterMode.DYNAMIC);
    SmartDashboard.putData("Shooter Modu", m_shooterModeChooser);

    configureBindings();
  }

  // =============================================================================
  private void configureBindings() {

    // ── Y: Gyro Sıfırla ─────────────────────────────────────────
    new JoystickButton(m_controller, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ── A: X-Stance (Savunma) ────────────────────────────────────
    new JoystickButton(m_controller, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.stopModules(), m_robotDrive));

    // ── X: AprilTag Hizalama ─────────────────────────────────────
    // FIX: Eski pattern (RunCommand içinden schedule()) deprecated ve her
    //      20ms tetiklendiği için hatalıydı.
    //
    // ConditionalCommand buton basıldığı anda modu bir kez kontrol eder:
    //   - STATIC veya DYNAMIC → m_alignCmd çalışır
    //   - FIXED               → InstantCommand (hiçbir şey yapmaz)
    //
    // whileTrue sayesinde buton bırakılınca WPILib otomatik cancel eder.
    // Ayrıca onFalse'a gerek kalmaz.
    new JoystickButton(m_controller, XboxController.Button.kX.value)
        .whileTrue(new ConditionalCommand(
            m_alignCmd,
            new InstantCommand(),
            () -> m_shooterModeChooser.getSelected() != ShooterMode.FIXED
        ));

    // ── B: Ateş Et ───────────────────────────────────────────────
    new JoystickButton(m_controller, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> {
          ShooterMode mode = m_shooterModeChooser.getSelected();
          switch (mode) {
            case FIXED:
            case STATIC:
              m_shooter.runFixed(false);
              break;
            case DYNAMIC:
              m_shooter.runFromTY(m_limelight.getTY(), m_limelight.hasTarget());
              break;
          }
        }, m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stop(), m_shooter));

    // ── RB: Indexer Ters (Sıkışma Açma) ─────────────────────────
    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_shooter.runFixed(true), m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stop(), m_shooter));
  }

  // =============================================================================
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}