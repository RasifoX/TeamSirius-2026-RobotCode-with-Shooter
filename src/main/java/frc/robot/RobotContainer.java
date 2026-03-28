package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTagCMD;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // 1. ALT SİSTEMLER
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // 2. KUMANDA
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // VARSAYILAN SÜRÜŞ KOMUTU
    m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(
        m_robotDrive,
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        () -> true));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Y TUŞU: GYRO SIFIRLAMA
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // A TUŞU: SAVUNMA MODU (X-STANCE)
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.stopModules(), m_robotDrive));

    // X TUŞU: APRIL TAG HİZALAMA (Savaş Modu - Basılı tutunca Limelight devralır)
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new AlignToTagCMD(m_robotDrive, m_limelight));

    // B TUŞU: SHOOTER'I ÇALIŞTIR (Basılı tutunca ateşler)
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_shooter.runFixed(false), m_shooter))
        .onFalse(new RunCommand(() -> m_shooter.stop(), m_shooter));
        
    // Sağ Tampon (Right Bumper): İNDEXER'I TERS ÇALIŞTIR (Sıkışma açmak için)
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_shooter.runFixed(true), m_shooter))
        .onFalse(new RunCommand(() -> m_shooter.stop(), m_shooter));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}