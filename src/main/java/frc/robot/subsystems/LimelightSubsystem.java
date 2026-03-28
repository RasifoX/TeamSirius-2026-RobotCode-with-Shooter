package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tid;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry botpose_wpiblue;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable(LimelightConstants.LIMELIGHT_NAME);
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        botpose_wpiblue = table.getEntry("botpose_wpiblue");
    }

    public boolean hasTarget() {
        return tv.getDouble(0) == 1.0;
    }

    public int getTagID() {
        return (int) tid.getInteger(-1);
    }

    public double getTX() {
        return tx.getDouble(0.0);
    }

    public double getTY() {
        return ty.getDouble(0.0);
    }

    public Pose2d getBotPose() {
        double[] botpose = botpose_wpiblue.getDoubleArray(new double[7]);
        if (botpose == null || botpose.length < 6) {
            return null;
        }
        return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
        SmartDashboard.putNumber("Limelight/TX", getTX());
        SmartDashboard.putNumber("Limelight/TY", getTY());
        SmartDashboard.putNumber("Limelight/TargetID", getTagID());
    }
}