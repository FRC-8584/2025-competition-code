package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef_V3 extends Command {
    private Swerve swerve;
    private final double x_c;
    private double x_power, turn_power;
    private Supplier<Double> z_power;

    public MoveToReef_V3(Swerve swerve, Reef reef, Supplier<Double> z_power) {
        this.z_power = z_power;
        this.swerve = swerve;
        if(reef == Reef.Right) x_c = -0.18;
        else if(reef == Reef.Left) x_c = 0.16; 
        else x_c = 0;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(LimelightHelpers.getTargetCount(LimelightConstants.device) != 0){
            NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(3);
            double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
            turn_power = Tools.deadband(-pose[4]/25.0, 0.25);
            x_power = -Tools.deadband((x_c - pose[0]) / 1.0, 0.02);
            swerve.drive(-z_power.get() * 0.1, x_power, turn_power, false);
        }
        else {
            swerve.drive(-z_power.get() * 0.1, 0, 0, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
