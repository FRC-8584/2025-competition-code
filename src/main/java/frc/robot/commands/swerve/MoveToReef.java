package frc.robot.commands.swerve;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef extends Command {
    private Swerve swerve;
    private final double x_c, z_c;
    private double x_power,z_power, turn_power;

    public MoveToReef(Swerve swerve, Reef reef) {
        this.swerve = swerve;
        x_c = reef.getPosY();
        z_c = reef.getPosX();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(3);
        double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
        turn_power = Tools.deadband(-pose[4]/25.0, 1.0/25.0); 
        x_power = Tools.deadband((x_c - pose[0]) / 1.2, 0.01/1.2);
        z_power = -Tools.deadband((z_c - pose[2]) / 1.2, 0.01/1.2);
        swerve.drive(z_power , x_power, turn_power, false);
    }

    @Override
    public void end(boolean interrupted) {
        NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(1);
    }

    @Override
    public boolean isFinished() {
        if(turn_power == 0 && x_power == 0 && z_power == 0) return true;
        return false;
    }
}
