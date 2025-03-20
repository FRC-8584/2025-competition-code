package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double[] pose;

    /**
     * Move to reef
     * @param swerve swerve subsystem
     * @param reef reef u want to go
     */
    public MoveToReef(Swerve swerve, Reef reef) {
        this.swerve = swerve;
        x_c = reef.getPosY();
        z_c = reef.getPosX();
        LimelightConstants.PIDs.turn_pid.setIZone(0.5);
        LimelightConstants.PIDs.x_pid.setIZone(0.5);
        LimelightConstants.PIDs.y_pid.setIZone(0.5);
    }

    @Override
    public void initialize() {
        LimelightConstants.PIDs.turn_pid.reset();
        LimelightConstants.PIDs.x_pid.reset();
        LimelightConstants.PIDs.y_pid.reset();
    }

    @Override
    public void execute() {
        if(Tools.isInRange(pose[4], -1.0, 1.0)) LimelightConstants.PIDs.turn_pid.reset();
        if(Tools.isInRange(pose[0], -0.01, 0.01)) LimelightConstants.PIDs.y_pid.reset();
        if(Tools.isInRange(pose[1], -0.01, 0.01)) LimelightConstants.PIDs.x_pid.reset();
        if(LimelightHelpers.getTargetCount(LimelightConstants.device) != 0) {
            pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
            turn_power = -LimelightConstants.PIDs.turn_pid.calculate(-pose[4]/ LimelightConstants.TurnSlowDownRange); 
            x_power = -LimelightConstants.PIDs.y_pid.calculate((x_c - pose[0])/ LimelightConstants.YSlowDownRange);
            z_power = -LimelightConstants.PIDs.x_pid.calculate((z_c - pose[2])/ LimelightConstants.XSlowDownRange);
        }
        else {
            turn_power = 0;
            x_power = 0;
            z_power = 0;
        }
        swerve.drive(z_power , x_power, turn_power, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0 , 0, 0, false);  
    }

    @Override
    public boolean isFinished() {
        if(
            Tools.isInRange(pose[4], -1, 1) &&
            Tools.isInRange(pose[0], x_c-0.1, turn_power+0.1) &&
            Tools.isInRange(pose[1], z_c-0.1, turn_power+0.1)
        ) {
            SmartDashboard.putBoolean("IsFinished Aim", true);
            return true;
        }
        SmartDashboard.putBoolean("isFinished Aim", false);
        return false;
    }
}
