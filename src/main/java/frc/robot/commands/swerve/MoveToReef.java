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
    private final double y_c, x_c;
    private double y_power,x_power, turn_power;

    /**
     * Move to reef
     * @param swerve swerve subsystem
     * @param reef reef u want to go
     */
    public MoveToReef(Swerve swerve, Reef reef) {
        this.swerve = swerve;
        y_c = reef.getPosY();
        x_c = reef.getPosX();
        LimelightConstants.PIDs.turn_pid.setIZone(0.3);
        LimelightConstants.PIDs.x_pid.setIZone(0.3);
        LimelightConstants.PIDs.y_pid.setIZone(0.3);
    }

    @Override
    public void initialize() {
        LimelightConstants.PIDs.turn_pid.reset();
        LimelightConstants.PIDs.x_pid.reset();
        LimelightConstants.PIDs.y_pid.reset();
    }

    @Override
    public void execute() {
        double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
        if(LimelightHelpers.getTargetCount(LimelightConstants.device) != 0) {
            turn_power = -LimelightConstants.PIDs.turn_pid.calculate(-pose[4]/ LimelightConstants.TurnSlowDownRange) * 0.4; 
            y_power = -LimelightConstants.PIDs.y_pid.calculate((y_c - pose[0])/ LimelightConstants.YSlowDownRange) * 0.3;
            x_power = LimelightConstants.PIDs.x_pid.calculate((x_c - pose[2])/ LimelightConstants.XSlowDownRange) * 0.4;
        }
        else {
            turn_power = 0;
            y_power = 0;
            x_power = 0;
        }
        swerve.drive(x_power , y_power, turn_power, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0 , 0, 0, false);  
    }

    @Override
    public boolean isFinished() {
        if(
            Tools.isInRange(y_power, -0.05, 0.05) &&
            Tools.isInRange(x_power, -0.05, 0.05) &&
            Tools.isInRange(turn_power, -0.05, 
            0.05) && LimelightHelpers.getTargetCount(LimelightConstants.device) != 0
        ) {
            SmartDashboard.putBoolean("Is finished aimming", true);
            return true;
        }
        SmartDashboard.putBoolean("Is finished aimming", false);
        return false;
    }
}
