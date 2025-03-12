package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperationConstant;
import frc.robot.Constants.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef extends Command {
    private Swerve swerve;
    private final double x_c, z_c;
    private double x_power,z_power, turn_power;

    /**
     * Move to reef
     * @param swerve swerve subsystem
     * @param reef reef u want to go
     */
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
        if(LimelightHelpers.getTargetCount(LimelightConstants.device) != 0) {
            double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
            turn_power = Tools.deadband(-pose[4]/50.0, 0.1) * 0.5 / OperationConstant.DriveSpeed; 
            x_power = Tools.deadband((x_c - pose[0]) / 0.8, 0.1)* 0.5 / OperationConstant.DriveSpeed;
            z_power = -Tools.deadband((z_c - pose[2]) / 0.8, 0.1)* 0.5 / OperationConstant.DriveSpeed;
            swerve.drive(z_power , x_power, turn_power, false);
        }
        else {
            swerve.drive(0 , 0, 0, false);   
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0 , 0, 0, false);  
    }

    @Override
    public boolean isFinished() {
        if(turn_power == 0 && x_power == 0 && z_power == 0) return true;
        return false;
    }
}
