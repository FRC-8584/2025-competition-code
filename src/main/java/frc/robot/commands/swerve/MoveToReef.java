package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
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
    private final PIDController x_pid, z_pid, t_pid;

    /**
     * Move to reef
     * @param swerve swerve subsystem
     * @param reef reef u want to go
     */
    public MoveToReef(Swerve swerve, Reef reef) {
        this.swerve = swerve;
        x_c = reef.getPosY();
        z_c = reef.getPosX();
        x_pid = new PIDController(0.1, 0, 1e-6);
        z_pid = new PIDController(0.1, 0, 1e-6);
        t_pid = new PIDController(0.1, 0, 1e-6);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(3);
        if(LimelightHelpers.getTargetCount(LimelightConstants.device) != 0) {
            double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
            turn_power = Tools.deadband(-pose[4]/70.0, 0.1); 
            x_power = Tools.deadband((x_c - pose[0]) / 0.8, 0.1);
            z_power = -Tools.deadband((z_c - pose[2]) / 0.8, 0.1);
            swerve.drive(z_power , x_power, turn_power, false);
        }
        // if(LimelightHelpers.getTargetCount(LimelightConstants.device) != 0) {
        //     double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
        //     turn_power = Tools.deadband(t_pid.calculate(-pose[4]), 0.1); 
        //     x_power = Tools.deadband(x_pid.calculate(x_c - pose[0]), 0.1);
        //     z_power = -Tools.deadband(z_pid.calculate(z_c - pose[2]), 0.1);
        //     swerve.drive(z_power , x_power, turn_power, false);
        // }
        else {
            swerve.drive(0 , 0, 0, false);   
        }
        System.out.println(z_power+", "+x_power+", "+turn_power);
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
