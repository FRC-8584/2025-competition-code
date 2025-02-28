package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class SwerveAimReef extends Command {
    private Swerve swerve;
    private Reef reef;

    private double tag_x;
    private double tag_y;
    private double tag_angle;

    public SwerveAimReef(Swerve swerve, Reef reef) {
        this.swerve = swerve;
        this.reef = reef;
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        double[] pose = LimelightHelpers.getTargetPose_CameraSpace("limelight");
        tag_x = pose[0] + LimelightConstants.X;
        tag_y = pose[2] + LimelightConstants.Y;
        tag_angle = Math.atan2(tag_y, tag_x) * 180.0 / Math.PI;
        // x
        double x, y, turn;
        if(reef == Reef.Left) {
            if(Tools.isInRange(tag_x, -0.16, -0.14)) x = 0;
            else x = ((tag_x > -0.15) ? -0.1 : 0.1) * SwerveConstants.MaxDriveSpeed; 
        }
        else {
            if(Tools.isInRange(tag_x, 0.14, 0.16)) x = 0;
            else x = ((tag_x > 0.15) ? -0.1 : 0.1) * SwerveConstants.MaxDriveSpeed;
        }
        // y
        if(Tools.isInRange(tag_y, 0.19, 0.21)) y = 0;
        else y =  ((tag_y > 0.2) ? -0.1 : 0.1) * SwerveConstants.MaxDriveSpeed;
        // turn
        if (tag_angle > 90) tag_angle-= 90;
        else  tag_angle = 90-tag_angle;
        if(Tools.isInRange(tag_angle, -3.0, 3.0)) turn = 0;
        else turn = ((tag_angle > 0) ? -0.1 : 0.1) * SwerveConstants.MaxTurnSpeed;
        swerve.drive(x, y, turn, false);
    }

    @Override
    public boolean isFinished() {
        return (Tools.isInRange(tag_x, -0.16, -0.14) && Tools.isInRange(tag_y, 0.19, 0.21) && Tools.isInRange(tag_angle, -3.0, 3.0))? true: false;
    } 
}
