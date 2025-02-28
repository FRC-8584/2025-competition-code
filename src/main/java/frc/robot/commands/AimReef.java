package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;

public class AimReef extends Command {
    private Swerve swerve;
    private Reef reef;

    public AimReef(Swerve swerve, Reef reef) {
        this.swerve = swerve;
        this.reef = reef;
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        if(reef == Reef.Left){}
        else{}
        double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight");
        SmartDashboard.putNumber("tx", pose[0]);
        SmartDashboard.putNumber("ty", pose[1]);
    }
}
