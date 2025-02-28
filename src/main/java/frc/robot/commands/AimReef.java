package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;

public class AimReef extends Command {
    private Swerve m_swerve;
    private Reef m_reef;

    public AimReef(Swerve swerve, Reef reef) {
        m_swerve = swerve;
        m_reef = reef;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        if(m_reef == Reef.Left) {}
        else {}
        double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight");
        SmartDashboard.putNumber("tx", pose[0]);
        SmartDashboard.putNumber("ty", pose[1]);
    }
}