package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public class IntakePutAlgae extends Command {
    private Intake intake;

    public IntakePutAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.setShaftPosition(20.0);
    }

    @Override
    public void execute() {
        intake.setGrabberPower(IntakeConstants.PutPower);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setGrabberPower(0);
        intake.setShaftPosition(0.0);
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}
