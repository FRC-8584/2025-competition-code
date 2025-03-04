package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public class PutAlgae extends Command {
    private Intake intake;
    private double counter;

    public PutAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        counter = 0;
        intake.setShaftPosition(20.0);
    }

    @Override
    public void execute() {
        intake.setGrabberPower(IntakeConstants.PutPower);
        if(intake.isDetected()) counter++;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setGrabberPower(0);
        intake.setShaftPosition(0.0);
        intake.setState(false);
    }

    @Override
    public boolean isFinished() {
        if(counter > 20) return true;
        return false;
    }
}
