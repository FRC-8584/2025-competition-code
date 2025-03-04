package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public class GrabAlgae extends Command {
    private Intake intake;
    private double counter;

    public GrabAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        counter = 0;
    }

    @Override
    public void execute() {
        intake.setGrabberPower(IntakeConstants.GrabPower);
        intake.setShaftPosition(60.0);
        if(intake.getGrabberCurrent() > IntakeConstants.StuckCurrent && intake.isDetected()) counter++;
        if(counter > 30) intake.setState(true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setGrabberPower(0);
        intake.setShaftPosition(0.0);
    }

    @Override
    public boolean isFinished() {
        return intake.isGet();
    }
}
