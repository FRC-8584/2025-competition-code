package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeGrabAlgae extends Command {
    private Intake intake;
    private double counter;

    public IntakeGrabAlgae(Intake intake) {
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
        intake.setShaftPosition(52.0);
        if(intake.isDetected() && intake.getGrabberCurrent() > IntakeConstants.StuckCurrent) counter ++;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setShaftPosition(0);
        intake.setGrabberPower(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println(counter);
        if(counter > 15) return true;
        return false;
    }
}
