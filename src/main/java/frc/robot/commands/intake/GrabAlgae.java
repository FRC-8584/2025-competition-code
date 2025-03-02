package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GrabAlgae extends Command {
    private Intake intake;

    public GrabAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        double current = intake.getGrabberCurent();
    }
}
