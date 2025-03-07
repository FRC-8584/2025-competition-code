package frc.robot.commands.claw;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Tools;

public class ControlGrabber extends Command {
    private Claw claw;
    private Supplier<Double> x, y;

    public ControlGrabber(
        Claw claw,
        Supplier<Double> x,
        Supplier<Double> y
    ) {
        this.claw = claw;
        this.x = x;
        this.y = y;
        addRequirements(this.claw);
    }

    @Override
    public void execute() {
        double power = Tools.deadband(x.get()-y.get(), 0.2);
        if(power == 0 &&  !claw.isStuck()) {
            claw.setGrabberPower(0);
            claw.stuckAlgae(true);
        }
        else if(power !=0 && claw.isStuck()){
            claw.stuckAlgae(false);
            claw.setGrabberPower(power);
        }
    }
}
