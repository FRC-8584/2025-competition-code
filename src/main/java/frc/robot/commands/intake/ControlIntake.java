package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Tools;

public class ControlIntake extends Command {
    private Intake intake;
    private Supplier<Double> x, y;
    
    public ControlIntake(Intake intake, Supplier<Double> x, Supplier<Double> y) {
        this.intake = intake;
        this.y = y;
        this.x = x;
        addRequirements(this.intake);
    }
    
    @Override
    public void execute() {
        double power = Tools.deadband(x.get() - y.get(), 0.2);
        if(power == 0) {
            
        }
    }
}
