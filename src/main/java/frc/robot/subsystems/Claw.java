package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.utils.Tools;

public class Claw extends SubsystemBase {
  private final SparkMax Shaft_motor = new SparkMax(Constants.MotorControllerID.Claw_ShaftID, MotorType.kBrushless);
  private final SparkMax Grapper_motor = new SparkMax(Constants.MotorControllerID.Claw_GrapperID, MotorType.kBrushless);
  
  private double position;// shaft position (deg)
  private final double min_pos = 0;// minimum position (deg)
  private final double max_pos = 135;// maximum position (deg)
  private final double init_pos = 0;// initial position (deg)

  public Claw() {
    position = init_pos;

    Shaft_motor.configure(
      Constants.MotorControllerCfg.getClawShaftCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Grapper_motor.configure(
      Constants.MotorControllerCfg.getClawGrapperCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    position = init_pos + Shaft_motor.getEncoder().getPosition();// shaft position (deg)
  }

  /**
   * @param setpoint
   * set shaft position (deg)
   */
  public void setShaftPosition(double setpoint) {
    setpoint = Tools.bounding(setpoint, min_pos, max_pos);
    Shaft_motor.getClosedLoopController().setReference(setpoint - init_pos, ControlType.kPosition);
  }

  /**
   * @param input
   * run the grapper
   * range: 1.0 ~ -1.0
   */
  public void setGrapperPower(double input) {
    input = Tools.bounding(input);
    Grapper_motor.set(input * Constants.MotorConstants.kGrapperSpd);
  }

  /**
   * @return shaft position (deg)
   */
  public double getPosition() {
    return position;
  }
}
