package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.utils.Tools;

public class Elevator extends SubsystemBase {
  private final SparkMax Lmotor = new SparkMax(Constants.MotorControllerID.L_ElevatorID, MotorType.kBrushless);
  private final SparkMax Rmotor = new SparkMax(Constants.MotorControllerID.R_ElevatorID, MotorType.kBrushless);

  private final double min_pos = 15;// cm
  private final double max_pos = 103;// cm
  private final double init_pos = 15;// cm
  private final double rotate_rate = 6;// rotations/cm

  private double position;
  private double L_pos, R_pos;

  public Elevator() {
    position = init_pos;
    L_pos = init_pos;
    R_pos = init_pos;
  }

  @Override
  public void periodic() {
    L_pos = Lmotor.getEncoder().getPosition()/rotate_rate;
    R_pos = Rmotor.getEncoder().getPosition()/rotate_rate;
    position = (L_pos + R_pos) / 2.0;
  }

  /**
   * @param setpoint
   * set elevator position (cm)
   */
  public void setPosition(double setpoint) {
    setpoint = Tools.bounding(setpoint, min_pos, max_pos);
    Lmotor.getClosedLoopController().setReference((setpoint - init_pos)*rotate_rate, ControlType.kPosition);
    Rmotor.getClosedLoopController().setReference((setpoint - init_pos)*rotate_rate, ControlType.kPosition);
  }

  /**
   * @param input
   * range: 1.0 ~ -1.0 (up ~ down)
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    Lmotor.set(input * Constants.MotorConstants.kElevatorSpd);
    Rmotor.set(input * Constants.MotorConstants.kElevatorSpd);
  }

  /**
   * @return elevator position (cm)
   */
  public double getPosition() {
    return position;
  }
}
