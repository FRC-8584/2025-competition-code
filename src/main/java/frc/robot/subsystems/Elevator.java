package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.utils.Tools;

public class Elevator extends SubsystemBase {
  private final SparkMax Lmotor = new SparkMax(Constants.MotorControllerID.L_ElevatorID, MotorType.kBrushless);
  private final SparkMax Rmotor = new SparkMax(Constants.MotorControllerID.R_ElevatorID, MotorType.kBrushless);

  private final double min_pos = 15;// cm
  private final double max_pos = 103;// cm
  private final double rotate_rate = 6;// rotations/cm

  private double position;
  private double L_pos, R_pos;

  public Elevator() {
    position = min_pos;
    L_pos = min_pos;
    R_pos = min_pos;
  }

  @Override
  public void periodic() {}

  public void setPosition() {

  }

  /**
   * input > 0 --> up
   * input < 0 --> down
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    Lmotor.set(input * Constants.MotorConstants.kElevatorSpd);
    Rmotor.set(-input * Constants.MotorConstants.kElevatorSpd);
  }

  public double getPosition() {
    return position;
  }
}
