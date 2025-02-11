package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Tools;

public class Elevator extends SubsystemBase {
  private final SparkMax Lmotor = new SparkMax(Constants.MotorControllerID.Left_ElevatorID, MotorType.kBrushless);
  private final SparkMax Rmotor = new SparkMax(Constants.MotorControllerID.Right_ElevatorID, MotorType.kBrushless);
  
  private double position;// average position (cm)
  private double L_pos, R_pos;// left & right motor position (cm)

  public Elevator() {
    L_pos = ElevatorConstants.kElevatorMinPosition;
    R_pos = ElevatorConstants.kElevatorMinPosition;
    position = ElevatorConstants.kElevatorMinPosition;

    Lmotor.configure(
      Constants.ElevatorConstants.getLeftElevatorCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Rmotor.configure(
      Constants.ElevatorConstants.getRightElevatorCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    L_pos = Lmotor.getEncoder().getPosition();// return left motor position (cm)
    R_pos = Rmotor.getEncoder().getPosition();// return right motor position (cm)
    position = (L_pos + R_pos) / 2.0;// average position (cm)
  }

  /**
   * @param setpoint
   * set elevator position (cm)
   */
  public void setPosition(double setpoint) {
    Lmotor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    Rmotor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
  }

  /**
   * @param input
   * range: 1.0 ~ -1.0 (up ~ down)
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    Lmotor.set(input * Constants.ElevatorConstants.kElevatorSpeed);
    Rmotor.set(input * Constants.ElevatorConstants.kElevatorSpeed);
  }

  /**
   * @return elevator position (cm)
   */
  public double getPosition() {
    return position;
  }
}
