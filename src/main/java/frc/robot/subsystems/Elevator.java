package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.MotorControllerID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Tools;

public class Elevator extends SubsystemBase {
  private final SparkMax Elevator_Left_motor = new SparkMax(MotorControllerID.Left_ElevatorID, MotorType.kBrushless);
  private final SparkMax Elevator_Right_motor = new SparkMax(MotorControllerID.Right_ElevatorID, MotorType.kBrushless);
  
  private double position;// average position (cm)
  private double setpoint;
  private double L_pos, R_pos;// left & right motor position (cm)

  public Elevator() {
    L_pos = ElevatorConstants.kElevatorMinPosition;
    R_pos = ElevatorConstants.kElevatorMinPosition;
    position = ElevatorConstants.kElevatorMinPosition;
    setpoint = ElevatorConstants.kElevatorMinPosition;

    Elevator_Left_motor.configure(
      ElevatorConstants.getLeftElevatorCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Elevator_Right_motor.configure(
      ElevatorConstants.getRightElevatorCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    L_pos = Elevator_Left_motor.getEncoder().getPosition();// return left motor position (cm)
    R_pos = Elevator_Right_motor.getEncoder().getPosition();// return right motor position (cm)
    position = (L_pos + R_pos) / 2.0;// average position (cm)

    //set position
    Elevator_Left_motor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    Elevator_Right_motor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);

    // update data
    SmartDashboard.putNumber("Elevator L_Pos", L_pos);
    SmartDashboard.putNumber("Elevator R_Pos", R_pos);
    SmartDashboard.putNumber("Elevator Height", position);
    SmartDashboard.putNumber("Elevator setpoint", setpoint);
  }

  /**
   * @param setpoint
   * set elevator position (cm)
   */
  public void setPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * @param input
   * range: 1.0 ~ -1.0 (up ~ down)
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    if (input > 0) {
      Elevator_Left_motor.set(input * ElevatorConstants.kElevatorUpSpeed);
      Elevator_Right_motor.set(input * ElevatorConstants.kElevatorUpSpeed);
    }
    else if (input < 0) {
      Elevator_Left_motor.set(input * ElevatorConstants.kElevatorDownSpeed);
      Elevator_Right_motor.set(input * ElevatorConstants.kElevatorDownSpeed);
    }
    else {
      Elevator_Left_motor.set(0);
      Elevator_Right_motor.set(0);
    }
  }

  /**
   * @return elevator position (cm)
   */
  public double getPosition() {
    return position;
  }

  /**
   * @return elevator setpoint (cm)
   */
  public double getSetpoint() {
    return setpoint;
  }
}
