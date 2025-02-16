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
  private final SparkMax Lmotor = new SparkMax(MotorControllerID.Left_ElevatorID, MotorType.kBrushless);
  private final SparkMax Rmotor = new SparkMax(MotorControllerID.Right_ElevatorID, MotorType.kBrushless);
  
  private double position;// average position (cm)
  private double setpoint;
  private double L_pos, R_pos;// left & right motor position (cm)

  public Elevator() {
    L_pos = ElevatorConstants.kElevatorMinPosition;
    R_pos = ElevatorConstants.kElevatorMinPosition;
    setpoint = ElevatorConstants.kElevatorMinPosition;

    Lmotor.configure(
      ElevatorConstants.getLeftElevatorCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Rmotor.configure(
      ElevatorConstants.getRightElevatorCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    L_pos = Lmotor.getEncoder().getPosition();// return left motor position (cm)
    R_pos = Rmotor.getEncoder().getPosition();// return right motor position (cm)
    position = (L_pos + R_pos) / 2.0;// average position (cm)

    //set position
    Lmotor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    Rmotor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);

    // upload data
    SmartDashboard.putNumber("L_Pos", L_pos);
    SmartDashboard.putNumber("R_Pos", R_pos);
    SmartDashboard.putNumber("Elevator Height", position);
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
      Lmotor.set(input * ElevatorConstants.kElevatorUpSpeed);
      Rmotor.set(input * ElevatorConstants.kElevatorUpSpeed);
    }
    else if (input < 0) {
      Lmotor.set(input * ElevatorConstants.kElevatorDownSpeed);
      Rmotor.set(input * ElevatorConstants.kElevatorDownSpeed);
    }
    else {
      Lmotor.set(0);
      Rmotor.set(0);
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
