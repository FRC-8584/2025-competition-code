package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.Tools;

public class Climber extends SubsystemBase {
  private final SparkMax Climber_Left_motor = new SparkMax(CAN_DeviceID.Left_ClimberID, MotorType.kBrushless);
  private final SparkMax Climber_Right_motor = new SparkMax(CAN_DeviceID.Right_ClimberID, MotorType.kBrushless);
  
  private double position;// average position (deg)
  private double L_pos, R_pos;// left & right motor position (deg)

  public Climber() {
    L_pos = ClimberConstants.kClimberMinPosition;
    R_pos = ClimberConstants.kClimberMinPosition;
    position = ClimberConstants.kClimberMinPosition;

    Climber_Left_motor.configure(
      ClimberConstants.getLeftClimberCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Climber_Right_motor.configure(
      ClimberConstants.getRightClimberCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    L_pos = Climber_Left_motor.getEncoder().getPosition();// return left motor position (deg)
    R_pos = Climber_Right_motor.getEncoder().getPosition();// return right motor position (deg)
    position = (L_pos + R_pos) / 2.0;// average position (deg)
    SmartDashboard.putNumber("Climber L_Pos", L_pos);
    SmartDashboard.putNumber("Climber R_Pos", R_pos);
    SmartDashboard.putNumber("Climber Height", position);
  }

  /**
   * @param setpoint
   * set elevator position (cm)
   */
  public void setPosition(double setpoint) {
    Climber_Left_motor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    Climber_Right_motor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
  }

  /**
   * @param input
   * range: 1.0 ~ -1.0 (up ~ down)
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    if (input > 0) {
      Climber_Left_motor.set(input * ClimberConstants.kClimberForwardSpeed);
      Climber_Right_motor.set(input * ClimberConstants.kClimberForwardSpeed);
    }
    else if (input < 0) {
      Climber_Left_motor.set(input * ClimberConstants.kClimberReverseSpeed);
      Climber_Right_motor.set(input * ClimberConstants.kClimberReverseSpeed);
    }
    else {
      Climber_Left_motor.set(0);
      Climber_Right_motor.set(0);
    }
  }

  /**
   * @return elevator position (cm)
   */
  public double getPosition() {
    return position;
  }
}
