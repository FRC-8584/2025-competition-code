package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.MotorControllerID;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.Tools;

public class Climber extends SubsystemBase {
  private final SparkMax Lmotor = new SparkMax(MotorControllerID.Left_ClimberID, MotorType.kBrushless);
  private final SparkMax Rmotor = new SparkMax(MotorControllerID.Right_ClimberID, MotorType.kBrushless);
  
  private double position;// average position (deg)
  private double L_pos, R_pos;// left & right motor position (deg)

  public Climber() {
    L_pos = ClimberConstants.kClimberMinPosition;
    R_pos = ClimberConstants.kClimberMinPosition;
    position = ClimberConstants.kClimberMinPosition;

    Lmotor.configure(
      ClimberConstants.getLeftClimberCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Rmotor.configure(
      ClimberConstants.getRightClimberCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    L_pos = Lmotor.getEncoder().getPosition();// return left motor position (deg)
    R_pos = Rmotor.getEncoder().getPosition();// return right motor position (deg)
    position = (L_pos + R_pos) / 2.0;// average position (deg)
    SmartDashboard.putNumber("L_Pos", L_pos);
    SmartDashboard.putNumber("R_Pos", R_pos);
    SmartDashboard.putNumber("Elevator Height", position);
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
    if (input > 0) {
      Lmotor.set(input * ClimberConstants.kClimberForwardSpeed);
      Rmotor.set(input * ClimberConstants.kClimberForwardSpeed);
    }
    else if (input < 0) {
      Lmotor.set(input * ClimberConstants.kClimberReverseSpeed);
      Rmotor.set(input * ClimberConstants.kClimberReverseSpeed);
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
}
