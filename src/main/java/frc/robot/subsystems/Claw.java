package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.utils.Tools;

public class Claw extends SubsystemBase {
  private final SparkMax Shaft_motor = new SparkMax(Constants.MotorControllerID.Claw_ShaftID, MotorType.kBrushless);
  private final SparkMax Grapper_motor = new SparkMax(Constants.MotorControllerID.Claw_GrapperID, MotorType.kBrushless);
  
  private double position;// shaft position (deg)

  public Claw() {
    position = ClawConstants.kShaftMinPosition;

    Shaft_motor.configure(
      Constants.ClawConstants.getShaftCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    Grapper_motor.configure(
      Constants.ClawConstants.getGrapperCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    position = Shaft_motor.getEncoder().getPosition();// shaft position (deg)
    SmartDashboard.putNumber("Shaft Angle", position);
  }

  /**
   * @param setpoint
   * set shaft position (deg)
   */
  public void setShaftPosition(double setpoint) {
    Shaft_motor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
  }

  /**
   * @param input
   * run the grapper
   * range: 1.0 ~ -1.0
   */
  public void setGrapperPower(double input) {
    input = Tools.bounding(input);
    Grapper_motor.set(input * Constants.ClawConstants.kGrapperSpeed);
  }

  /**
   * @return shaft position (deg)
   */
  public double getPosition() {
    return position;
  }
}
