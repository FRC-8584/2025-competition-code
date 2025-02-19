package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.ShaftConstants;

public class Shaft extends SubsystemBase {
  private final SparkMax Shaft_motor = new SparkMax(CAN_DeviceID.ShaftID, MotorType.kBrushless);
  
  private double position;// shaft position (deg)

  public Shaft() {
    position = ShaftConstants.kShaftMinPosition;

    Shaft_motor.configure(
      ShaftConstants.getShaftCfg(),
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
  public void setPosition(double setpoint) {
    Shaft_motor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
  }

  /**
   * @return shaft position (deg)
   */
  public double getPosition() {
    return position;
  }
}
