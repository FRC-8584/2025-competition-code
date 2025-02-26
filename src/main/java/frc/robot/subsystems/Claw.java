package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants.Levels;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final SparkMax shaft_motor;
  private final SparkMax grabber_motor;

  private Levels level;

  public Claw() {
    shaft_motor = new SparkMax(CAN_DeviceID.ShaftID, MotorType.kBrushless);
    grabber_motor = new SparkMax(CAN_DeviceID.GrabberID, MotorType.kBrushless);

    shaft_motor.configure(ClawConstants.Configs.getShaftConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void setShaftLevel(Levels level) {
    this.level = level;
  }

  public void setGrabberPower(double power) {
    grabber_motor.set(power);
  }

  public double getPosition() {
    return shaft_motor.getEncoder().getPosition();
  }

  private void setShaftPosition(double angle) {
    shaft_motor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    if(level == Levels.L1) setShaftPosition(ClawConstants.Level_1_Angle);
    if(level == Levels.L2) setShaftPosition(ClawConstants.Level_2_Angle);
    if(level == Levels.L3) setShaftPosition(ClawConstants.Level_3_Angle);
    if(level == Levels.L4) setShaftPosition(ClawConstants.Level_4_Angle);
  }
}
