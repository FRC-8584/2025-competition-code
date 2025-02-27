package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants.Levels;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final SparkMax shaft_motor;
  private final SparkMax grabber_motor;
  private final AnalogInput sensor;
  private Levels level;

  public Claw() {
    shaft_motor = new SparkMax(CAN_DeviceID.ShaftID, MotorType.kBrushless);
    grabber_motor = new SparkMax(CAN_DeviceID.GrabberID, MotorType.kBrushless);
    sensor = new AnalogInput(ClawConstants.SensorPort);

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

  public boolean isGet() {
    if (sensor.getValue() > ClawConstants.SensorThreshold) return true;
    else return false;
  }

  private void setShaftPosition(double angle) {
    shaft_motor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    if(level == Levels.L1) setShaftPosition(ClawConstants.Levels.L1.getAngle());
    if(level == Levels.Default) setShaftPosition(ClawConstants.Levels.Default.getAngle());
    if(level == Levels.L2) setShaftPosition(ClawConstants.Levels.L2.getAngle());
    if(level == Levels.L3) setShaftPosition(ClawConstants.Levels.L3.getAngle());
  }
}
