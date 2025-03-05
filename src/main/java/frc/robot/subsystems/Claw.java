package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.Levels;

public class Claw extends SubsystemBase {
  private final SparkMax shaft_motor;
  private final SparkMax grabber_motor;
  private final AnalogInput sensor;
  private Levels level = Levels.Default;

  private double stuck_pose;
  private boolean stuck;


  public Claw() {
    shaft_motor = new SparkMax(CAN_DeviceID.Claw_ShaftID, MotorType.kBrushless);
    grabber_motor = new SparkMax(CAN_DeviceID.Claw_GrabberID, MotorType.kBrushless);
    sensor = new AnalogInput(ClawConstants.SensorPort);

    shaft_motor.configure(ClawConstants.Configs.getShaftConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    grabber_motor.configure(ClawConstants.Configs.getGrabberConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    stuck_pose = grabber_motor.getEncoder().getPosition();
    stuck = true;

    grabber_motor.set(0);
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

  public boolean detectCoral() {
    if (sensor.getValue() > ClawConstants.SensorThreshold) return true;
    else return false;
  }

  public boolean isStuck() {
    return stuck;
  }

  public void stuckAlgae(boolean stuck) {
    this.stuck = stuck;
    stuck_pose = grabber_motor.getEncoder().getPosition();
  }

  private void setShaftPosition(double angle) {
    shaft_motor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    setShaftPosition(level.getAngle());
    if(stuck) grabber_motor.getClosedLoopController().setReference(stuck_pose, ControlType.kPosition);
    System.out.println(sensor.getValue());
  }
}
