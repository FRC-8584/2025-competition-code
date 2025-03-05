package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
  private final SparkMax shaft_motor;
  private final SparkMax grabber_motor;
  private final AnalogInput sensor;
  private boolean get;
  private double pose; 

  public Intake() {
    shaft_motor = new SparkMax(CAN_DeviceID.Intake_ShaftID, MotorType.kBrushless);
    grabber_motor = new SparkMax(CAN_DeviceID.Intake_GrabberID, MotorType.kBrushed);
    sensor = new AnalogInput(2);
    
    shaft_motor.configure(IntakeConstants.Configs.getShaftConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    grabber_motor.configure(IntakeConstants.Configs.getGrabberConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pose = 0;
    get = false;
  }

  public void setShaftPosition(double angle) {
    pose = angle;
  }

  public void setGrabberPower(double power) {
      grabber_motor.set(power);
  }

  public double getGrabberCurrent() {
    return grabber_motor.getOutputCurrent();
  }

  public boolean isDetected() {
    return sensor.getValue() > IntakeConstants.SensorThreshold ? true : false;
  }

  public boolean isGet() {
    return get;
  }

  public void setState(boolean isGet) {
    get = isGet;
  }

  @Override
  public void periodic() {
    shaft_motor.getClosedLoopController().setReference(pose ,ControlType.kPosition);
    SmartDashboard.getBoolean("Intake : Is get ALGAE", isGet());
  }
}
