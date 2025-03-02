package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
  private final SparkMax shaft_motor;
  private final SparkMax grabber_motor;
  private double pose;

  public Intake() {
    pose = 0;
    shaft_motor = new SparkMax(CAN_DeviceID.Intake_ShaftID, MotorType.kBrushless);
    grabber_motor = new SparkMax(CAN_DeviceID.Intake_GrabberID, MotorType.kBrushless);

    shaft_motor.configure(IntakeConstants.Configs.getShaftConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    grabber_motor.configure(IntakeConstants.Configs.getGrabberConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setShaftPosition(double angle) {
    pose = angle;
  }

  public void setGrabberPower(double power) {
    grabber_motor.set(power);
  }

  public double getGrabberCurent() {
    System.out.println(grabber_motor.getOutputCurrent());
    return 0;
  }

  @Override
  public void periodic() {
    shaft_motor.getClosedLoopController().setReference(pose ,ControlType.kPosition);
  }
}
