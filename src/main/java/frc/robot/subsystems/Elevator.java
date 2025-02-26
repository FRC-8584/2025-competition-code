// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkMax l_motor = new SparkMax(Constants.CAN_DeviceID.Left_ElevatorID, MotorType.kBrushless);
  SparkMax r_motor = new SparkMax(Constants.CAN_DeviceID.Right_ElevatorID, MotorType.kBrushless);

  public double UShouldBeHere = 0.0;

  public enum Levels{
    L1, L2, L3, L4
  }

  public Elevator() {
    l_motor.configure(Constants.ElevatorConstants.Configs.GetElevatorConfig(false), ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    r_motor.configure(Constants.ElevatorConstants.Configs.GetElevatorConfig(true), ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
  }

  public void SetPower(double n) {
    l_motor.set(n);
    r_motor.set(n);
  }

  public void SetPosition(Levels m_L) {
    switch (m_L) {
      case L1:

        l_motor.getClosedLoopController().setReference(0.0, ControlType.kPosition);
        r_motor.getClosedLoopController().setReference(0.0, ControlType.kPosition);
        break;

      case L2:

        l_motor.getClosedLoopController().setReference(1.0, ControlType.kPosition);
        r_motor.getClosedLoopController().setReference(1.0,ControlType.kPosition);
        break;

      case L3:

        l_motor.getClosedLoopController().setReference(0.0, ControlType.kPosition);
        r_motor.getClosedLoopController().setReference(0.0, ControlType.kPosition);
        break;

      case L4:

        l_motor.getClosedLoopController().setReference(0.0, ControlType.kPosition);
        r_motor.getClosedLoopController().setReference(0.0, ControlType.kPosition);
        break;

      default:
        break;
    }
  }

  @Override
  public void periodic() {
    
  }
  public double getPosition() {
    return (l_motor.getEncoder().getPosition() + r_motor.getEncoder().getPosition()) / 2.0;
  }
}
