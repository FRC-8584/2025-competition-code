// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.Levels;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkMax l_motor = new SparkMax(Constants.CAN_DeviceID.Left_ElevatorID, MotorType.kBrushless);
  SparkMax r_motor = new SparkMax(Constants.CAN_DeviceID.Right_ElevatorID, MotorType.kBrushless);

  public Levels m_L = Levels.Default;

  public Elevator() {
    l_motor.configure(Constants.ElevatorConstants.Configs.GetElevatorConfig(true), ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    r_motor.configure(Constants.ElevatorConstants.Configs.GetElevatorConfig(false), ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
  }

  public void SetPower(double n) {
    l_motor.set(n);
    r_motor.set(n);
  }

  public void SetPosition() {
    l_motor.getClosedLoopController().setReference(m_L.getHeight(), ControlType.kPosition);
    r_motor.getClosedLoopController().setReference(m_L.getHeight(), ControlType.kPosition);
  }
  
  @Override
  public void periodic() {
    SetPosition();
    SmartDashboard.putNumber("The_Elevator_Height", getPosition());
  }
  public double getPosition() {
    return (l_motor.getEncoder().getPosition() + r_motor.getEncoder().getPosition()) / 2.0;
  }
}
