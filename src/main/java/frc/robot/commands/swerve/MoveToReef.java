// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef extends Command {
  private Swerve swerve;
  private Reef reef;
  
  public MoveToReef(Swerve swerve, Reef reef) {
    this.swerve = swerve;
    this.reef = reef;
    addRequirements(this.swerve);
  }

  @Override
  public void execute() {
    double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    double turn = -pose[4]/ 45.0;
    double tx;
    if(reef == Reef.Left) tx = -17.0;
    else tx = 17.0;
    swerve.drive(
      Tools.deadband((pose[2] - 0.53) / .8, 0.1) ,
      Tools.deadband((tx - pose[0]) / .8, 0.1),
      Tools.deadband(turn, 0.1),false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
