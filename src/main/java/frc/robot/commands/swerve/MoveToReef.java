// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef extends Command {
  private Swerve swerve;
  private Reef reef;
  private double y_offset;
  private double t_angle, t_err, t_v;
  private double x_err, x_v;
  private double y_err, y_v;
  private double[] pose;
  
  public MoveToReef(Swerve swerve, Reef reef, double x_offset) {
    this.swerve = swerve;
    this.reef = reef;
    this.y_offset = x_offset;
    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    // turn
    t_angle = swerve.getGyroAngle().getDegrees()-pose[4];
    if(t_angle > 180.0) t_angle -= 360.0;
    else if(t_angle <=-180.0) t_angle += 360.0;
  }

  @Override
  public void execute() {
    // turn
    t_err = t_angle - swerve.getGyroAngle().getDegrees();
    if(t_err > 180.0) t_err -= 360.0;
    else if(t_err <=-180.0) t_err += 360.0;
    t_v = (t_err / 90.0) * SwerveConstants.MaxTurnSpeed;

    // move
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, (t_err / 90.0) * SwerveConstants.MaxTurnSpeed);
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds,
        Rotation2d.fromDegrees(t_err))
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MaxDriveSpeed);
    swerve.drive(states);
    System.out.println(t_err);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(Tools.isInRange(t_v, -0.1, 0.1)) return true;
    else return false;
  }
}
