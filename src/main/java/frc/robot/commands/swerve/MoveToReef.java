package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperationConstant.Reef;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef extends Command {
  private Swerve swerve;
  private Reef reef;

  private double x_set_pos;
  private double y_set_pos;
  private double t_set_pos;// tag angle from field relative

  private double x_bot_pos;
  private double y_bot_pos;
  private double t_bot_pos;// robot angle from field relative

  private double x_err;
  private double y_err;
  private double t_err;

  private double x_velocity;
  private double y_velocity;
  private double t_velocity;

  private boolean x_isFinish;
  private boolean y_isFinish;
  private boolean t_isFinish;
  
  public MoveToReef(Swerve swerve, Reef reef, double x_setpoint) {
    this.swerve = swerve;
    this.reef = reef;
    this.x_set_pos = x_setpoint;
    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    double raw_x = pose[2];
    double raw_y = -pose[0];
    double raw_turn = -pose[4];

    final double temp_sin = Math.sin(raw_turn / 180.0 * Math.PI);
    final double temp_cos = Math.cos(raw_turn / 180.0 * Math.PI);

    x_bot_pos = -(raw_x * temp_cos + raw_y * temp_sin);
    y_bot_pos = raw_x * temp_sin - raw_y * temp_cos;
    t_bot_pos = swerve.getGyroAngle().getDegrees();

    if(reef == Reef.Left) y_set_pos = -0.17;// left
    else if(reef == Reef.Right) y_set_pos = 0.17;// right
    else y_set_pos = 0.0;// none

    // turn
    t_set_pos = t_bot_pos + raw_turn;

    if(t_set_pos > 180.0) t_set_pos -= 360.0;
    else if(t_set_pos <= -180.0) t_set_pos += 360.0;

    x_isFinish = false;
    y_isFinish = false;
    t_isFinish = false;

    logInfo();
  }

  @Override
  public void execute() {
    t_bot_pos = swerve.getGyroAngle().getDegrees();
    t_err = t_set_pos - t_bot_pos;

    if(t_err > 180.0) t_err -= 360.0;
    else if(t_err <=-180.0) t_err += 360.0;

    // get current chassis speed
    ChassisSpeeds current_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      swerve.getRobotRelativeSpeeds(),
      Rotation2d.fromDegrees(t_err)
    );

    x_bot_pos += current_speeds.vxMetersPerSecond * 0.02;// (vx)m/s * 0.02s
    y_bot_pos += current_speeds.vyMetersPerSecond * 0.02;// (vy)m/s * 0.02s

    x_err = x_set_pos - x_bot_pos;
    y_err = y_set_pos - y_bot_pos;

    x_velocity = Tools.bounding(x_err / 1.5) * SwerveConstants.MaxDriveSpeed;
    y_velocity = Tools.bounding(y_err / 2.0) * SwerveConstants.MaxDriveSpeed;
    t_velocity = Tools.bounding(t_err / 90.0) * SwerveConstants.MaxTurnSpeed;

    checkIsFinished();

    // apply new chassis speed
    ChassisSpeeds apply_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(x_velocity, y_velocity, t_velocity),
      Rotation2d.fromDegrees(t_err)
    );

    swerve.drive(apply_speeds, false);
    logInfo();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    if(x_isFinish && y_isFinish && t_isFinish) return true;
    else return false;
  }

  private void checkIsFinished() {
    if(Tools.isInRange(x_err, -0.02, 0.02)){
      x_velocity = 0;
      x_isFinish = true;
    }
    else x_isFinish = false;

    if(Tools.isInRange(y_err, -0.02, 0.02)){
      y_velocity = 0;
      y_isFinish = true;
    }
    else y_isFinish = false;

    if(Tools.isInRange(t_err, -2.0, 2.0)){
      t_velocity = 0;
      t_isFinish = true;
    }
    else t_isFinish = false;
  }

  private void logInfo() {
    SmartDashboard.putNumber("robot x position", x_bot_pos);
    SmartDashboard.putNumber("robot y position", y_bot_pos);
    SmartDashboard.putNumber("robot angle", t_bot_pos);

    SmartDashboard.putBoolean("robot x position finished", x_isFinish);
    SmartDashboard.putBoolean("robot y position finished", y_isFinish);
    SmartDashboard.putBoolean("robot angle finished", t_isFinish);
  }
}
