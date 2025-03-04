package frc.robot.modules;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final TalonFX turnMotor;
  private final TalonFX driveMotor;
  private final CANcoder turnEncoder;
  private final PIDController pid;
  private double max = 0;

  public SwerveModule(int turnMotorId, int driveMotorId, int turnEncoderId, double offset) {
    turnMotor = new TalonFX(turnMotorId);
    driveMotor = new TalonFX(driveMotorId);
    turnEncoder = new CANcoder(turnEncoderId);
    pid = new PIDController(0.5, 0, 0);

    driveMotor.set(0);
    turnMotor.set(0);

    applyConfigs(turnEncoderId, offset);
  }

  public void setState(SwerveModuleState state) {
    state.optimize(getEncoderAngle());
    state.cosineScale(getEncoderAngle());
    double err_degree = state.angle.minus(getEncoderAngle()).getDegrees();

    driveMotor.set(state.speedMetersPerSecond / SwerveConstants.MaxDriveSpeed);
    turnMotor.set(err_degree / 90.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.WheelPerimeter, getEncoderAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.WheelPerimeter, getEncoderAngle());
  }

  private Rotation2d getEncoderAngle() {
    return Rotation2d.fromDegrees(turnMotor.getPosition().getValueAsDouble() * 360.0);
  }

  private void applyConfigs(int turnEncoderId, double offset) {
    turnMotor.getConfigurator().apply(SwerveConstants.Configs.turnMotorConfig(turnEncoderId));
    driveMotor.getConfigurator().apply(SwerveConstants.Configs.driveMotorConfig());
    turnEncoder.getConfigurator().apply(SwerveConstants.Configs.turnEncoderConfig(offset));
  }

  public void logging(String name) {
    double v = driveMotor.getVelocity().getValueAsDouble();
    max = v > max ? v : max;
    SmartDashboard.putNumber(name+ " V", max);
  }
}
