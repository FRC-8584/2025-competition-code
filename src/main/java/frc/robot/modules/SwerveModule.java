package frc.robot.modules;

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
    Rotation2d angle =  Rotation2d.fromDegrees(turnEncoder.getPosition().getValueAsDouble() * 360.0);
    state.optimize(angle);
    state.cosineScale(angle);
    double err_degree = state.angle.minus(angle).getDegrees();

    double drive_power = state.speedMetersPerSecond / SwerveConstants.MaxDriveSpeed;
    double turn_power = pid.calculate(err_degree / 90.0);

    driveMotor.set(drive_power);
    turnMotor.set(turn_power);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.WheelPerimeter, getEncoderAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.WheelPerimeter, getEncoderAngle());
  }

  private Rotation2d getEncoderAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getPosition().getValueAsDouble() * 360.0);
  }

  private void applyConfigs(int turnEncoderId, double offset) {
    turnMotor.getConfigurator().apply(SwerveConstants.Configs.turnMotorConfig(turnEncoderId));
    driveMotor.getConfigurator().apply(SwerveConstants.Configs.driveMotorConfig());
    turnEncoder.getConfigurator().apply(SwerveConstants.Configs.turnEncoderConfig(offset));
  }

  public void logging(String name) {
  }
}
