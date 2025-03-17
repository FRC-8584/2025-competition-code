package frc.robot.modules;

import com.ctre.phoenix6.Orchestra;
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

  private SwerveModuleState current_state;
  private SwerveModulePosition current_position;
  private Rotation2d current_angle;
  private double max = 0;

  public SwerveModule(int turnMotorId, int driveMotorId, int turnEncoderId, double offset) {
    turnMotor = new TalonFX(turnMotorId);
    driveMotor = new TalonFX(driveMotorId);
    turnEncoder = new CANcoder(turnEncoderId);
    pid = new PIDController(0.5, 0, 0);

    driveMotor.set(0);
    turnMotor.set(0);
    current_state = new SwerveModuleState(0, getEncoderAngle());
    current_position = new SwerveModulePosition(0, getEncoderAngle());
    current_angle = Rotation2d.fromDegrees(turnEncoder.getPosition().getValueAsDouble() * 360.0);

    applyConfigs(turnEncoderId, offset);
  }

  public void setState(SwerveModuleState state) {
    state.optimize(current_angle);
    state.cosineScale(current_angle);
    double err_degree = state.angle.minus(current_angle).getDegrees();

    double apply_drive_output = state.speedMetersPerSecond / SwerveConstants.MaxDriveSpeed;
    double apply_turn_output = pid.calculate(err_degree / 90.0);

    driveMotor.set(apply_drive_output);
    turnMotor.set(apply_turn_output);
  }

  public SwerveModuleState getState() {
    return current_state;
  }

  public SwerveModulePosition getPosition() {
    return current_position;
  }

  private Rotation2d getEncoderAngle() {
    return current_angle;
  }

  private void applyConfigs(int turnEncoderId, double offset) {
    turnMotor.getConfigurator().apply(SwerveConstants.Configs.turnMotorConfig(turnEncoderId));
    driveMotor.getConfigurator().apply(SwerveConstants.Configs.driveMotorConfig());
    turnEncoder.getConfigurator().apply(SwerveConstants.Configs.turnEncoderConfig(offset));
  }

  public void update() {
    current_state = new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.WheelPerimeter, getEncoderAngle());
    current_position = new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.WheelPerimeter, getEncoderAngle());
    current_angle = Rotation2d.fromDegrees(turnEncoder.getPosition().getValueAsDouble() * 360.0);
  }

  public void logging(String name) {
    double v = driveMotor.getVelocity().getValueAsDouble();
    max = v > max ? v : max;
  }

  public void SetSinging(Orchestra m_Orchestra) {
    m_Orchestra.addInstrument(driveMotor);
    m_Orchestra.addInstrument(turnMotor);
    return;
  }
}
