package frc.robot.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;

import frc.robot.utils.Tools;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;

  private final PIDController pid;

  private SwerveModuleState current_state;
  private SwerveModulePosition current_position;
  private Rotation2d current_angle;

  public SwerveModule(int turnMotorId, int dirveMotorId, int turnEncoderId, double encoder_offset) {
    driveMotor = new TalonFX(dirveMotorId);
    turnMotor = new TalonFX(turnMotorId);
    turnEncoder = new CANcoder(turnEncoderId);

    applyCfg(encoder_offset);
    pid = new PIDController(0.7, 0, 1e-6);

    driveMotor.set(0);
    turnMotor.set(0);
  }

  public void setState(SwerveModuleState state) {
    state.optimize(current_angle);
    state.cosineScale(current_angle);
    double err_degree = state.angle.minus(current_angle).getDegrees();

    driveMotor.set(state.speedMetersPerSecond / SwerveConstants.kMaxDriveSpeed);
    turnMotor.set(Tools.bounding(pid.calculate(err_degree / 90.0)));
  }

  public SwerveModuleState getState() {
    return current_state;
  }
  
  public SwerveModulePosition getPosition() {
    return current_position;
  }

  public Rotation2d getEncAngle() {
    return current_angle;
  }

  private void applyCfg(double CANcoder_offset) {
    this.driveMotor.getConfigurator().apply(SwerveConstants.getDriveMotorCfg());
    this.turnMotor.getConfigurator().apply(SwerveConstants.getTurnMotorCfg());
    this.turnEncoder.getConfigurator().apply(SwerveConstants.getCANcoderCfg(CANcoder_offset));
  }

  public void update() {
    current_state = new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.kWheelPerimeter, getEncAngle());
    current_position = new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() / SwerveConstants.kDriveGearRatio * SwerveConstants.kWheelPerimeter, getEncAngle());
    current_angle = Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
  }

  public void logInfo(String name) {}
}
