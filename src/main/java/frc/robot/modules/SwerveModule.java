package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;

  private final PIDController pid;
  
  private double invert;
  
  public SwerveModule(int turnMotorId, int dirveMotorId, int turnEncoderId, double encoder_offset) {
    driveMotor = new TalonFX(dirveMotorId);
    turnMotor = new TalonFX(turnMotorId);
    turnEncoder = new CANcoder(turnEncoderId);

    pid = new PIDController(0.7, 0, 1e-6);

    invert = 1.0;

    applyConfigs(
			SwerveConstants.getDriveMotorCfg(),
			SwerveConstants.getTurnMotorCfg(),
			SwerveConstants.getCANcoderCfg(encoder_offset)
		);
  }

  public void setState(SwerveModuleState state) {
    double err_degree = errCalculator(state.angle.getDegrees() - getEncoderAngle().getDegrees());
    driveMotor.set(state.speedMetersPerSecond * invert);
    turnMotor.set(pid.calculate(err_degree / 90));
  }
  
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getEncoderAngle());
  }

  private Rotation2d getEncoderAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
  }

  private double errCalculator(double err) {
		if(invert == -1){
			err -= 180;
			err = err < -180 ? err + 360 : err;
		}

		err = err > 180 ? err - 360 : err;
		err = err < -180 ? err + 360 : err;

		if(-90 <= err && err < 90){}
		else if(90 <= err && err < 180){
			err -= 180;
			invert *= -1.0;
		}
		else if(-180 <= err && err < -90){
			err += 180;
			invert *= -1.0;
		}
    return err;
  }

  public void applyConfigs(TalonFXConfiguration driveMotor, TalonFXConfiguration turnMotor, CANcoderConfiguration turnEncoder) {
    this.driveMotor.getConfigurator().apply(driveMotor);
    this.turnMotor.getConfigurator().apply(turnMotor);
    this.turnEncoder.getConfigurator().apply(turnEncoder);
  }

  public void logging(String name) {}
}
