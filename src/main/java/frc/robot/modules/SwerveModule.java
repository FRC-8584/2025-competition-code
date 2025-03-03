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

  private double invert;

  public SwerveModule(int turnMotorId, int dirveMotorId, int turnEncoderId, double encoder_offset) {
    driveMotor = new TalonFX(dirveMotorId);
    turnMotor = new TalonFX(turnMotorId);
    turnEncoder = new CANcoder(turnEncoderId);

    applyCfg(encoder_offset);
    pid = new PIDController(0.7, 0, 1e-6);
    invert = 1.0;

    driveMotor.set(0);
    turnMotor.set(0);
  }

  public void setState(SwerveModuleState state) {
    double err_degree = errCalculator(state.angle.getDegrees() - getEncAngle().getDegrees());
    driveMotor.set((state.speedMetersPerSecond / SwerveConstants.kMaxDriveSpeed) * Math.cos(err_degree * Math.PI / 180.0) * invert);
    turnMotor.set(Tools.bounding(pid.calculate(err_degree / 90.0)));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.kWheelPerimeter, getEncAngle());
  }
  
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getEncAngle());
  }

  private Rotation2d getEncAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
  }

  private double errCalculator(double err) {
		if(invert == -1){
			err -= 180;
			err = err < -180 ? err + 360 : err;
		}

		err = err > 180 ? err - 360 : err;
		err = err < -180 ? err + 360 : err;

		if(-90 <= err && err < 90) {
      // do nothing
    }
		else if(90 <= err && err < 180) {
			err -= 180;
			invert *= -1.0;
		}
		else if(-180 <= err && err < -90) {
			err += 180;
			invert *= -1.0;
		}

    return err;
  }

  private void applyCfg(double CANcoder_offset) {
    this.driveMotor.getConfigurator().apply(SwerveConstants.getDriveMotorCfg());
    this.turnMotor.getConfigurator().apply(SwerveConstants.getTurnMotorCfg());
    this.turnEncoder.getConfigurator().apply(SwerveConstants.getCANcoderCfg(CANcoder_offset));
  }

  public void logInfo(String name) {}
}
