package frc.robot.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final TalonFX turnMotor;
    private final TalonFX driveMotor;
    private final CANcoder turnEncoder;
    private final PIDController pid;

    private double invert;

    public SwerveModule(int turnMotorId, int driveMotorId, int turnEncoderId, double offset) {
        turnMotor = new TalonFX(turnMotorId);
        driveMotor = new TalonFX(driveMotorId);
        turnEncoder = new CANcoder(turnEncoderId);
        pid = new PIDController(0.5, 0, 0);

        applyConfigs(turnEncoderId, offset);
        invert = 1.0;
    }

    public void setState(SwerveModuleState state) {
        double err_degree = errCalculator(state.angle.getDegrees() - getEncoderAngle().getDegrees());
        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.MaxDriveSpeed * invert);
        turnMotor.set(pid.calculate(err_degree / 90.0));
    }

    public SwerveModulePosition gePosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() / 8.14 * SwerveConstants.WheelPerimeter, getEncoderAngle());
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

    private Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
      }

    private void applyConfigs(int turnEncoderId, double offset) {
        turnMotor.getConfigurator().apply(SwerveConstants.Configs.turnMotorConfig(turnEncoderId));
        driveMotor.getConfigurator().apply(SwerveConstants.Configs.driveMotorConfig());
        turnEncoder.getConfigurator().apply(SwerveConstants.Configs.turnEncoderConfig(offset));
    }

    public void logging(String name) {
        SmartDashboard.putNumber(name+ " enc", getEncoderAngle().getDegrees());
    }
}
