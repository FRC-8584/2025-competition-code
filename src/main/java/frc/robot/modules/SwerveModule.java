package frc.robot.modules;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
	public final TalonFX driveMotor, turnMotor;
	public final CANcoder enc_CANcoder;

	private double turnValue;
	private double invert;

	/**********functions**********/

	public SwerveModule(final int turnMotorID, final int driveMotorID, final int CANcoderID){
		driveMotor = new TalonFX(driveMotorID);
		turnMotor = new TalonFX(turnMotorID);
		enc_CANcoder = new CANcoder(CANcoderID);

		driveMotor.getConfigurator().apply(SwerveConstants.getDriveMotorCfg());
		turnMotor.getConfigurator().apply(SwerveConstants.getTurnMotorCfg());
		enc_CANcoder.getConfigurator().apply(SwerveConstants.getCANcoderCfg());

		invert = 1;
	}

	public void update() {
		turnValue = enc_CANcoder.getPosition().getValueAsDouble();
	}

	/*** motor ***/

	public void setpoint(final double speed, final double angle) {
		double error = angle - turnValue;//SP - PV 

		if(invert == -1){
			error -= 180;
			error = error < -180 ? error + 360 : error;
		}

		error = error > 180 ? error - 360 : error;
		error = error < -180 ? error + 360 : error;

		if(-90 <= error && error < 90){}
		else if(90 <= error && error < 180){
			error -= 180;
			invert *= -1.0;
		}
		else if(-180 <= error && error < -90){
			error += 180;
			invert *= -1.0;
		}

		final double turnPower = 0;
		final double drivePower = speed * Math.cos(error * 0.0174533);

		turnMotor.set(turnPower);
		driveMotor.set(drivePower);
	}

	public void stop() {
		turnMotor.set(0);
		driveMotor.set(0);
	}

	/*** encoder ***/

	public double getEncValue() {
		return turnValue;
	}

}
