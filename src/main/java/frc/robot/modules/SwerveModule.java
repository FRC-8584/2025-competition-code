package frc.robot.modules;

import com.revrobotics.spark.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.PID;
import frc.robot.utils.Tools;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveModule {
	public final TalonSRX turningMotor;
	public final SparkMax driveMotor;

	private String name = "";
	private PID pid;

	private double turnValue;
	private double invert;

	/**********functions**********/

	public SwerveModule(final int turningMotorID, final int driveMotorID, final PID pid){
		turningMotor = new TalonSRX(turningMotorID);
		driveMotor = new SparkMax(driveMotorID, SparkLowLevel.MotorType.kBrushed);
		this.pid = pid;
		this.pid.setDeadband(0.01);

		turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		invert = 1;
	}

	public void update() {
		final double value = -((int)turningMotor.getSelectedSensorPosition() & 0x03ff) * 0.3515625;

		turnValue = value < 0 ? value + 360 : value;
		SmartDashboard.putNumber(name, turnValue);
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

		final double turnPower = Tools.bounding(pid.calculate(error / 90.0));
		final double drivePower = invert * speed * Math.cos(error * 0.0174533);

		turningMotor.set(TalonSRXControlMode.PercentOutput, -turnPower);
		driveMotor.set(drivePower);
	}

	public void stop() {
		turningMotor.set(TalonSRXControlMode.PercentOutput, 0);
		driveMotor.set(0);
	}

	/*** encoder ***/

	public double getEncValue() {
		return turnValue;
	}

	/*** PID ***/

	public void setPID(PID pid) {
		this.pid = pid;
	}

	/*** name ***/

	public void setName(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}
}
