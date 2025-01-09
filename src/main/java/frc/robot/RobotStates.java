package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.utils.devices.BNO055;
import frc.robot.utils.devices.BNO055.opmode_t;
import frc.robot.utils.devices.BNO055.vector_type_t;

import frc.robot.Constants.SensorConstants;

public class RobotStates {
    private static RobotStates instance = new RobotStates();
    private java.util.Timer executor;
    
    // devices
    private static BNO055 m_gyro;
    private static DigitalInput m_intake_switch;

    // values
    private volatile double gyroX;
    private volatile boolean intake_switch;

    private RobotStates() {
        // Sensors
        m_gyro = BNO055.getInstance(opmode_t.OPERATION_MODE_NDOF, vector_type_t.VECTOR_EULER, I2C.Port.kMXP, BNO055.BNO055_ADDRESS_A);
        m_intake_switch = new DigitalInput(SensorConstants.IntakeSwitchPortID);
        
        // Timer
        executor = new java.util.Timer();
		executor.schedule(new RobotStatesUpdateTask(this), 0L, 20);
    }

    public static RobotStates getInstance() {
        return instance;
    }

    // GET functions.
    public double getGyroX() {
        return this.gyroX;
    }

    public boolean getIntakeSwitch() {
        return this.intake_switch;
    }

    // Update device values.
    private void update() {
        gyroX = m_gyro.getVector()[0];
        intake_switch = m_intake_switch.get();
    }

    private class RobotStatesUpdateTask extends TimerTask {
		private RobotStates robotStates;

		private RobotStatesUpdateTask(RobotStates robotStates) {
			if (robotStates == null) {
				throw new NullPointerException("RobotStates pointer null");
			}
			this.robotStates = robotStates;
		}
        
		public void run() {
			robotStates.update();
		}
	}
}
