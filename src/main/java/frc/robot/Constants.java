package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Constants {

  public static class OperatorConstants {
    public static final int Player1Port = 0;
    public static final int Player2Port = 1;

    // Heading
    public static final double OriginRobotHeading = 0; //degrees
    public static final double DriverHeading = 0; //degrees
  }

  /***** Autonomous Time Action *****/
  public static enum AutoActions {
    ShootOnly
  }

  /***** Alliance *****/
  public static enum Alliance {
    RED, Blue
  }

  /***** Robot Mechanical Constants *****/
  public static class MechanicalConstants {
    // Chassis
    public static final double RobotLength = 1.0;
    public static final double RobotWidth = 1.0;
    public static final double r = Math.sqrt(RobotLength * RobotLength + RobotWidth * RobotWidth);
  }

  /***** Sensor Constants *****/
  public static class SensorConstants {
    public static final int IntakeSwitchPortID = 1;
  }

  /***** Field Constants *****/
  public static class FieldConstants {}

  /***** motor controller ID *****/
  public static class MotorControllerID {
    public static final int LF_TurnID         =  1;
    public static final int LR_TurnID         =  4;
    public static final int RF_TurnID         =  2;
    public static final int RR_TurnID         =  3;

    public static final int LF_DriveID        =  5;
    public static final int LR_DriveID        =  8;
    public static final int RF_DriveID        =  6;
    public static final int RR_DriveID        =  7;

    public static final int Left_ElevatorID   =  9;
    public static final int Right_ElevatorID  =  10;

    public static final int Claw_ShaftID      =  11;
    public static final int Claw_GrapperID    =  12;
  }

  /***** motor controller config *****/
  public static class MotorControllerCfg {
    public static final SparkMaxConfig getLeftElevatorCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();
      final double gearbox_rate = 25;
      final double position_per_rotations = 6;// cm/rotations
      config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(position_per_rotations/gearbox_rate)// return position (cm)
        .velocityConversionFactor(position_per_rotations/gearbox_rate/60.0);// return velocity (cm/s)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(1, 0, 0)
        .maxOutput(MotorConstants.kElevatorSpd)
        .minOutput(-MotorConstants.kElevatorSpd);

      return config;
    }

    public static final SparkMaxConfig getRightElevatorCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();
      final double gearbox_rate = 25;
      final double position_per_rotations = 6;// cm/rotations
      config
        .inverted(true)// inverted
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(position_per_rotations/gearbox_rate)// return position (cm)
        .velocityConversionFactor(position_per_rotations/gearbox_rate/60.0);// return velocity (cm/s)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(1, 0, 0)
        .maxOutput(MotorConstants.kElevatorSpd)
        .minOutput(-MotorConstants.kElevatorSpd);

      return config;
    }

    public static final SparkMaxConfig getClawShaftCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();
      final double gearbox_rate = 80;
      final double position_per_rotations = 360;// deg/rotations
      config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(position_per_rotations/gearbox_rate);// return position (deg)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(0.03, 0, 0)
        .maxOutput(MotorConstants.kShaftSpd)
        .minOutput(-MotorConstants.kShaftSpd);

      return config;
    }

    public static final SparkMaxConfig getClawGrapperCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();
      config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
      return config;
    }
  }

  /***** motor speed constants *****/
  public static class MotorConstants {
    // Robot Move & Turn Speed
    public static final double kMove = 0.6;
    public static final double kTrun = 0.6;

    // Elevator Move Speed
    public static final double kElevatorSpd = 0.4;

    // Claw Shaft & Grapper Speed
    public static final double kShaftSpd = 0.3;
    public static final double kGrapperSpd = 0.8;
  }
}
