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

    public static final int Left_ElevatorID   =  10;
    public static final int Right_ElevatorID  =  9;

    public static final int ShaftID      =  11;
    public static final int GrapperID    =  12;
  }

  public static class SwerveConstants {
    // Robot Move & Turn Speed
    public static final double kMoveSpeed = 1.0;
    public static final double kTrunSpeed = 1.0;
  }

  public static class ElevatorConstants {
    // Elevator Move Speed
    public static final double kElevatorUpSpeed = 1.0;
    public static final double kElevatorDownSpeed = 0.6;

    // Motor rotate rate (cm/rotations)
    public static final double kRotateRate = 11.43/15.0;

    // Shaft position limits (cm)
    public static final double kElevatorMinPosition = 0;
    public static final double kElevatorMaxPosition = 75.0;

    // Motor controller closed loop control pid (Elevator)
    public static final double kp = 0.08;
    public static final double ki = 0;
    public static final double kd = 0;

    // Motor controller inverted settings
    public static final boolean kLeftElevatorInverted = false;
    public static final boolean kRightElevatorInverted = true;

    public static final SparkMaxConfig getLeftElevatorCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kLeftElevatorInverted)
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(kRotateRate)// return position (cm)
        .velocityConversionFactor(kRotateRate/60.0);// return velocity (cm/s)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(kp, ki, kd)
        .outputRange(-kElevatorDownSpeed, kElevatorUpSpeed)
        .positionWrappingInputRange(kElevatorMinPosition, kElevatorMaxPosition);

      return config;
    }

    public static final SparkMaxConfig getRightElevatorCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kRightElevatorInverted)// inverted
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(kRotateRate)// return position (cm)
        .velocityConversionFactor(kRotateRate/60.0);// return velocity (cm/s)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(kp, ki, kd)
        .outputRange(-kElevatorDownSpeed, kElevatorUpSpeed)
        .positionWrappingInputRange(kElevatorMinPosition, kElevatorMaxPosition);

      return config;
    }
  }

  public static class ShaftConstants {
    // Shaft Speed
    public static final double kShaftSpeed = 1.0;

    // Motor rotate rate (deg/rotations)
    public static final double kRotateRate = 360.0/80.0;

    // Shaft position limits (deg)
    public static final double kShaftMinPosition = 0;
    public static final double kShaftMaxPosition = 135.0;

    // Motor controller closed loop control pid (Shaft)
    public static final double kp = 0.01;
    public static final double ki = 0;
    public static final double kd = 0;

    // Motor controller inverted settings
    public static final boolean kShaftInverted = true;
    
    public static final SparkMaxConfig getShaftCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kShaftInverted)
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(kRotateRate);// return position (deg)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(kp, ki, kd)
        .outputRange(-kShaftSpeed, kShaftSpeed)
        .positionWrappingInputRange(kShaftMinPosition, kShaftMaxPosition);

      return config;
    }
  }


  public static class GrapperConstants {
    // Grapper Speed
    public static final double kGrapperSpeed = 1.0;

    // Motor controller inverted settings
    public static final boolean kGrapperInverted = false;

    public static final SparkMaxConfig getGrapperCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kGrapperInverted)
        .idleMode(IdleMode.kBrake);

      return config;
    }
  }
}
