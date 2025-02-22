package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

  /***** CAN Device ID *****/
  public static class CAN_DeviceID {
    public static final int LF_TurnID         =  1;
    public static final int LR_TurnID         =  4;
    public static final int RF_TurnID         =  2;
    public static final int RR_TurnID         =  3;

    public static final int LF_DriveID        =  5;
    public static final int LR_DriveID        =  8;
    public static final int RF_DriveID        =  6;
    public static final int RR_DriveID        =  7;

    public static final int LF_CANcoderID     =  9;
    public static final int LR_CANcoderID     =  12;
    public static final int RF_CANcoderID     =  10;
    public static final int RR_CANcoderID     =  11;

    public static final int Left_ElevatorID   =  13;
    public static final int Right_ElevatorID  =  14;

    public static final int ShaftID           =  15;
    public static final int GrabberID         =  16;

    public static final int Left_ClimberID    =  17;
    public static final int Right_ClimberID   =  18;

  }

  public static class SwerveConstants {
    // Robot Move & Turn Speed
    public static final double kDriveSpeed = 1.0;
    public static final double kTurnSpeed = 1.0;

    // Motor gear ratio
    public static final double kDriveMotorGearRatio = 8.14;

    // Motor controller inverted settings
    public static final boolean kDriveMotorInverted = false;
    public static final boolean kTurnMotorInverted = false;

    // Swerve measurements
    public static final double kSwerveMaxSpeed = 4.0;
    public static final double kWheelRadius = 50.27;// mm
    public static final double kDistancePerRotate = kWheelRadius * 2 * Math.PI / kDriveMotorGearRatio / 1000.0;// m/rotation
    public static final double kWidth = 712.0;
    public static final double kHeight = 709.0;

    // encoder offsets
    public static final double kLF_offset = -0.438721;
    public static final double kLR_offset = -0.307373;
    public static final double kRF_offset = -0.430176;
    public static final double kRR_offset = -0.307861;

    public static final TalonFXConfiguration getDriveMotorCfg() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.Feedback.SensorToMechanismRatio = kDistancePerRotate;
      return config;
    }

    public static final TalonFXConfiguration getTurnMotorCfg() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      return config;
    }

    public static final CANcoderConfiguration getCANcoderCfg(double offset) {
      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      config.MagnetSensor.MagnetOffset = offset;
      return config;
    }
  }

  public static class ElevatorConstants {
    // Elevator Move Speed
    public static final double kElevatorUpSpeed = 1.0;
    public static final double kElevatorDownSpeed = 1.0;

    // Motor rotate rate (cm/rotations)
    public static final double kRotateRate = 11.43/15.0;

    // Shaft position limits (cm)
    public static final double kElevatorMinPosition = 0;
    public static final double kElevatorMaxPosition = 75.0;

    // Motor controller closed loop control pid (Elevator)
    public static final double kp = 0.1;
    public static final double ki = 0;
    public static final double kd = 0;

    // Motor controller inverted settings
    public static final boolean kLeftElevatorInverted = true;
    public static final boolean kRightElevatorInverted = false;

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
    public static final double kShaftMaxPosition = 200.0;

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


  public static class GrabberConstants {
    // Grabber Speed
    public static final double kGrabberSpeed = 1.0;

    // Motor controller inverted settings
    public static final boolean kGrabberInverted = false;

    public static final SparkMaxConfig getGrabberCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kGrabberInverted)
        .idleMode(IdleMode.kBrake);

      return config;
    }
  }

  public static class ClimberConstants {
    // Climber Move Speed
    public static final double kClimberForwardSpeed = 1.0;
    public static final double kClimberReverseSpeed = 1.0;

    // Motor rotate rate (deg/rotations)
    public static final double kRotateRate = 360.0/(100.0*5.77777);

    // Climber Shaft position limits (deg)
    public static final double kClimberMinPosition = 0;
    public static final double kClimberMaxPosition = 270.0;

    // Motor controller closed loop control pid (Climber Shaft)
    public static final double kp = 0.02;
    public static final double ki = 0;
    public static final double kd = 0;

    // Motor controller inverted settings
    public static final boolean kLeftClimberInverted = false;
    public static final boolean kRightClimberInverted = true;

    public static final SparkMaxConfig getLeftClimberCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kLeftClimberInverted)
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(kRotateRate)// return position (deg)
        .velocityConversionFactor(kRotateRate/60.0);// return velocity (deg/s)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(kp, ki, kd)
        .outputRange(-kClimberReverseSpeed, kClimberForwardSpeed)
        .positionWrappingInputRange(kClimberMinPosition, kClimberMaxPosition);

      return config;
    }

    public static final SparkMaxConfig getRightClimberCfg() {
      final SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(kRightClimberInverted)
        .idleMode(IdleMode.kBrake);
      config.encoder
        .positionConversionFactor(kRotateRate)// return position (deg)
        .velocityConversionFactor(kRotateRate/60.0);// return velocity (deg/s)
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
        .pid(kp, ki, kd)
        .outputRange(-kClimberReverseSpeed, kClimberForwardSpeed)
        .positionWrappingInputRange(kClimberMinPosition, kClimberMaxPosition);

      return config;
    }
  }
}
