package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import frc.robot.utils.AxieOptimizer;

public class Constants {
  public static enum Levels {
    Coral_L1(0, 0), 
    Coral_L2(15.0, 5.0), 
    Coral_L3(15.0, 27.0), 
    Coral_L4(55.0, 75.0),
    Algea_L1(55.0, 5.0),
    Algea_L2(55.0, 27.0),
    Dodge(13.0, -1),
    Default(13.0, -1);

    private double claw_angle, elevator_height;

    private Levels(double claw_angle, double elevator_height) {
      this.claw_angle = claw_angle;
      this.elevator_height = elevator_height;
    }

    public double getAngle() {
      return claw_angle;
    }

    public double getHeight() {
      return elevator_height;
    }
  }

  public static class OperationConstant {

    public static enum Reef {
      Left, Right, None
    }

    public static final AxieOptimizer[] axieOptimizers = 
      new AxieOptimizer[] {
        new AxieOptimizer(0.035),
        new AxieOptimizer(0.035),
        new AxieOptimizer(0.05)
    };
          
    public static final double TurnSpeed = 0.4;
    public static final double DriveSpeed = 0.7;

    public static class Keys {
      public static final int ToReefL1 = 1;
      public static final int ToReefL2 = 2;
      public static final int ToReefL3 = 4;
      public static final int ToReefL4 = 3;
      public static final int GetCoral = 5;
      public static final int StopGetCoral = 6;
    }
  }
  
  public static class CAN_DeviceID {
    public static final int FL_TurnID         =  1;
    public static final int BL_TurnID         =  4;
    public static final int FR_TurnID         =  2;
    public static final int BR_TurnID         =  3;
  
    public static final int FL_DriveID        =  5;
    public static final int BL_DriveID        =  8;
    public static final int FR_DriveID        =  6;
    public static final int BR_DriveID        =  7;
  
    public static final int FL_CANcoderID     =  9;
    public static final int BL_CANcoderID     =  12;
    public static final int FR_CANcoderID     =  10;
    public static final int BR_CANcoderID     =  11;
  
    public static final int Left_ElevatorID   =  13;
    public static final int Right_ElevatorID  =  14;
  
    public static final int ShaftID           =  15;
    public static final int GrabberID         =  16;
  }

  public static class SwerveConstants{
    public static final double kDriveGearRatio = 8.14;
    public static final double kWheelRadius = 0.053; // m
    public static final double kSwerveWheelDistance_x = 0.565;// m
    public static final double kSwerveWheelDistance_y = 0.585;// m

    public static final double WheelPerimeter = kWheelRadius * 2 * Math.PI;  // m
    public static final double kRadian = 
      Math.sqrt(Math.pow(kSwerveWheelDistance_x/2.0, 2) + Math.pow(kSwerveWheelDistance_y/2.0, 2));// m

    public static final double MaxDriveSpeed = 1.483295; // m/s
    public static final double MaxTurnSpeed = MaxDriveSpeed / kRadian; // rad/s

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d[] {
        new Translation2d(kSwerveWheelDistance_x/2.0, kSwerveWheelDistance_y/2.0),
        new Translation2d(kSwerveWheelDistance_x/2.0, -kSwerveWheelDistance_y/2.0),
        new Translation2d(-kSwerveWheelDistance_x/2.0, -kSwerveWheelDistance_y/2.0),
        new Translation2d(-kSwerveWheelDistance_x/2.0, kSwerveWheelDistance_y/2.0)
      }
    );

    // Motor & Sensor direction
    public static final InvertedValue kDriveDirection = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kTurnDirection = InvertedValue.CounterClockwise_Positive;
    public static final SensorDirectionValue kCANcoderDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static class CancoderOffsets {
      public static final double FrontLeft = 0.300537;
      public static final double FrontRight = 0.385986;
      public static final double BackRight = -0.204590;
      public static final double BackLeft = -0.287550;
    }

    public static class Configs {
      public static TalonFXConfiguration driveMotorConfig() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = kDriveDirection;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        return configs;
      }

      public static TalonFXConfiguration turnMotorConfig(int CANcoderID) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = kTurnDirection;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // configs.Feedback.FeedbackRemoteSensorID = CANcoderID;
        // configs.Feedback.RotorToSensorRatio = 1;
        configs.Slot0.kP = 0.1;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 0;

        return configs;
      }

      public static CANcoderConfiguration turnEncoderConfig(double offset) {
        CANcoderConfiguration configs = new CANcoderConfiguration();
        configs.MagnetSensor.SensorDirection = kCANcoderDirection;
        configs.MagnetSensor.MagnetOffset = offset;

        return configs;
      }
    }

    public static final PIDController XAutoAimPID = new PIDController(0.2, 0, 0);
    public static final PIDController YAutoAimPID = new PIDController(0.2, 0, 0);
  }

  public static class ClawConstants {
    public static final double MaxAngle = 135; // degree
    public static final double MinAngle = 0; // degree

    public static final double MaxPower = 0.5;
    public static final double MinPower = -0.5;

    public static final double GrabPower = 0.8;
    public static final double PutPower = 0.8;

    public static final int SensorPort = 0;
    public static final double SensorThreshold = 1350;
    public static final double GrabDelay = 0.3; //s
    public static final double PutDelay = 0.4; //s

    public static class  Configs {
      public static SparkMaxConfig getShaftConfig() {
        SparkMaxConfig configs = new SparkMaxConfig();
        configs
          .inverted(true)
          .idleMode(IdleMode.kBrake);
        configs.encoder
          .positionConversionFactor(360.0 / 80.0);
        configs.closedLoop
          .outputRange(MinPower, MaxPower)
          .positionWrappingInputRange(MaxAngle, MinAngle)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.05, 0, 0);

        return configs;
      }
    }
  }

  public static class ElevatorConstants{
    // Elevator Move Speed
    public static final double kElevatorUpSpeed = 1.0;
    public static final double kElevatorDownSpeed = -1.0;
  
    // Motor rotate rate (cm/rotations)
    public static final double kRotateRate = 11.43/15.0;
  
    // Shaft position limits (cm)
    public static final double kElevatorMinPosition = 0.0;
    public static final double kElevatorMaxPosition = 75.0;
  
    // Motor controller closed loop control pid (Elevator)
    public static final double kp = 0.1;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
  
    // Motor controller inverted settings
    public static final boolean kLeftElevatorInverted = true;
    public static final boolean kRightElevatorInverted = false;

      public static class Configs {
        public static SparkMaxConfig GetElevatorConfig(boolean inverted) {
          SparkMaxConfig config = new SparkMaxConfig();
          config
            .idleMode(IdleMode.kBrake)
            .inverted(inverted);
          config.encoder
            .positionConversionFactor(kRotateRate);
          config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kp, ki, kd)
            .outputRange(kElevatorDownSpeed, kElevatorUpSpeed)
            .positionWrappingInputRange(kElevatorMinPosition, kElevatorMaxPosition);

          return config;
        }
      }
    }

  public static class IntakeConstants {
    public static final double MaxAngle = 45.0;
    public static final double MinAngle = -10.0;
    public static final double MinPower= -0.4;
    public static final double MaxPower= 0.4;
    public static final double GrabPower= 0.4;

    public static class  Configs {
      public static SparkMaxConfig getShaftConfig() {
        SparkMaxConfig configs = new SparkMaxConfig();
        configs
          .inverted(true)
          .idleMode(IdleMode.kBrake);
        configs.encoder
          .positionConversionFactor(360.0 / 25.0);
        configs.closedLoop
          .outputRange(MinPower, MaxPower)
          .positionWrappingInputRange(MaxAngle, MinAngle)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.05, 0, 0);

        return configs;
      }

      public static SparkMaxConfig getGrabberConfig() {
        SparkMaxConfig configs = new SparkMaxConfig();
        configs
          .inverted(false)
          .idleMode(IdleMode.kBrake);

        return configs;
      }
    }
  }

  public static class LimelightConstants {
    /* device name */
    public static final String device = "limelight";

    /* config limelight offsets */
    public static final double X = 0.19; //m
    public static final double Z = 0.14; //m
    public static final double Y = 0.94; //m
    public static final double Pitch = -46; //degree
    public static final double Roll  = -5.0; //degree
    public static final double Yaw = 18.2; //degree

    public static final double X_Distance = 0.15; //m
    public static final double Y_Distance = 0.20; //m
    public static final double AutoAimDeadband = 0.01; //m
  }
}
