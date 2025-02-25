package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utils.AxieOptimizer;

public class Constants {
    public static class OperationConstant {
        public static final AxieOptimizer[] axieOptimizers = 
            new AxieOptimizer[] {
                new AxieOptimizer(0.04),
                new AxieOptimizer(0.04),
                new AxieOptimizer(0.05)
            };
        public static final double TurnSpeed = 0.5;
        public static final double DriveSpeed = 0.7;
    }
    
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

    public static class SwerveConstants{
        public static final double WheelRadius = 0.05; //m
        public static final double WheelPerimeter = WheelRadius * 2 * Math.PI;  //m
        public static final double MaxDriveSpeed = 100.0 / 8.14 * WheelPerimeter; //m/s
        public static final double MaxTurnSpeed = (MaxDriveSpeed / (0.82 * Math.PI)) * Math.PI * 2; //rad/s

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d[] {
                new Translation2d(0.29, 0.29),
                new Translation2d(0.29, -0.29),
                new Translation2d(-0.29, -0.29),
                new Translation2d(-0.29, 0.29)
            }
        );

        public static class CancoderOffsets {
            public static final double FrontLeft = -0.468750;
            public static final double FrontRight = -0.380859;
            public static final double BackRight = -0.181641;
            public static final double BackLeft = -0.287598;
        }

        public static class Configs {
            public static TalonFXConfiguration driveMotorConfig() {
                TalonFXConfiguration configs = new TalonFXConfiguration();
                configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                return configs;
            }
            public static TalonFXConfiguration turnMotorConfig(int cancoderId) {
                TalonFXConfiguration configs = new TalonFXConfiguration();
                configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                configs.Slot0.kP = 0.1;
                configs.Slot0.kI = 0;
                configs.Slot0.kD = 0;
                configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                return configs;
            }
            public static CANcoderConfiguration turnEncoderConfig(double offset) {
                CANcoderConfiguration configs = new CANcoderConfiguration();
                configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                configs.MagnetSensor.MagnetOffset = offset;
                return configs;
            }
        }
    }
}
