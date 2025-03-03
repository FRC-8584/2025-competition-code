package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.OperationConstant;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.CancoderOffsets;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {
    private final SwerveModule front_left, front_right, back_right, back_left;
    private final AHRS gyro;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    private final double invert, initial_angle;

    /**
     * Drive by "Axie"
     * @param x x value (-1 ~ 1)
     * @param y y value (-1 ~ 1)
     * @param turn turn value (-1 ~ 1)
     */
    public void drive(double x, double y, double turn, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(
            x * SwerveConstants.MaxDriveSpeed * OperationConstant.DriveSpeed * invert,
            y * SwerveConstants.MaxDriveSpeed * OperationConstant.DriveSpeed * invert,
            turn * SwerveConstants.MaxTurnSpeed * OperationConstant.TurnSpeed
        );
        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    getGyroAngle()
                ):
                speeds
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MaxDriveSpeed);
        front_left.setState(states[0]);
        front_right.setState(states[1]);
        back_right.setState(states[2]);
        back_left.setState(states[3]);
    }

    /**
     * Drive by "ChassisSpeed"
     * @param x vxMetersPerSecond
     * @param y vyMetersPerSecond
     * @param turn omegaRadiusPerSecond
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    getGyroAngle()
                ):
                speeds
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MaxDriveSpeed);
        front_left.setState(states[0]);
        front_right.setState(states[1]);
        back_right.setState(states[2]);
        back_left.setState(states[3]);
    }

    public void drive(SwerveModuleState[] states) {
        front_left.setState(states[0]);
        front_right.setState(states[1]);
        back_right.setState(states[2]);
        back_left.setState(states[3]);
    }

    private void updateOdometry() {
        poseEstimator.update(
            getGyroAngle(),
            new SwerveModulePosition[] {
                front_left.gePosition(),
                front_right.gePosition(),
                back_right.gePosition(),
                back_left.gePosition()
            });
    }

    private Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.kinematics.toChassisSpeeds(
            front_left.getState(),
            front_right.getState(),
            back_right.getState(),
            back_left.getState()
        );
    }

    public Rotation2d getGyroAngle() {
        double angle = (360 - (gyro.getAngle() - initial_angle)) % 360.0;
        if (angle > 180) angle -= 360;
        if (angle <-180) angle += 360;
        return Rotation2d.fromDegrees(angle);
    }

    private void configAuto() {
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            config = null;
            e.printStackTrace();
        }
        AutoBuilder.configure(
            this::getPose,
            this::resetPose, 
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> drive(speeds, false), 
            new PPHolonomicDriveController( 
                    new PIDConstants(5.0, 0.0, 0.0), 
                    new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this 
        );
    }

    public Swerve() {
        front_left = new SwerveModule(CAN_DeviceID.FL_TurnID, CAN_DeviceID.FL_DriveID, CAN_DeviceID.FL_CANcoderID, CancoderOffsets.FrontLeft);
        front_right = new SwerveModule(CAN_DeviceID.FR_TurnID, CAN_DeviceID.FR_DriveID, CAN_DeviceID.FR_CANcoderID, CancoderOffsets.FrontRight);
        back_right = new SwerveModule(CAN_DeviceID.BR_TurnID, CAN_DeviceID.BR_DriveID, CAN_DeviceID.BR_CANcoderID, CancoderOffsets.BackRight);
        back_left = new SwerveModule(CAN_DeviceID.BL_TurnID, CAN_DeviceID.BL_DriveID, CAN_DeviceID.BL_CANcoderID, CancoderOffsets.BackLeft);

        gyro = new AHRS(NavXComType.kMXP_SPI);
        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.kinematics,
            getGyroAngle(),
            new SwerveModulePosition[] {
                front_left.gePosition(),
                front_right.gePosition(),
                back_right.gePosition(),
                back_left.gePosition()
            }, new Pose2d(0, 0, getGyroAngle())
        );

        invert = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? -1.0: 1.0;
        initial_angle = gyro.getAngle() + ( DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? 180: 0);

        configAuto();
    }   

    @Override
    public void periodic() {
      updateOdometry();
      field.setRobotPose(getPose());
      SmartDashboard.putData("field",field);
      front_left.logging("fl");
    }
}