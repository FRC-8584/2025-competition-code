package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.modules.SwerveModule;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.SwerveConstants;

import frc.robot.utils.Tools;

public class Swerve extends SubsystemBase {
  private final SwerveModule LF;
  private final SwerveModule LR;
  private final SwerveModule RF;
  private final SwerveModule RR;

  private final AHRS gyro;

  private final Field2d field;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final double initial_angle;
  private final double invert;

	public Swerve() {
		LF = new SwerveModule(
			CAN_DeviceID.LF_TurnID,
			CAN_DeviceID.LF_DriveID,
			CAN_DeviceID.LF_CANcoderID,
			SwerveConstants.kLF_offset
		);

		LR = new SwerveModule(
			CAN_DeviceID.LR_TurnID,
			CAN_DeviceID.LR_DriveID,
			CAN_DeviceID.LR_CANcoderID,
			SwerveConstants.kLR_offset
		);

		RF = new SwerveModule(
			CAN_DeviceID.RF_TurnID,
			CAN_DeviceID.RF_DriveID,
			CAN_DeviceID.RF_CANcoderID,
			SwerveConstants.kRF_offset
		);

		RR = new SwerveModule(
			CAN_DeviceID.RR_TurnID,
			CAN_DeviceID.RR_DriveID,
			CAN_DeviceID.RR_CANcoderID,
			SwerveConstants.kRR_offset
		);

		gyro = new AHRS(NavXComType.kMXP_SPI);

		poseEstimator = new SwerveDrivePoseEstimator(
			SwerveConstants.kinematics,
			getGyroAngle(),
			new SwerveModulePosition[] {
				LF.getPosition(),
				RF.getPosition(),
				RR.getPosition(),
				LR.getPosition()
			},
			new Pose2d(0, 0, getGyroAngle())
		);

		field = new Field2d();
		invert = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? -1.0 : 1.0;
		initial_angle = gyro.getAngle();
	}

  @Override
  public void periodic() {
	  updateLoggingValue();
    updateOdometry();
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

	/**
   * Drive by "Axie"
   * @param x x value (-1 ~ 1)
   * @param y y value (-1 ~ 1)
   * @param turn turn value (-1 ~ 1)
   */
  public void drive(double x, double y, double turn, boolean fieldRelative) {
    // set the chassis speed (x, y, turn)
		ChassisSpeeds speeds = new ChassisSpeeds(
      Tools.deadband(x, 0.05) * SwerveConstants.kMaxDriveSpeed * SwerveConstants.kDriveSpeed * invert,
      Tools.deadband(y, 0.05) * SwerveConstants.kMaxDriveSpeed * SwerveConstants.kDriveSpeed * invert,
      Tools.deadband(turn, 0.05) * SwerveConstants.kMaxTurnSpeed * SwerveConstants.kTurnSpeed
    );
    drive(speeds, fieldRelative);
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
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle()) : speeds
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxDriveSpeed);

    LF.setState(states[0]);
    RF.setState(states[1]);
    RR.setState(states[2]);
    LR.setState(states[3]);
  }

  private Rotation2d getGyroAngle() {
    double angle = (360 - (gyro.getAngle() - initial_angle)) % 360.0;
    if (angle > 180) angle -= 360;
    if (angle <-180) angle += 360;
    return Rotation2d.fromDegrees(angle);
  }

  private void updateOdometry() {
    poseEstimator.update(
      getGyroAngle(),
      new SwerveModulePosition[]{
        LF.getPosition(),
        RF.getPosition(),
        RR.getPosition(),
        LR.getPosition()
			}
		);
  }

	private void updateLoggingValue() {
    SmartDashboard.putData("Field", field);
	}
}
