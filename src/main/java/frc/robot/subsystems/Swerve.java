package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.SwerveConstants;
import frc.robot.modules.SwerveModule;
import frc.robot.utils.Tools;

public class Swerve extends SubsystemBase {
	private final AHRS gyro;
	
  private final SwerveModule LF;
  private final SwerveModule LR;
  private final SwerveModule RF;
  private final SwerveModule RR;
	
  private final Field2d field;
	
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
	
  private final double initial_angle;
  private final double invert;
	
	public Swerve() {
		gyro = new AHRS(NavXComType.kMXP_SPI);
	
		LF = new SwerveModule(
			CAN_DeviceID.LF_TurnID,
			CAN_DeviceID.LF_DriveID,
			CAN_DeviceID.LF_CANcoderID,
			SwerveConstants.kLF_offset);
		
		LR = new SwerveModule(
			CAN_DeviceID.LR_TurnID,
			CAN_DeviceID.LR_DriveID,
			CAN_DeviceID.LR_CANcoderID,
			SwerveConstants.kLR_offset);
	
		RF = new SwerveModule(
			CAN_DeviceID.RF_TurnID,
			CAN_DeviceID.RF_DriveID,
			CAN_DeviceID.RF_CANcoderID,
			SwerveConstants.kRF_offset);
	
		RR = new SwerveModule(
			CAN_DeviceID.RR_TurnID,
			CAN_DeviceID.RR_DriveID,
			CAN_DeviceID.RR_CANcoderID,
			SwerveConstants.kRR_offset);
	
		final boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
	
		initial_angle = gyro.getAngle() + (isRedAlliance ? 180 : 0);
		invert = isRedAlliance ? -1.0 : 1.0; 
	
		kinematics = new SwerveDriveKinematics(
			new Translation2d[] {
				new Translation2d(SwerveConstants.kHeight/2.0, SwerveConstants.kWidth/2.0),
				new Translation2d(SwerveConstants.kHeight/2.0, -SwerveConstants.kWidth/2.0),
				new Translation2d(-SwerveConstants.kHeight/2.0, -SwerveConstants.kWidth/2.0),
				new Translation2d(-SwerveConstants.kHeight/2.0, SwerveConstants.kWidth/2.0)
			}
		);
	
		poseEstimator = new SwerveDrivePoseEstimator(
			kinematics,
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
	}

  @Override
  public void periodic() {
		updateLoggingValue();
    updateOdometry();
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

	/**
	 * Drive the swerve basis.
	 */
  public void drive(double x, double y, double turn, boolean fieldRelative) {
		x = invert * Tools.deadband(x, 0.05) * SwerveConstants.kDriveSpeed;
    y = invert * Tools.deadband(y, 0.05) * SwerveConstants.kDriveSpeed;
		turn = Tools.deadband(turn, 0.05) * SwerveConstants.kTurnSpeed;

    SwerveModuleState states[] = kinematics.toSwerveModuleStates(
      fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(x, y, turn, getGyroAngle()):
        new ChassisSpeeds(x, y, turn)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kSwerveMaxSpeed);
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
		// front_left.logging("lf");
    // front_right.logging("rf");
    // back_right.logging("rr");
    // back_left.logging("lr");
    SmartDashboard.putData("Field", field);
	}



}
