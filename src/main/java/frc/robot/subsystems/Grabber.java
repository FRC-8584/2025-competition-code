package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants.CAN_DeviceID;
import frc.robot.Constants.GrabberConstants;
import frc.robot.utils.Tools;

public class Grabber extends SubsystemBase {
  private final SparkMax Grabber_motor = new SparkMax(CAN_DeviceID.GrabberID, MotorType.kBrushless);

  private boolean isGetCoral;
  private boolean isGetAlgae;

  public Grabber() {
    isGetCoral = false;
    isGetAlgae = false;

    Grabber_motor.configure(
      GrabberConstants.getGrabberCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Grabber Get CORAL", isGetCoral);
    SmartDashboard.putBoolean("Grabber Get ALGAE", isGetAlgae);
  }

  /**
   * @param input
   * run the grapper
   * range: 1.0 ~ -1.0
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    Grabber_motor.set(input * GrabberConstants.kGrabberSpeed);
  }
}
