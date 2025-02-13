package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants;
import frc.robot.Constants.GrapperConstants;
import frc.robot.utils.Tools;

public class Grapper extends SubsystemBase {
  private final SparkMax Grapper_motor = new SparkMax(Constants.MotorControllerID.GrapperID, MotorType.kBrushless);

  public Grapper() {
    Grapper_motor.configure(
      GrapperConstants.getGrapperCfg(),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  /**
   * @param input
   * run the grapper
   * range: 1.0 ~ -1.0
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    Grapper_motor.set(input * GrapperConstants.kGrapperSpeed);
  }
}
