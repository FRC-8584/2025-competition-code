package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.utils.Tools;

public class Elevator extends SubsystemBase {
  private final SparkMax Lmotor = new SparkMax(Constants.MotorControllerID.L_ElevatorID, MotorType.kBrushless);
  private final SparkMax Rmotor = new SparkMax(Constants.MotorControllerID.R_ElevatorID, MotorType.kBrushless);

  private final SparkMaxConfig Lmotor_cfg = new SparkMaxConfig();
  private final SparkMaxConfig Rmotor_cfg = new SparkMaxConfig();
  
  private double position;
  private double L_pos, R_pos;
  private final double min_pos = 15;// cm
  private final double max_pos = 103;// cm
  private final double init_pos = 15;// cm
  private final double rotate_rate = 6;// rotations/cm

  public Elevator() {
    position = init_pos;
    L_pos = init_pos;
    R_pos = init_pos;

    Lmotor_cfg
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    Lmotor_cfg.encoder
      .positionConversionFactor(1/rotate_rate)// return position (cm)
      .velocityConversionFactor(1/rotate_rate/60.0);// return velocity (cm/s)
    Lmotor_cfg.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
      .pid(1, 0, 0);

    Rmotor_cfg
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    Rmotor_cfg.encoder
      .positionConversionFactor(1/rotate_rate)// return position (cm)
      .velocityConversionFactor(1/rotate_rate/60.0);// return velocity (cm/s)
    Rmotor_cfg.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)// build-in encoder
      .pid(1, 0, 0);

    Lmotor.configure(Lmotor_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Rmotor.configure(Rmotor_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    L_pos = Lmotor.getEncoder().getPosition();
    R_pos = Rmotor.getEncoder().getPosition();
    position = (L_pos + R_pos) / 2.0;
  }

  /**
   * @param setpoint
   * set elevator position (cm)
   */
  public void setPosition(double setpoint) {
    setpoint = Tools.bounding(setpoint, min_pos, max_pos);
    Lmotor.getClosedLoopController().setReference(setpoint - init_pos, ControlType.kPosition);
    Rmotor.getClosedLoopController().setReference(setpoint - init_pos, ControlType.kPosition);
  }

  /**
   * @param input
   * range: 1.0 ~ -1.0 (up ~ down)
   */
  public void setPower(double input) {
    input = Tools.bounding(input);
    Lmotor.set(input * Constants.MotorConstants.kElevatorSpd);
    Rmotor.set(input * Constants.MotorConstants.kElevatorSpd);
  }

  /**
   * @return elevator position (cm)
   */
  public double getPosition() {
    return position;
  }
}
