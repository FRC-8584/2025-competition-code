package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int Player1Port = 0;
    public static final int Player2Port = 1;


    // Robot Move & Turn Speed
    public static final double kMove = 0.6;
    public static final double kTrun = 0.6;

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
  public static class FieldConstants {
  }

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

    public static final int Intake1ID         =  9;
    public static final int Intake2ID         = 10;
    public static final int TransferID        = 11;
    public static final int LShooterID        = 12;
    public static final int RShooterID        = 13;

    public static final int LChuteID          = 14;
    public static final int RChuteID          = 15;
  }

  /***** motor speed constants *****/
  public static class MotorConstants {
  }
}
