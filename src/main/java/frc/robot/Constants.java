package frc.robot;

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

  /***** motor controller ID *****/
  public static class MotorControllerID {
    public static final int LF_TurnID         =  1;
    public static final int LR_TurnID         =  2;
    public static final int RF_TurnID         =  3;
    public static final int RR_TurnID         =  4;

    public static final int LF_DriveID        =  5;
    public static final int LR_DriveID        =  6;
    public static final int RF_DriveID        =  7;
    public static final int RR_DriveID        =  8;

    public static final int L_ElevatorID      =  9;
    public static final int R_ElevatorID      =  10;

    public static final int WirstID           =  11;
    public static final int ClawID            =  12;
  }

  /***** motor speed constants *****/
  public static class MotorConstants {
    // Robot Move & Turn Speed
    public static final double kMove = 0.6;
    public static final double kTrun = 0.6;

    // Elevator Move Speed
    public static final double kElevatorSpd = 0.4;

    // Wirst & Claw Speed
    public static final double kWirstSpd = 0.5;
    public static final double kClawSpd = 0.8;
  }
}
