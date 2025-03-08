// package frc.robot.commands.swerve;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.LimelightConstants;
// import frc.robot.Constants.Reef;
// import frc.robot.subsystems.Swerve;
// import frc.robot.utils.LimelightHelpers;
// import frc.robot.utils.Tools;

// public class MoveToReef_V2 extends Command {
//     private Swerve swerve;
//     private boolean x_finish, z_finish, turn_finish;
//     private final double x_c, z_c, turn_c;
//     private double x_power, z_power, turn_power;

//     public MoveToReef_V2(Swerve swerve, Reef reef) {
//         this.swerve = swerve;
//         if(reef == Reef.Right) {
//             x_c = LimelightConstants.RightTxInCameraSpace;
//             z_c = LimelightConstants.RightTzInCameraSpace;
//             turn_c = LimelightConstants.RightYawInCameraSpace;
//         }
//         else if(reef == Reef.Left) {
//             x_c = LimelightConstants.LeftTxInCameraSpace;
//             z_c = LimelightConstants.LeftTzInCameraSpace;
//             turn_c = LimelightConstants.LeftYawInCameraSpace;
//         }
//         else {
//             x_c = LimelightConstants.MiddleTxInCameraSpace;
//             z_c = LimelightConstants.MiddleTzInCameraSpace;
//             turn_c = LimelightConstants.MiddleYawInCameraSpace;
//         }
//         addRequirements(this.swerve);
//     }

//     @Override
//     public void initialize() {
//         x_finish = false;
//         z_finish = false;
//         turn_finish = false;
//     }

//     @Override
//     public void execute() {
//         double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.device);
//         if(!turn_finish){
//             turn_power = Tools.deadband((pose[4] - turn_c)/20.0, 0.08);
//             if(turn_power == 0) turn_finish = true;
//         }
//         else{
//             if(!x_finish) {
//                 x_power = Tools.deadband((pose[0] - x_c)/0.80, 0.08);
//                 if(x_power == 0) x_finish = true;
//             }
//             else {
//                 z_power = Tools.deadband((pose[0] - z_c)/0.80, 0.08);
//                 if(z_power == 0) z_finish = true;
//             }
//         }
//         swerve.drive(z_power, x_power, turn_power, false);
//     }

//     @Override
//     public boolean isFinished() {
//         if(turn_finish && x_finish && z_finish) return true;
//         else return false;
//     }
// }
