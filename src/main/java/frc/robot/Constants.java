package frc.robot;

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
}
