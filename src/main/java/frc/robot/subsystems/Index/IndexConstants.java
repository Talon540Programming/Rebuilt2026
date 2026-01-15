package frc.robot.subsystems.Index;

public class IndexConstants {
    public static final int kIndexMotorId = 15;
    public static final int kCANRangeId = 22;
    
    public static final double kIndexDutyCycle = 0.5;      // Normal indexing speed
    public static final double kReverseDutyCycle = -0.3;   // Reverse/eject speed
    public static final double kFeedDutyCycle = 0.8;       // Fast feed during shooting
    
    // CANRange detection threshold in meters - tune on real robot
    public static final double kGamePieceDetectionThreshold = 0.15;
}
