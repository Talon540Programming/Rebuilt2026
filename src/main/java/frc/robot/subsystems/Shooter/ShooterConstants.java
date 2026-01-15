package frc.robot.subsystems.Shooter;

public class ShooterConstants {
     // Flywheel motors (2x X60 on same mechanism)
    public static final int kFlywheelMotor1Id = 17;
    public static final int kFlywheelMotor2Id = 18;
    
    // Hood motor (X44 rack and pinion)
    public static final int kHoodMotorId = 19;
    
    // Hood angle limits (radians)
    public static final double kHoodMinAngle = Math.PI / 8;  // 22.5 degrees
    public static final double kHoodMaxAngle = Math.PI / 2;  // 90 degrees
    
    // Flywheel PID constants - tune on real robot
    public static final double kFlywheelkP = 0.1;
    public static final double kFlywheelkI = 0.0;
    public static final double kFlywheelkD = 0.0;
    public static final double kFlywheelkV = 0.12; // Feedforward
    
    // Hood PID constants - tune on real robot
    public static final double kHoodkP = 1.0;
    public static final double kHoodkI = 0.0;
    public static final double kHoodkD = 0.0;

    public static final class KickupConstants {
    public static final int kKickupMotorId = 16;
    
    public static final double kFeedDutyCycle = 0.8;
    public static final double kReverseDutyCycle = -0.3;
  }
}
