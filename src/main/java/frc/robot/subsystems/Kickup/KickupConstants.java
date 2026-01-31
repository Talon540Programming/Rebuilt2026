package frc.robot.subsystems.Kickup;

/**
 * Constants for the Kickup subsystem.
 * 
 * The kickup wheels index FUEL into the shooter from the hopper.
 * It uses one X60 Kraken motor with MotionMagic velocity control for smooth
 * and fast indexing into the shooter mechanism.
 * 
 * Values should be tuned during testing and robot characterization.
 */
public final class KickupConstants {
    
    // ========== CAN IDs ==========
    /** CAN ID for the kickup wheel motor (X60) */
    public static final int MOTOR_ID = 34; // TODO: Set actual CAN ID
    
    /** CAN bus name for kickup motor */
    public static final String CAN_BUS_NAME = "Default Name";
    
    
    // ========== Gear Ratio ==========
    /**
     * Gear ratio for the kickup motor.
     * Ratio of motor rotations to wheel rotations.
     * TODO: Measure and update actual gear ratio
     */
    public static final double GEAR_RATIO = 1.0;
    
    
    // ========== Velocity Targets ==========
    /** Target velocity when feeding FUEL into shooter (rotations per second) */
    public static final double FEED_VELOCITY_RPS = 80.0; // TODO: Tune - should be fast
    
    /** Target velocity for slow feeding (rotations per second) */
    public static final double SLOW_FEED_VELOCITY_RPS = 40.0; // TODO: Tune
    
    
    // ========== MotionMagic Velocity Configuration ==========
    /** MotionMagic acceleration (rotations per second^2) */
    public static final double ACCELERATION = 500.0; // TODO: Tune - should accelerate quickly
    
    /** MotionMagic jerk (rotations per second^3) - smoothness of acceleration */
    public static final double JERK = 5000.0; // TODO: Tune
    
    
    // ========== PID Constants ==========
    /** PID slot for MotionMagic velocity control */
    public static final int PID_SLOT = 0;
    
    /** Proportional gain for velocity control */
    public static final double kP = 0.1; // TODO: Tune this value
    
    /** Integral gain for velocity control */
    public static final double kI = 0.0; // TODO: Tune this value
    
    /** Derivative gain for velocity control */
    public static final double kD = 0.0; // TODO: Tune this value
    
    /** Velocity feedforward */
    public static final double kV = 0.12; // TODO: Tune with SysID
    
    /** Static friction feedforward */
    public static final double kS = 0.0; // TODO: Tune with SysID
    
    /** Acceleration feedforward */
    public static final double kA = 0.0; // TODO: Tune with SysID
    
    
    // ========== Current Limits ==========
    /** Supply current limit (A) */
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    
    /** Enable supply current limit */
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    
    /** Stator current limit (A) */
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    
    /** Enable stator current limit */
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
    
    
    // ========== Motor Configuration ==========
    /** Use brake mode for kickup motor (false = coast to reduce wear) */
    public static final boolean USE_BRAKE_MODE = false;
    
    /** Invert kickup motor */
    public static final boolean INVERT_MOTOR = false; // TODO: Verify during testing
    
    
    // ========== Tolerances ==========
    /** Velocity tolerance for at-target check (rps) */
    public static final double VELOCITY_TOLERANCE = 5.0;
    
    
    // ========== Simulation Constants ==========
    /** Simulated moment of inertia for kickup wheels (kg*m^2) */
    public static final double SIM_MOI = 0.02; // TODO: Calculate actual MOI
    
    
    private KickupConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}