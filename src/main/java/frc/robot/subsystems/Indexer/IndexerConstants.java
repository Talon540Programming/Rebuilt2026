package frc.robot.subsystems.Indexer;

/**
 * Constants for the Indexer subsystem.
 * 
 * The indexer controls the hopper belts and rollers that move FUEL from the intake
 * through the hopper to the kickup wheels. It uses one X44 motor with MotionMagic
 * velocity control for smooth acceleration and consistent indexing speed.
 * 
 * Values should be tuned during testing and robot characterization.
 */
public final class IndexerConstants {
    
    // ========== CAN IDs ==========
    /** CAN ID for the indexer motor (X44) */
    public static final int MOTOR_ID = 32; // TODO: Set actual CAN ID
    
    /** CAN bus name for indexer motor */
    public static final String CAN_BUS_NAME = "Default Name";
    
    
    // ========== Gear Ratio ==========
    /**
     * Gear ratio for the indexer motor.
     * Ratio of motor rotations to belt/roller rotations.
     * TODO: Measure and update actual gear ratio
     */
    public static final double GEAR_RATIO = 1.0;
    
    
    // ========== Velocity Targets ==========
    /** Target velocity when indexing FUEL forward (rotations per second) */
    public static final double INDEX_VELOCITY_RPS = 40.0; // TODO: Tune this value
    
    /** Target velocity when feeding to shooter (rotations per second) */
    public static final double FEED_VELOCITY_RPS = 60.0; // TODO: Tune this value
    
    /** Target velocity when reversing/ejecting FUEL (rotations per second) */
    public static final double REVERSE_VELOCITY_RPS = -30.0; // TODO: Tune this value
    
    
    // ========== MotionMagic Velocity Configuration ==========
    /** MotionMagic acceleration (rotations per second^2) */
    public static final double ACCELERATION = 300.0; // TODO: Tune this value
    
    /** MotionMagic jerk (rotations per second^3) - smoothness of acceleration */
    public static final double JERK = 3000.0; // TODO: Tune this value
    
    
    // ========== PID Constants ==========
    /** PID slot for MotionMagic velocity control */
    public static final int PID_SLOT = 0;
    
    /** Proportional gain for velocity control */
    public static final double kP = 0.11; // TODO: Tune this value
    
    /** Integral gain for velocity control */
    public static final double kI = 0.0; // TODO: Tune this value
    
    /** Derivative gain for velocity control */
    public static final double kD = 0.0; // TODO: Tune this value
    
    /** Velocity feedforward */
    public static final double kV = 0.12; // TODO: Tune with SysID
    
    /** Static friction feedforward */
    public static final double kS = 0.25; // TODO: Tune with SysID
    
    /** Acceleration feedforward */
    public static final double kA = 0.01; // TODO: Tune with SysID
    
    
    // ========== Current Limits ==========
    /** Supply current limit (A) */
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    
    /** Enable supply current limit */
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    
    /** Stator current limit (A) */
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    
    /** Enable stator current limit */
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
    
    
    // ========== Motor Configuration ==========
    /** Use brake mode for indexer motor (false = coast to reduce wear) */
    public static final boolean USE_BRAKE_MODE = false;
    
    /** Invert indexer motor */
    public static final boolean INVERT_MOTOR = false; // TODO: Verify during testing
    
    
    // ========== Tolerances ==========
    /** Velocity tolerance for at-target check (rps) */
    public static final double VELOCITY_TOLERANCE = 5.0;
    
    
    // ========== Simulation Constants ==========
    /** Simulated moment of inertia for indexer belts/rollers (kg*m^2) */
    public static final double SIM_MOI = 0.015; // TODO: Calculate actual MOI
    
    
    private IndexerConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}