package frc.robot.subsystems.Wrist;

/**
 * Constants for the Wrist subsystem.
 * 
 * The wrist controls the 4-bar linkage rotation for the over-the-bumper intake system.
 * It uses one X60 Kraken motor with MotionMagic position control to smoothly move
 * between stowed (inside robot) and deployed (over bumper) positions.
 * 
 * Values should be tuned during testing and robot characterization.
 */
public final class WristConstants {
    
    // ========== CAN IDs ==========
    /** CAN ID for the wrist rotation motor (X60 Kraken) */
    public static final int MOTOR_ID = 30; // TODO: Set actual CAN ID
    
    /** CAN bus name for wrist motor */
    public static final String CAN_BUS_NAME = "Default Name";
    
    
    // ========== Gear Ratio ==========
    /**
     * Gear ratio for the wrist motor.
     * Ratio of motor rotations to mechanism rotations.
     * TODO: Measure and update actual gear ratio
     */
    public static final double GEAR_RATIO = 1.0;
    
    
    // ========== Position Limits ==========
    /**
     * Minimum angle for wrist (stowed position) in rotations.
     * This is the hard stop when intake is retracted inside the robot.
     * TODO: Measure actual minimum angle after physical testing
     */
    public static final double MIN_ANGLE_ROTATIONS = 0.0;
    
    /**
     * Maximum angle for wrist (deployed position) in rotations.
     * This is the hard stop when intake is extended over the bumper.
     * TODO: Measure actual maximum angle after physical testing
     */
    public static final double MAX_ANGLE_ROTATIONS = 0.25;
    
    
    // ========== MotionMagic Position Configuration ==========
    /** MotionMagic cruise velocity (rotations per second) */
    public static final double CRUISE_VELOCITY = 2.0; // TODO: Tune this value
    
    /** MotionMagic acceleration (rotations per second^2) */
    public static final double ACCELERATION = 4.0; // TODO: Tune this value
    
    /** MotionMagic jerk (rotations per second^3) - smoothness of acceleration */
    public static final double JERK = 40.0; // TODO: Tune this value
    
    
    // ========== PID Constants ==========
    /** PID slot for MotionMagic position control */
    public static final int PID_SLOT = 0;
    
    /** Proportional gain for position control */
    public static final double kP = 24.0; // TODO: Tune this value
    
    /** Integral gain for position control */
    public static final double kI = 0.0; // TODO: Tune this value
    
    /** Derivative gain for position control */
    public static final double kD = 0.1; // TODO: Tune this value
    
    /** Velocity feedforward */
    public static final double kV = 0.12; // TODO: Tune with SysID
    
    /** Static friction feedforward */
    public static final double kS = 0.25; // TODO: Tune with SysID
    
    /** Acceleration feedforward */
    public static final double kA = 0.01; // TODO: Tune with SysID
    
    /** Gravity feedforward (V) - accounts for weight of intake mechanism */
    public static final double kG = 0.0; // TODO: Tune this value
    
    
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
    /** Use brake mode for wrist motor (true = hold position when stopped) */
    public static final boolean USE_BRAKE_MODE = true;
    
    /** Invert wrist motor */
    public static final boolean INVERT_MOTOR = false; // TODO: Verify during testing
    
    
    // ========== Tolerances ==========
    /** Position tolerance for at-target check (rotations) */
    public static final double POSITION_TOLERANCE = 0.01;
    
    
    // ========== Simulation Constants ==========
    /** Simulated moment of inertia for wrist (kg*m^2) */
    public static final double SIM_MOI = 0.1; // TODO: Calculate actual MOI
    
    /** Simulated minimum angle for wrist (radians) */
    public static final double SIM_MIN_ANGLE_RAD = 0.0;
    
    /** Simulated maximum angle for wrist (radians) */
    public static final double SIM_MAX_ANGLE_RAD = Math.PI / 2.0;
    
    /** Simulated length of wrist arm for gravity calculation (meters) */
    public static final double SIM_ARM_LENGTH = 0.5; // TODO: Measure actual arm length
    
    /** Simulated mass of intake mechanism (kg) */
    public static final double SIM_ARM_MASS = 5.0; // TODO: Measure actual mass
    
    
    private WristConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}