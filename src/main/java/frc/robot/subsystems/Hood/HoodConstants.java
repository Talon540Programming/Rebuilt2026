package frc.robot.subsystems.Hood;

/**
 * Constants for the Hood subsystem.
 * 
 * The hood controls the shooter angle using a rack and pinion mechanism.
 * It uses one X44 Kraken motor with MotionMagic position control to smoothly adjust
 * the shooter angle between minimum (pi/8) and maximum (pi/2) positions.
 * 
 * Values should be tuned during testing and robot characterization.
 */
public final class HoodConstants {
    
    // ========== CAN IDs ==========
    /** CAN ID for the hood angle motor (X44) */
    public static final int MOTOR_ID = 33; // TODO: Set actual CAN ID
    
    /** CAN bus name for hood motor */
    public static final String CAN_BUS_NAME = "Default Name";
    
    
    // ========== Gear Ratio ==========
    /**
     * Gear ratio for the hood motor.
     * Ratio of motor rotations to mechanism rotations.
     * TODO: Measure and update actual gear ratio from rack and pinion
     */
    public static final double GEAR_RATIO = 1.0;
    
    
    // ========== Position Limits ==========
    /**
     * Minimum angle for hood (pi/8 radians = 22.5 degrees) in rotations.
     * This is the lowest shooter angle.
     */
    public static final double MIN_ANGLE_ROTATIONS = (Math.PI / 8.0) / (2.0 * Math.PI);
    
    /**
     * Maximum angle for hood (pi/2 radians = 90 degrees) in rotations.
     * This is the highest shooter angle.
     */
    public static final double MAX_ANGLE_ROTATIONS = (Math.PI / 2.0) / (2.0 * Math.PI);
    
    
    // ========== MotionMagic Position Configuration ==========
    /** MotionMagic cruise velocity (rotations per second) */
    public static final double CRUISE_VELOCITY = 1.5; // TODO: Tune this value
    
    /** MotionMagic acceleration (rotations per second^2) */
    public static final double ACCELERATION = 3.0; // TODO: Tune this value
    
    /** MotionMagic jerk (rotations per second^3) - smoothness of acceleration */
    public static final double JERK = 30.0; // TODO: Tune this value
    
    
    // ========== PID Constants ==========
    /** PID slot for MotionMagic position control */
    public static final int PID_SLOT = 0;
    
    /** Proportional gain for position control */
    public static final double kP = 20.0; // TODO: Tune this value
    
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
    
    /** Gravity feedforward (V) - accounts for weight of hood mechanism */
    public static final double kG = 0.0; // TODO: Tune this value
    
    
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
    /** Use brake mode for hood motor (true = hold position when stopped) */
    public static final boolean USE_BRAKE_MODE = true;
    
    /** Invert hood motor */
    public static final boolean INVERT_MOTOR = false; // TODO: Verify during testing
    
    
    // ========== Tolerances ==========
    /** Position tolerance for at-target check (rotations) */
    public static final double POSITION_TOLERANCE = 0.005;
    
    
    // ========== Simulation Constants ==========
    /** Simulated moment of inertia for hood (kg*m^2) */
    public static final double SIM_MOI = 0.05; // TODO: Calculate actual MOI
    
    /** Simulated minimum angle for hood (radians) */
    public static final double SIM_MIN_ANGLE_RAD = Math.PI / 8.0;
    
    /** Simulated maximum angle for hood (radians) */
    public static final double SIM_MAX_ANGLE_RAD = Math.PI / 2.0;
    
    /** Simulated length of hood arm for gravity calculation (meters) */
    public static final double SIM_ARM_LENGTH = 0.3; // TODO: Measure actual arm length
    
    /** Simulated mass of hood mechanism (kg) */
    public static final double SIM_ARM_MASS = 2.0; // TODO: Measure actual mass
    
    
    private HoodConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}