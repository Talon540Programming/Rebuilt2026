package frc.robot.subsystems.Shooter;

/**
 * Constants for the Shooter subsystem.
 * 
 * This class contains all configuration values for the shooter flywheel mechanism.
 * The shooter uses a leader-follower configuration where the bottom motor (leader)
 * controls velocity and the top motor (follower) mechanically follows.
 * Values should be tuned during testing and robot characterization.
 */
public final class ShooterConstants {
    
    // ========== CAN IDs ==========
    /** CAN ID for the leader (bottom) flywheel motor */
    public static final int LEADER_MOTOR_ID = 20; // TODO: Set actual CAN ID
    
    /** CAN ID for the follower (top) flywheel motor */
    public static final int FOLLOWER_MOTOR_ID = 21; // TODO: Set actual CAN ID
    
    /** CAN bus name for the shooter motors */
    public static final String CAN_BUS_NAME = "Default Name";
    
    
    // ========== Gear Ratios ==========
    /**
     * Gear ratio for the bottom (leader) flywheel.
     * Ratio of motor rotations to wheel rotations.
     * TODO: Measure and update actual gear ratio
     */
    public static final double BOTTOM_FLYWHEEL_GEAR_RATIO = 1.0;
    
    /**
     * Gear ratio for the top (follower) flywheel.
     * Top wheel is smaller (3x faster) than bottom wheel due to mechanical design.
     * This is configured in the SensorToMechanismRatio for velocity reporting.
     * TODO: Measure and update actual gear ratio
     */
    public static final double TOP_FLYWHEEL_GEAR_RATIO = 1.0 / 3.0;
    
    
    // ========== PID Constants (Leader Motor Only) ==========
    /**
     * Velocity PID slot for closed-loop control.
     * Slot 0 is used for shooter velocity control on the leader motor.
     * The follower motor does not use PID - it mechanically follows the leader.
     */
    public static final int VELOCITY_SLOT = 0;
    
    /** Proportional gain for velocity control (V per rps of error) */
    public static final double kP = 2; // TODO: Tune this value
    
    /** Integral gain for velocity control (V per rps*s of accumulated error) */
    public static final double kI = 0.0; // TODO: Tune this value
    
    /** Derivative gain for velocity control (V per rps/s of error derivative) */
    public static final double kD = 0.1; // TODO: Tune this value
    
    /** Velocity feedforward gain (V per rps) */
    public static final double kV = 0.12; // TODO: Tune this value with SysID
    
    /** Static friction feedforward (V to overcome static friction) */
    public static final double kS = 0.0; // TODO: Tune this value with SysID
    
    /** Acceleration feedforward (V per rps/s) */
    public static final double kA = 0.0; // TODO: Tune this value with SysID
    
    
    // ========== Current Limits ==========
    /** Supply current limit (A) - applies to both motors */
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    
    /** Enable supply current limit */
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    
    /** Stator current limit (A) - applies to both motors */
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    
    /** Enable stator current limit */
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
    
    
    // ========== Motor Configuration ==========
    /** Neutral mode for shooter motors (brake to maintain speed, coast to save power) */
    public static final boolean USE_BRAKE_MODE = false;
    
    /** Invert bottom (leader) motor */
    public static final boolean INVERT_BOTTOM_MOTOR = false; // TODO: Verify during testing
    
    /** Invert top (follower) motor - set true if it needs to spin opposite direction */
    public static final boolean INVERT_TOP_MOTOR = false; // TODO: Verify during testing
    
    
    // ========== Velocity Tolerances ==========
    /** Tolerance for velocity target on bottom flywheel (rps) */
    public static final double VELOCITY_TOLERANCE_RPS = 2.0;
    
    
    // ========== Simulation Constants ==========
    /** Simulated moment of inertia for bottom flywheel (kg*m^2) */
    public static final double SIM_BOTTOM_MOI = 0.05; // TODO: Calculate actual MOI
    
    /** Simulated moment of inertia for top flywheel (kg*m^2) - smaller wheel = less MOI */
    public static final double SIM_TOP_MOI = 0.05 / 3.0; // TODO: Calculate actual MOI
    
    /** Simulated gearing for X60 motor */
    public static final double SIM_GEARING = 1.0;
    
    
    private ShooterConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}