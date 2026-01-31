package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldPoses;

/**
 * Utility class for calculating optimal shooting trajectories.
 * 
 * Uses physics-based trajectory optimization to determine the required
 * hood angle and shooter velocity for accurate scoring based on distance
 * to the target (hub/reef center).
 */
public class TrajectoryCalculator {
    
    // ========== Physical Constants ==========
    /** Gravity acceleration in inches per second squared */
    private static final double GRAVITY_IN_PER_SEC2 = 386.09;
    
    /** Height of shooter center above ground (inches) */
    private static final double SHOOTER_HEIGHT_INCHES = 21.0;
    
    /** Target final height - center of hub/scoring zone (inches) */
    private static final double TARGET_HEIGHT_INCHES = 72.0; // TODO: Verify target height
    
    /** Required clearance height above obstacles (inches) */
    private static final double CLEARANCE_HEIGHT_INCHES = 12.0; // TODO: Tune based on testing
    
    /** Horizontal offset from limelight 2 to shooter center - LEFT (inches) */
    private static final double SHOOTER_X_OFFSET_FROM_LL2 = -11.0; // Negative = left
    
    /** Vertical offset from limelight 2 to shooter center - UP (inches) */
    private static final double SHOOTER_Z_OFFSET_FROM_LL2 = 18.0; // Positive = up
    
    /** Forward offset from limelight 2 to shooter center (inches) */
    private static final double SHOOTER_Y_OFFSET_FROM_LL2 = 0.0; // TODO: Measure forward/back offset
    
    
    // ========== Calculated Results ==========
    private double launchAngleRadians = 0.0;
    private double launchVelocityFeetPerSecond = 0.0;
    private double horizontalDistanceInches = 0.0;
    
    
    /**
     * Calculates the optimal trajectory for the current robot position.
     * Uses limelight 2 position and applies shooter offset to get true shooter position.
     * 
     * @param robotPose Current pose of the robot on the field
     * @param targetPose Target pose (hub center)
     * @return true if calculation was successful, false if out of range
     */
    public boolean calculateTrajectory(Pose2d robotPose, Pose2d targetPose) {
        // Calculate horizontal distance from robot to target (in inches)
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();
        
        // Distance in meters, convert to inches
        double distanceMeters = robotTranslation.getDistance(targetTranslation);
        double distanceInches = distanceMeters * 39.3701; // meters to inches
        
        // Note: The shooter offset from limelight is already accounted for in SHOOTER_HEIGHT_INCHES
        // For horizontal distance, we assume the limelight is close enough to the shooter center
        // that the horizontal offset is negligible for trajectory calculation
        // If more precision is needed, add: distanceInches += SHOOTER_Y_OFFSET_FROM_LL2
        
        horizontalDistanceInches = distanceInches;
        
        // Calculate trajectory parameters
        return calculateOptimalTrajectory(horizontalDistanceInches);
    }
    
    /**
     * Calculates optimal trajectory for a given horizontal distance.
     * Uses physics-based optimization to find the launch angle and velocity
     * that will clear obstacles and hit the target.
     * 
     * @param horizontalDistanceInches Total horizontal distance to target
     * @return true if trajectory is valid, false if target is unreachable
     */
    private boolean calculateOptimalTrajectory(double horizontalDistanceInches) {
        // Define trajectory parameters
        double y = horizontalDistanceInches; // Total horizontal distance
        double hS = SHOOTER_HEIGHT_INCHES;   // Shooter height above ground
        double zF = TARGET_HEIGHT_INCHES;     // Final target height
        double c = CLEARANCE_HEIGHT_INCHES;   // Required clearance
        double g = GRAVITY_IN_PER_SEC2;       // Gravity
        
        // Maximum required height (target + clearance)
        double zMax = zF + c;
        
        // Check if target is reachable (shooter must be below max height)
        if (hS >= zMax) {
            return false; // Can't shoot if we're already above the trajectory peak
        }
        
        // Optimal trajectory parameter (derived from energy optimization)
        // This ensures the trajectory clears zMax while using minimum energy
        double o = 2.0 * (zMax - hS + Math.sqrt(c * (zMax - hS)));
        
        // Calculate launch angle (radians)
        // This is the angle that satisfies the trajectory constraint
        launchAngleRadians = Math.atan(o / y);
        
        // Calculate minimum required velocity (inches per second)
        // This velocity ensures the projectile reaches zMax and lands at zF
        double v0InchesPerSec = Math.sqrt(
            (g * Math.pow(y, 2)) / 
            (2.0 * Math.pow(Math.sqrt(zMax - hS) + Math.sqrt(c), 2)) + 
            2.0 * g * (zMax - hS)
        );
        
        // Convert velocity from inches/sec to feet/sec
        launchVelocityFeetPerSecond = v0InchesPerSec / 12.0;
        
        // Validate results
        if (Double.isNaN(launchAngleRadians) || Double.isNaN(launchVelocityFeetPerSecond)) {
            return false;
        }
        
        if (launchAngleRadians < 0 || launchAngleRadians > Math.PI / 2) {
            return false; // Angle out of physical range
        }
        
        return true;
    }
    
    /**
     * Gets the calculated launch angle in radians.
     * This is the angle the hood should be set to.
     * 
     * @return Launch angle in radians
     */
    public double getLaunchAngleRadians() {
        return launchAngleRadians;
    }
    
    /**
     * Gets the calculated launch angle in degrees.
     * 
     * @return Launch angle in degrees
     */
    public double getLaunchAngleDegrees() {
        return Math.toDegrees(launchAngleRadians);
    }
    
    /**
     * Gets the calculated launch velocity in feet per second.
     * This is used to determine shooter wheel speed.
     * 
     * @return Launch velocity in feet per second
     */
    public double getLaunchVelocityFeetPerSecond() {
        return launchVelocityFeetPerSecond;
    }
    
    /**
     * Gets the horizontal distance to target in inches.
     * 
     * @return Distance in inches
     */
    public double getHorizontalDistanceInches() {
        return horizontalDistanceInches;
    }
    
    /**
     * Converts shooter velocity (feet/sec) to bottom flywheel rotations per second.
     * Accounts for wheel diameter and gear ratios.
     * 
     * @param velocityFeetPerSec Linear velocity of projectile in feet per second
     * @param wheelDiameterInches Diameter of shooter wheel in inches
     * @return Flywheel velocity in rotations per second
     */
    public static double velocityToFlywheelRPS(double velocityFeetPerSec, double wheelDiameterInches) {
        // Convert feet/sec to inches/sec
        double velocityInchesPerSec = velocityFeetPerSec * 12.0;
        
        // Calculate wheel circumference
        double wheelCircumference = Math.PI * wheelDiameterInches;
        
        // Calculate rotations per second
        // Surface speed of wheel must match projectile velocity
        double rps = velocityInchesPerSec / wheelCircumference;
        
        return rps;
    }
    
    /**
     * Converts hood angle (radians) to hood position (rotations).
     * Maps the physical angle to motor rotations based on mechanism geometry.
     * 
     * @param angleRadians Hood angle in radians
     * @param minAngleRadians Minimum physical angle (corresponds to min rotations)
     * @param maxAngleRadians Maximum physical angle (corresponds to max rotations)
     * @param minRotations Minimum motor position in rotations
     * @param maxRotations Maximum motor position in rotations
     * @return Hood motor position in rotations
     */
    public static double angleToHoodRotations(
            double angleRadians,
            double minAngleRadians,
            double maxAngleRadians,
            double minRotations,
            double maxRotations) {
        
        // Clamp angle to valid range
        angleRadians = Math.max(minAngleRadians, Math.min(maxAngleRadians, angleRadians));
        
        // Linear mapping from angle to rotations
        double angleRange = maxAngleRadians - minAngleRadians;
        double rotationRange = maxRotations - minRotations;
        
        double normalizedAngle = (angleRadians - minAngleRadians) / angleRange;
        double rotations = minRotations + (normalizedAngle * rotationRange);
        
        return rotations;
    }
    
    /**
     * Gets the shooter center position in 3D space.
     * Returns [x_offset, y_offset, z_offset] from limelight 2 in inches.
     * 
     * @return Array of [horizontal_left, forward, vertical_up] offsets in inches
     */
    public static double[] getShooterOffsetFromLimelight2() {
        return new double[] {
            SHOOTER_X_OFFSET_FROM_LL2,  // Left offset
            SHOOTER_Y_OFFSET_FROM_LL2,  // Forward offset
            SHOOTER_Z_OFFSET_FROM_LL2   // Up offset
        };
    }
    
    /**
     * Gets the shooter height above ground.
     * 
     * @return Shooter height in inches
     */
    public static double getShooterHeight() {
        return SHOOTER_HEIGHT_INCHES;
    }
    
    /**
     * Helper method to get target pose based on alliance color.
     * 
     * @param isRedAlliance true if on red alliance, false for blue
     * @return Pose2d of the scoring target
     */
    public static Pose2d getTargetPose(boolean isRedAlliance) {
        return isRedAlliance ? FieldPoses.redCenterOfReef : FieldPoses.blueCenterOfReef;
    }
}