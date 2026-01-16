package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.ShootingConstants;

/**
 * Calculates the optimal hood angle and flywheel velocity for shooting
 * based on distance from the hub.
 * 
 * Uses projectile motion equations to find the trajectory that clears
 * the hub rim with specified clearance.
 */
public class ShootingCalculator {
    
    /**
     * Calculate the optimal hood angle for a given distance from the hub.
     * 
     * @param distanceMeters Distance from robot to hub center in meters
     * @return Hood angle in radians
     */
    public static double calculateHoodAngle(double distanceMeters) {
        // Convert to inches for calculation (original formula uses inches)
        double distFromCenter = Units.metersToInches(distanceMeters);
        
        double backDistance = ShootingConstants.kBackDistanceInches;
        double maxHeight = ShootingConstants.kMaxHeightInches;
        double clearance = ShootingConstants.kClearanceInches;
        
        // Calculate theta using trajectory formula
        double numerator = 2 * (maxHeight + Math.sqrt(clearance * maxHeight));
        double denominator = distFromCenter + backDistance;
        double theta = Math.atan(numerator / denominator);
        
        // Clamp to hood limits
        theta = MathUtil.clamp(theta, ShootingConstants.kHoodMinAngle, ShootingConstants.kHoodMaxAngle);
        
        return theta;
    }
    
    /**
     * Calculate the required initial velocity for a given distance from the hub.
     * 
     * @param distanceMeters Distance from robot to hub center in meters
     * @return Initial velocity in feet per second
     */
    public static double calculateInitialVelocityFPS(double distanceMeters) {
        // Convert to inches for calculation
        double distFromCenter = Units.metersToInches(distanceMeters);
        
        double backDistance = ShootingConstants.kBackDistanceInches;
        double maxHeight = ShootingConstants.kMaxHeightInches;
        double clearance = ShootingConstants.kClearanceInches;
        double gravity = ShootingConstants.kGravityInchesPerSecSq;
        
        // Calculate initial velocity using trajectory formula
        double numerator = gravity * Math.pow((distFromCenter + backDistance), 2);
        double denominator = 2 * Math.pow((Math.sqrt(maxHeight) + Math.sqrt(clearance)), 2);
        double termOne = numerator / denominator;
        double termTwo = 2 * gravity * maxHeight;
        double velocityInchesPerSec = Math.sqrt(termOne + termTwo);
        
        // Convert to feet per second
        return velocityInchesPerSec / 12.0;
    }
    
    /**
     * Calculate the required flywheel RPM for a given distance from the hub.
     * 
     * @param distanceMeters Distance from robot to hub center in meters
     * @return Flywheel velocity in RPM
     */
    public static double calculateFlywheelRPM(double distanceMeters) {
        double velocityFPS = calculateInitialVelocityFPS(distanceMeters);
        
        // Convert linear velocity (ft/s) to wheel RPM
        // v = ω * r, where v is linear velocity, ω is angular velocity, r is wheel radius
        // ω (rad/s) = v / r
        // RPM = ω * 60 / (2π)
        double wheelRadiusFeet = ShootingConstants.kFlywheelRadiusInches / 12.0;
        double omegaRadPerSec = velocityFPS / wheelRadiusFeet;
        double rpm = (omegaRadPerSec * 60.0) / (2.0 * Math.PI);
        
        // Clamp to safe RPM range
        rpm = MathUtil.clamp(rpm, ShootingConstants.kMinFlywheelRPM, ShootingConstants.kMaxFlywheelRPM);
        
        return rpm;
    }
    
    /**
     * Calculate distance from robot to hub center.
     * 
     * @param robotPose Current robot pose
     * @param isRedAlliance True if on red alliance
     * @return Distance in meters
     */
    public static double getDistanceToHub(Pose2d robotPose, boolean isRedAlliance) {
        Pose2d hubPose = isRedAlliance ? FieldPoses.redHub : FieldPoses.blueHub;
        
        double dx = hubPose.getX() - robotPose.getX();
        double dy = hubPose.getY() - robotPose.getY();
        
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Container class for shooting solution (hood angle + flywheel RPM).
     */
    public static class ShootingSolution {
        public final double hoodAngleRadians;
        public final double flywheelRPM;
        public final double distanceMeters;
        
        public ShootingSolution(double hoodAngleRadians, double flywheelRPM, double distanceMeters) {
            this.hoodAngleRadians = hoodAngleRadians;
            this.flywheelRPM = flywheelRPM;
            this.distanceMeters = distanceMeters;
        }
        
        public double getHoodAngleDegrees() {
            return Math.toDegrees(hoodAngleRadians);
        }
    }
    
    /**
     * Calculate complete shooting solution for current robot position.
     * 
     * @param robotPose Current robot pose
     * @param isRedAlliance True if on red alliance
     * @return ShootingSolution containing hood angle and flywheel RPM
     */
    public static ShootingSolution calculateSolution(Pose2d robotPose, boolean isRedAlliance) {
        double distance = getDistanceToHub(robotPose, isRedAlliance);
        double hoodAngle = calculateHoodAngle(distance);
        double flywheelRPM = calculateFlywheelRPM(distance);
        
        return new ShootingSolution(hoodAngle, flywheelRPM, distance);
    }
}