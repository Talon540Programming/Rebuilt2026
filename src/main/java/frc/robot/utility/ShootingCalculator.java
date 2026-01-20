package frc.robot.utility;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

/**
 * Calculates the optimal hood angle and flywheel velocity for shooting
 * based on distance from the hub.
 * 
 * Uses projectile motion equations to find the trajectory that clears
 * the hub rim with specified clearance.
 */
public class ShootingCalculator {

    /**
     * Container class for shooting solution (hood angle + flywheel RPM).
     */
    public static class ShootingSolution {
        public final double hoodAngleRadians;
        public final double flywheelRPM;
        public final double distanceMeters;
        public final Translation2d virtualGoal;
        
        public ShootingSolution(double hoodAngleRadians, double flywheelRPM, double distanceMeters) {
            this.hoodAngleRadians = hoodAngleRadians;
            this.flywheelRPM = flywheelRPM;
            this.distanceMeters = distanceMeters;
            this.virtualGoal = null;
        }
        
        public ShootingSolution(double hoodAngleRadians, double flywheelRPM, double distanceMeters, 
                Translation2d virtualGoal) {
            this.hoodAngleRadians = hoodAngleRadians;
            this.flywheelRPM = flywheelRPM;
            this.distanceMeters = distanceMeters;
            this.virtualGoal = virtualGoal;
        }
        
        public double getHoodAngleDegrees() {
            return Math.toDegrees(hoodAngleRadians);
        }
    }
    
    /**
     * Calculate the optimal hood angle for a given distance from the hub.
     * 
     * @param distanceMeters Distance from robot to hub center in meters
     * @return Hood angle in radians
     */
    public static double calculateHoodAngle(double distanceMeters) {
        // Convert to inches for calculation (original formula uses inches)
        double distFromCenter = Units.metersToInches(distanceMeters);
        
        double backDistance = ShootingConstants.backDistanceInches;
        double maxHeight = ShootingConstants.maxHeightInches;
        double clearance = ShootingConstants.clearanceInches;
        
        // Calculate theta using trajectory formula
        double numerator = 2 * (maxHeight + Math.sqrt(clearance * maxHeight));
        double denominator = distFromCenter + backDistance;
        double theta = Math.atan(numerator / denominator);
        
        // Clamp to hood limits
        theta = MathUtil.clamp(theta, ShootingConstants.hoodMinAngle, ShootingConstants.hoodMaxAngle);
        
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
        
        double backDistance = ShootingConstants.backDistanceInches;
        double maxHeight = ShootingConstants.maxHeightInches;
        double clearance = ShootingConstants.clearanceInches;
        double gravity = ShootingConstants.gravityInchesPerSecSq;
        
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
        double wheelRadiusFeet = ShootingConstants.flywheelRadiusInches / 12.0;
        double omegaRadPerSec = velocityFPS / wheelRadiusFeet;
        double rpm = (omegaRadPerSec * 60.0) / (2.0 * Math.PI);
        
        // Clamp to safe RPM range
        rpm = MathUtil.clamp(rpm, ShootingConstants.minFlywheelRPM, ShootingConstants.maxFlywheelRPM);
        
        return rpm;
    }

    /**
     * Calculate the time of flight for the ball to reach the hub.
     * Uses projectile motion for an asymmetric trajectory (shooter lower than target).
     * 
     * @param distanceMeters Distance from robot to hub center in meters
     * @return Time of flight in seconds
     */
    public static double calculateTimeOfFlight(double distanceMeters) {
        Logger.recordOutput("Shooting/TOF_InputDistanceMeters", distanceMeters);
        // Convert to inches for calculation
        double distFromCenter = Units.metersToInches(distanceMeters);
        
        double backDistance = ShootingConstants.backDistanceInches;
        double totalHorizontalDist = distFromCenter + backDistance;
        
        // Get the hood angle and initial velocity for this distance
        double theta = calculateHoodAngle(distanceMeters);
        double velocityFPS = calculateInitialVelocityFPS(distanceMeters);
        double velocityInchesPerSec = velocityFPS * 12.0;
        
        // Horizontal velocity component (constant throughout flight)
        double vx = velocityInchesPerSec * Math.cos(theta);
        
        // Time = horizontal distance / horizontal velocity
        if (vx <= 0) {
            return 1.0; // Fallback to 1 second if something is wrong
        }
        
        double timeSeconds = totalHorizontalDist / vx;

        Logger.recordOutput("Shooting/TOF_HorizontalDistInches", totalHorizontalDist);
        Logger.recordOutput("Shooting/TOF_Theta", Math.toDegrees(theta));
        Logger.recordOutput("Shooting/TOF_VelocityFPS", velocityFPS);
        Logger.recordOutput("Shooting/TOF_VxInchesPerSec", vx);
        Logger.recordOutput("Shooting/TOF_Calculated", timeSeconds);
        
        return timeSeconds;
    }

    /**
     * Calculate the virtual goal position that compensates for robot movement.
     * Uses iterative convergence like 2022 FRC teams.
     * 
     * @param robotPose Current robot pose
     * @param robotVelocity Field-relative velocity (vx, vy in m/s)
     * @param isRedAlliance True if on red alliance
     * @return Virtual goal position as Translation2d
     */
    public static Translation2d calculateVirtualGoal(
            Pose2d robotPose, 
            ChassisSpeeds robotVelocity,
            boolean isRedAlliance) {
        
        Pose2d hubPose = isRedAlliance ? FieldPoses.redHub : FieldPoses.blueHub;
        Translation2d actualGoal = hubPose.getTranslation();
        
        // Initial distance estimate
        double dist = getDistanceToHub(robotPose, isRedAlliance);
        double shotTime = calculateTimeOfFlight(dist);
        
        Translation2d virtualGoal = actualGoal;
        
        // Iterative convergence (up to 5 iterations, exit early if converged)
        for (int i = 0; i < 5; i++) {
            // Calculate virtual goal by offsetting for robot velocity
            // We subtract velocity * time because we want to aim where the goal
            // "appears" to be relative to where we'll be when the ball arrives
            double virtualGoalX = actualGoal.getX() - shotTime * robotVelocity.vxMetersPerSecond * ShootingConstants.compensationFactorX.get();
            double virtualGoalY = actualGoal.getY() - shotTime * robotVelocity.vyMetersPerSecond * ShootingConstants.compensationFactorY.get();
            
            Translation2d testGoal = 
                new Translation2d(virtualGoalX, virtualGoalY);
            
            // Calculate new distance and shot time to virtual goal
            double newDist = testGoal.getDistance(robotPose.getTranslation());
            double newShotTime = calculateTimeOfFlight(newDist);
            
            // Check for convergence (within 10ms)
            if (Math.abs(newShotTime - shotTime) <= 0.010) {
                virtualGoal = testGoal;
                break;
            }
            
            // Update for next iteration
            shotTime = newShotTime;
            
            // On last iteration, use the result
            if (i == 4) {
                virtualGoal = testGoal;
            }
        }

        Logger.recordOutput("Shooting/TimeOfFlight", shotTime);
        Logger.recordOutput("Shooting/VirtualGoalOffset", actualGoal.getDistance(virtualGoal));
        
            return virtualGoal;
    }

    /**
     * Calculate complete shooting solution with shoot-while-moving compensation.
     * 
     * @param robotPose Current robot pose
     * @param robotVelocity Field-relative velocity
     * @param isRedAlliance True if on red alliance
     * @return ShootingSolution based on virtual goal distance
     */
    public static ShootingSolution calculateSolutionWithMovement(
            Pose2d robotPose, 
            ChassisSpeeds robotVelocity,
            boolean isRedAlliance) {
        
            Translation2d virtualGoal = 
            calculateVirtualGoal(robotPose, robotVelocity, isRedAlliance);
        
        // Calculate distance to VIRTUAL goal (not actual goal)
        double distance = virtualGoal.getDistance(robotPose.getTranslation());
        double hoodAngle = calculateHoodAngle(distance);
        double flywheelRPM = calculateFlywheelRPM(distance);
        
        return new ShootingSolution(hoodAngle, flywheelRPM, distance, virtualGoal);
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