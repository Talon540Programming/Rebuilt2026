package frc.robot.utility;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.PassingConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

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
        theta = MathUtil.clamp((Math.PI/2) - theta, ShootingConstants.hoodMinAngle, ShootingConstants.hoodMaxAngle);
        
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
        // Get shooter position for accurate distance calculation
        Translation2d actualGoal = hubPose.getTranslation();
        Translation2d shooterPos = getShooterPosition(robotPose);
        
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
            double newDist = testGoal.getDistance(shooterPos);
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
        Translation2d shooterPos = getShooterPosition(robotPose);
        double distance = virtualGoal.getDistance(shooterPos);
        double hoodAngle = calculateHoodAngle(distance);
        double flywheelRPM = calculateFlywheelRPM(distance);
        
        return new ShootingSolution(hoodAngle, flywheelRPM, distance, virtualGoal);
    }

    /**
 * Get the shooter's position in field coordinates.
 * Accounts for shooter offset from robot center.
 * 
 * @param robotPose Current robot pose (center of robot)
 * @return Shooter position in field coordinates
 */
public static Translation2d getShooterPosition(Pose2d robotPose) {
    // Convert shooter offset from inches to meters
    double offsetXMeters = Units.inchesToMeters(ShootingConstants.shooterOffsetXInches.get());
    double offsetYMeters = Units.inchesToMeters(ShootingConstants.shooterOffsetYInches.get());
    
    // Transform from robot-relative to field-relative
    Transform2d shooterOffset = new Transform2d(
        new Translation2d(offsetXMeters, offsetYMeters),
        new Rotation2d()
    );
    
    Pose2d shooterPose = robotPose.transformBy(shooterOffset);
    
    Logger.recordOutput("Shooting/ShooterPosition", shooterPose.getTranslation());
    
    return shooterPose.getTranslation();
}

    /**
     * Calculate the heading offset needed to point the shooter at the target.
     * Because the shooter is offset laterally, the robot needs to rotate slightly
     * so the shooter (not robot center) points at the hub.
     * 
     * @param robotPose Current robot pose
     * @param target Target position (hub or virtual goal)
     * @return Heading the robot should face (in radians)
     */
    public static double calculateAimingHeading(Pose2d robotPose, Translation2d target) {
        Translation2d shooterPos = getShooterPosition(robotPose);
        
        // Calculate angle from shooter to target
        double dx = target.getX() - shooterPos.getX();
        double dy = target.getY() - shooterPos.getY();
        double angleToTarget = Math.atan2(dy, dx);
        
        Logger.recordOutput("Shooting/AimingHeadingDeg", Math.toDegrees(angleToTarget));
        
        return angleToTarget;
    }
    
    /**
     * Calculate distance from shooter (not robot center) to hub center.
     * 
     * @param robotPose Current robot pose
     * @param isRedAlliance True if on red alliance
     * @return Distance in meters from shooter to hub
     */
    public static double getDistanceToHub(Pose2d robotPose, boolean isRedAlliance) {
        Pose2d hubPose = isRedAlliance ? FieldPoses.redHub : FieldPoses.blueHub;
        
        // Use shooter position, not robot center
        Translation2d shooterPos = getShooterPosition(robotPose);
        
        double dx = hubPose.getX() - shooterPos.getX();
        double dy = hubPose.getY() - shooterPos.getY();
        
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

    // ==================== PASSING SHOT CALCULATIONS ====================
    
    /**
     * Container class for passing solution (hood angle + flywheel RPM + time of flight).
     */
    public static class PassingSolution {
        public final double hoodAngleRadians;
        public final double flywheelRPM;
        public final double distanceMeters;
        public final double timeOfFlightSeconds;
        public final boolean isValid;
        
        public PassingSolution(double hoodAngleRadians, double flywheelRPM, double distanceMeters, 
                double timeOfFlightSeconds, boolean isValid) {
            this.hoodAngleRadians = hoodAngleRadians;
            this.flywheelRPM = flywheelRPM;
            this.distanceMeters = distanceMeters;
            this.timeOfFlightSeconds = timeOfFlightSeconds;
            this.isValid = isValid;
        }
        
        public double getHoodAngleDegrees() {
            return Math.toDegrees(hoodAngleRadians);
        }
    }
    
    /**
     * Calculate the complete passing solution for lobbing the ball to the alliance wall.
     * Uses minimum angle that clears the trench, with fallback to steeper angles if needed.
     * 
     * @param robotPose Current robot pose
     * @param isRedAlliance True if on red alliance
     * @return PassingSolution containing hood angle, flywheel RPM, and validity
     */
    public static PassingSolution calculatePassingSolution(Pose2d robotPose, boolean isRedAlliance) {
        Translation2d shooterPos = getShooterPosition(robotPose);
        
        // Determine target and trench positions based on alliance
        double landingX = isRedAlliance ? PassingConstants.redLandingZoneX : PassingConstants.blueLandingZoneX;
        double trenchX = isRedAlliance ? PassingConstants.redTrenchX : PassingConstants.blueTrenchX;
        
        // Calculate horizontal distances (all positive values)
        double shooterX = shooterPos.getX();
        double totalDistance = Math.abs(shooterX - landingX);
        double distanceToTrench = Math.abs(shooterX - trenchX);
        
        // Convert heights to meters for consistency
        double launchHeightMeters = Units.inchesToMeters(ShootingConstants.shooterHeightInches);
        double trenchClearanceMeters = Units.inchesToMeters(PassingConstants.totalTrenchClearanceInches);
        
        // Log inputs
        Logger.recordOutput("Passing/ShooterX", shooterX);
        Logger.recordOutput("Passing/LandingX", landingX);
        Logger.recordOutput("Passing/TrenchX", trenchX);
        Logger.recordOutput("Passing/TotalDistanceMeters", totalDistance);
        Logger.recordOutput("Passing/DistanceToTrenchMeters", distanceToTrench);
        
        // Find the optimal angle (minimum angle that clears trench)
        double optimalAngle = findOptimalPassingAngle(totalDistance, distanceToTrench, 
                launchHeightMeters, trenchClearanceMeters);
        
        if (Double.isNaN(optimalAngle)) {
            Logger.recordOutput("Passing/Valid", false);
            Logger.recordOutput("Passing/Error", "No valid angle found");
            return new PassingSolution(0, 0, totalDistance, 0, false);
        }
        
        // optimalAngle is the HOOD angle, convert to launch angle for calculations
        double launchAngle = hoodAngleToLaunchAngle(optimalAngle);
        
        // Calculate required velocity for this launch angle
        double velocityMps = calculatePassingVelocity(totalDistance, launchAngle, launchHeightMeters);
        
        // Convert velocity to flywheel RPM
        double rpm = velocityToFlywheelRPM(velocityMps);
        
        // Check if RPM is within limits
        boolean isValid = true;
        if (rpm > ShootingConstants.maxFlywheelRPM) {
            // Try to find a different angle that reduces required velocity
            optimalAngle = findFallbackPassingAngle(totalDistance, distanceToTrench,
                    launchHeightMeters, trenchClearanceMeters);
            
            if (Double.isNaN(optimalAngle)) {
                Logger.recordOutput("Passing/Valid", false);
                Logger.recordOutput("Passing/Error", "RPM too high, no fallback angle");
                return new PassingSolution(0, 0, totalDistance, 0, false);
            }
            
            // Recalculate with new hood angle
            launchAngle = hoodAngleToLaunchAngle(optimalAngle);
            velocityMps = calculatePassingVelocity(totalDistance, launchAngle, launchHeightMeters);
            rpm = velocityToFlywheelRPM(velocityMps);
            
            if (rpm > ShootingConstants.maxFlywheelRPM || rpm < ShootingConstants.minFlywheelRPM) {
                isValid = false;
            }
        }
        
        rpm = MathUtil.clamp(rpm, ShootingConstants.minFlywheelRPM, ShootingConstants.maxFlywheelRPM);
        
        // Calculate time of flight using launch angle
        double horizontalVelocity = velocityMps * Math.cos(launchAngle);
        double timeOfFlight = horizontalVelocity > 0 ? totalDistance / horizontalVelocity : 0;
        
        // Log outputs
        Logger.recordOutput("Passing/Valid", isValid);
        Logger.recordOutput("Passing/HoodAngleDeg", Math.toDegrees(optimalAngle));
        Logger.recordOutput("Passing/FlywheelRPM", rpm);
        Logger.recordOutput("Passing/VelocityMps", velocityMps);
        Logger.recordOutput("Passing/TimeOfFlight", timeOfFlight);
        
        return new PassingSolution(optimalAngle, rpm, totalDistance, timeOfFlight, isValid);
    }
    
   /**
     * Find the optimal hood angle that clears the trench with minimum time of flight.
     * Since hood angle 0 = straight up (90° launch) and hood max = flattest shot,
     * we search from max hood angle (flattest) to min hood angle (steepest) to find
     * the flattest trajectory that still clears the trench.
     * 
     * @param totalDistance Horizontal distance to landing zone (meters)
     * @param distanceToTrench Horizontal distance to trench (meters)
     * @param launchHeight Height of shooter from ground (meters)
     * @param trenchClearanceHeight Required clearance height at trench (meters)
     * @return Optimal hood angle in radians, or NaN if no valid angle exists
     */
    private static double findOptimalPassingAngle(double totalDistance, double distanceToTrench,
            double launchHeight, double trenchClearanceHeight) {
        
        double gravity = Units.inchesToMeters(ShootingConstants.gravityInchesPerSecSq);
        
        // Search from max hood angle (flattest) to min hood angle (steepest)
        // This finds the minimum time of flight trajectory that clears the trench
        double angleStep = Math.toRadians(0.5); // 0.5 degree steps
        
        for (double hoodAngle = ShootingConstants.hoodMaxAngle; hoodAngle >= ShootingConstants.hoodMinAngle; hoodAngle -= angleStep) {
            // Convert hood angle to launch angle (hood 0 = 90° launch, hood 54° = 36° launch)
            double launchAngle = hoodAngleToLaunchAngle(hoodAngle);
            
            // Calculate velocity needed to land at target with this launch angle
            double velocity = calculatePassingVelocity(totalDistance, launchAngle, launchHeight);
            
            if (Double.isNaN(velocity) || velocity <= 0) {
                continue;
            }
            
            // Check if this trajectory clears the trench
            double heightAtTrench = calculateHeightAtDistance(distanceToTrench, launchAngle, velocity, launchHeight, gravity);
            
            if (heightAtTrench >= trenchClearanceHeight) {
                // Return the HOOD angle (not launch angle)
                return hoodAngle;
            }
        }
        
        return Double.NaN; // No valid angle found
    }
    
    /**
     * Find a fallback hood angle when the flattest angle requires too much velocity.
     * Searches for the hood angle that minimizes required velocity while still clearing trench.
     * 
     * @return Fallback hood angle in radians, or NaN if none found
     */
    private static double findFallbackPassingAngle(double totalDistance, double distanceToTrench,
            double launchHeight, double trenchClearanceHeight) {
        
        double gravity = Units.inchesToMeters(ShootingConstants.gravityInchesPerSecSq);
        double bestHoodAngle = Double.NaN;
        double lowestRPM = Double.MAX_VALUE;
        
        double angleStep = Math.toRadians(0.5);
        
        for (double hoodAngle = ShootingConstants.hoodMinAngle; hoodAngle <= ShootingConstants.hoodMaxAngle; hoodAngle += angleStep) {
            // Convert hood angle to launch angle
            double launchAngle = hoodAngleToLaunchAngle(hoodAngle);
            
            double velocity = calculatePassingVelocity(totalDistance, launchAngle, launchHeight);
            
            if (Double.isNaN(velocity) || velocity <= 0) {
                continue;
            }
            
            // Check trench clearance
            double heightAtTrench = calculateHeightAtDistance(distanceToTrench, launchAngle, velocity, launchHeight, gravity);
            
            if (heightAtTrench >= trenchClearanceHeight) {
               // Calculate RPM for this velocity
                double rpm = velocityToFlywheelRPM(velocity);
                
                if (rpm < lowestRPM && rpm <= ShootingConstants.maxFlywheelRPM) {
                    lowestRPM = rpm;
                    bestHoodAngle = hoodAngle;
                }
            }
        }
        
        return bestHoodAngle;
    }
    
    /**
     * Calculate the required launch velocity to hit a target at a given distance and angle.
     * Accounts for launching above ground level and landing at ground level.
     * 
     * @param distance Horizontal distance to target (meters)
     * @param angle Launch angle (radians)
     * @param launchHeight Height of launch point (meters)
     * @return Required velocity in m/s, or NaN if impossible
     */
    private static double calculatePassingVelocity(double distance, double angle, double launchHeight) {
        double gravity = Units.inchesToMeters(ShootingConstants.gravityInchesPerSecSq);
        double cosAngle = Math.cos(angle);
        double tanAngle = Math.tan(angle);
        
        // Projectile motion equation solved for initial velocity
        // Landing at ground level (y = 0) from height h:
        // 0 = h + x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
        // Solving for v:
        // v² = (g*x²) / (2*cos²(θ)*(h + x*tan(θ)))
        
        double numerator = gravity * distance * distance;
        double denominator = 2 * cosAngle * cosAngle * (launchHeight + distance * tanAngle);
        
        if (denominator <= 0) {
            return Double.NaN;
        }
        
        return Math.sqrt(numerator / denominator);
    }
    
    /**
     * Calculate the height of the projectile at a given horizontal distance.
     * 
     * @param distance Horizontal distance from launch point (meters)
     * @param angle Launch angle (radians)
     * @param velocity Initial velocity (m/s)
     * @param launchHeight Initial height (meters)
     * @param gravity Gravitational acceleration (m/s²)
     * @return Height at the given distance (meters)
     */
    private static double calculateHeightAtDistance(double distance, double angle, double velocity,
            double launchHeight, double gravity) {
        double cosAngle = Math.cos(angle);
        double sinAngle = Math.sin(angle);
        
        // Time to reach this distance
        double horizontalVelocity = velocity * cosAngle;
        if (horizontalVelocity <= 0) {
            return Double.NEGATIVE_INFINITY;
        }
        double time = distance / horizontalVelocity;
        
        // Height at this time: y = y0 + vy*t - 0.5*g*t²
        double height = launchHeight + (velocity * sinAngle * time) - (0.5 * gravity * time * time);
        
        return height;
    }

    /**
     * Convert linear velocity to flywheel RPM.
     * 
     * @param velocityMps Velocity in meters per second
     * @return Flywheel RPM
     */
    private static double velocityToFlywheelRPM(double velocityMps) {
        double velocityFps = velocityMps * 3.28084; // m/s to ft/s
        double wheelRadiusFeet = ShootingConstants.flywheelRadiusInches / 12.0;
        double omegaRadPerSec = velocityFps / wheelRadiusFeet;
        return (omegaRadPerSec * 60.0) / (2.0 * Math.PI);
    }

    /**
     * Convert hood angle to launch angle.
     * Hood angle 0 = 90° launch (straight up), Hood angle 54° = 36° launch.
     * 
     * @param hoodAngleRadians Hood angle in radians
     * @return Launch angle in radians (from horizontal)
     */
    private static double hoodAngleToLaunchAngle(double hoodAngleRadians) {
        return (Math.PI / 2) - hoodAngleRadians;
    }
    
    /**
     * Calculate the heading the robot should face for passing (toward alliance wall).
     * 
     * @param isRedAlliance True if on red alliance
     * @return Heading in radians (toward alliance wall)
     */
    public static double calculatePassingHeading(boolean isRedAlliance) {
        // Blue alliance: face toward X = 0 (heading = π or 180°)
        // Red alliance: face toward X = field length (heading = 0°)
        if (isRedAlliance) {
            return 0.0; // Face toward positive X (red alliance wall)
        } else {
            return Math.PI; // Face toward negative X (blue alliance wall)
        }
    }
}