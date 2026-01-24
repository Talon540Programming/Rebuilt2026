package frc.robot.utility;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.HeadingPID;
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
     * Container class for passing solution (hood angle + flywheel RPM).
     */
    public static class PassingSolution {
        public final double hoodAngleRadians;
        public final double flywheelRPM;
        public final double distanceMeters;
        public final boolean isValid;
        
        public PassingSolution(double hoodAngleRadians, double flywheelRPM, double distanceMeters, boolean isValid) {
            this.hoodAngleRadians = hoodAngleRadians;
            this.flywheelRPM = flywheelRPM;
            this.distanceMeters = distanceMeters;
            this.isValid = isValid;
        }
        
        public double getHoodAngleDegrees() {
            return Math.toDegrees(hoodAngleRadians);
        }
    }
    
    /**
     * Calculate the passing solution using closed-form trajectory equations.
     * Finds the unique trajectory that clears the trench (with margin) and lands at the target.
     * 
     * @param robotPose Current robot pose
     * @param isRedAlliance True if on red alliance
     * @return PassingSolution containing hood angle, flywheel RPM, and validity
     */
    public static PassingSolution calculatePassingSolution(Pose2d robotPose, boolean isRedAlliance) {
        // Get trench X from hub position (they're at the same X)
        double trenchX = isRedAlliance ? FieldPoses.redHub.getX() : FieldPoses.blueHub.getX();
        
        // Calculate landing X from trench position and fixed distance
        // Blue: landing is toward X=0, so subtract. Red: landing is toward X=max, so add.
        double landingX = isRedAlliance 
            ? trenchX + PassingConstants.trenchToLandingDistanceMeters 
            : trenchX - PassingConstants.trenchToLandingDistanceMeters;
        
        // Get shooter X position
        Translation2d shooterPos = getShooterPosition(robotPose);
        double shooterX = shooterPos.getX();
        
        // Calculate distances (all positive)
        double distanceToTrench = Math.abs(shooterX - trenchX);
        double totalDistance = Math.abs(shooterX - landingX);
        
        // Heights in meters
        double launchHeight = Units.inchesToMeters(ShootingConstants.shooterHeightInches);
        double trenchClearanceHeight = Units.inchesToMeters(
            PassingConstants.trenchHeightInches + PassingConstants.trenchClearanceMarginInches.get()
        );
        double gravity = Units.inchesToMeters(ShootingConstants.gravityInchesPerSecSq);
        
        // Log inputs
        Logger.recordOutput("Passing/ShooterX", shooterX);
        Logger.recordOutput("Passing/TrenchX", trenchX);
        Logger.recordOutput("Passing/LandingX", landingX);
        Logger.recordOutput("Passing/DistanceToTrench", distanceToTrench);
        Logger.recordOutput("Passing/TotalDistance", totalDistance);
        
        // Check if robot is past the trench (no clearance needed)
        boolean trenchInPath = isRedAlliance 
            ? (shooterX < trenchX) 
            : (shooterX > trenchX);
        
        double launchAngle;
        double velocity;
        
        if (!trenchInPath) {
            // Robot is past trench or very close - use flattest angle (max hood angle)
            double hoodAngle = ShootingConstants.hoodMaxAngle;
            launchAngle = (Math.PI / 2) - hoodAngle;
            velocity = calculatePassingVelocity(totalDistance, launchAngle, launchHeight, gravity);
            
            Logger.recordOutput("Passing/TrenchInPath", false);
            Logger.recordOutput("Passing/HoodAngleDeg", Math.toDegrees(hoodAngle));
            Logger.recordOutput("Passing/VelocityMps", velocity);
            
            double rpm = velocityToFlywheelRPM(velocity);
            rpm = MathUtil.clamp(rpm, ShootingConstants.minFlywheelRPM, ShootingConstants.maxFlywheelRPM);
            boolean isValid = !Double.isNaN(velocity) && velocity > 0;
            
            Logger.recordOutput("Passing/FlywheelRPM", rpm);
            Logger.recordOutput("Passing/Valid", isValid);
            
            return new PassingSolution(hoodAngle, rpm, totalDistance, isValid);
        }
        
        Logger.recordOutput("Passing/TrenchInPath", true);
        
        // Closed-form solution for trajectory through two points:
        // Point 1: (distanceToTrench, trenchClearanceHeight)
        // Point 2: (totalDistance, 0)
        // Starting from: (0, launchHeight)
        //
        // tan(θ) = ((y₁ - h)*x₂ + h*x₁) / (x₁*(x₂ - x₁))
        
        double x1 = distanceToTrench;
        double y1 = trenchClearanceHeight;
        double x2 = totalDistance;
        double h = launchHeight;
        
        double numerator = (y1 - h) * x2 + h * x1;
        double denominator = x1 * (x2 - x1);
        
        // Check for invalid geometry
        if (denominator <= 0 || x2 <= x1) {
            Logger.recordOutput("Passing/Valid", false);
            Logger.recordOutput("Passing/Error", "Invalid geometry");
            return new PassingSolution(0, 0, totalDistance, false);
        }
        
        double tanTheta = numerator / denominator;
        launchAngle = Math.atan(tanTheta);
        
        // Convert launch angle to hood angle
        double hoodAngle = (Math.PI / 2) - launchAngle;
        
        // Clamp hood angle to mechanical limits
        hoodAngle = MathUtil.clamp(hoodAngle, ShootingConstants.hoodMinAngle, ShootingConstants.hoodMaxAngle);
        
        // Recalculate launch angle after clamping
        launchAngle = (Math.PI / 2) - hoodAngle;
        
        // Calculate required velocity
        velocity = calculatePassingVelocity(totalDistance, launchAngle, launchHeight, gravity);
        
        // Convert to RPM
        double rpm = velocityToFlywheelRPM(velocity);
        boolean isValid = !Double.isNaN(velocity) && velocity > 0 
            && rpm >= ShootingConstants.minFlywheelRPM 
            && rpm <= ShootingConstants.maxFlywheelRPM;
        
        rpm = MathUtil.clamp(rpm, ShootingConstants.minFlywheelRPM, ShootingConstants.maxFlywheelRPM);
        
        // Log outputs
        Logger.recordOutput("Passing/HoodAngleDeg", Math.toDegrees(hoodAngle));
        Logger.recordOutput("Passing/LaunchAngleDeg", Math.toDegrees(launchAngle));
        Logger.recordOutput("Passing/VelocityMps", velocity);
        Logger.recordOutput("Passing/FlywheelRPM", rpm);
        Logger.recordOutput("Passing/Valid", isValid);
        
        return new PassingSolution(hoodAngle, rpm, totalDistance, isValid);
    }
    
    /**
     * Calculate the required launch velocity to land at a given distance.
     * 
     * @param distance Horizontal distance to target (meters)
     * @param launchAngle Launch angle from horizontal (radians)
     * @param launchHeight Height of launch point (meters)
     * @param gravity Gravitational acceleration (m/s²)
     * @return Required velocity in m/s, or NaN if impossible
     */
    private static double calculatePassingVelocity(double distance, double launchAngle, 
            double launchHeight, double gravity) {
        double cosAngle = Math.cos(launchAngle);
        double tanAngle = Math.tan(launchAngle);
        
        double num = gravity * distance * distance;
        double denom = 2 * cosAngle * cosAngle * (launchHeight + distance * tanAngle);
        
        if (denom <= 0) {
            return Double.NaN;
        }
        
        return Math.sqrt(num / denom);
    }
    
    /**
     * Convert linear velocity to flywheel RPM.
     */
    private static double velocityToFlywheelRPM(double velocityMps) {
        double velocityFps = velocityMps * 3.28084;
        double wheelRadiusFeet = ShootingConstants.flywheelRadiusInches / 12.0;
        double omegaRadPerSec = velocityFps / wheelRadiusFeet;
        return (omegaRadPerSec * 60.0) / (2.0 * Math.PI);
    }
    
    /**
     * Calculate the heading the robot should face for passing (toward alliance wall).
     * 
     * @param isRedAlliance True if on red alliance
     * @return Heading in radians (toward alliance wall)
     */
    public static double calculatePassingHeading(boolean isRedAlliance) {
        if (isRedAlliance) {
            return Math.PI;
        } else {
            return 0.0;
        }
    }

    /**
     * Calculate the 3D launch velocity vector for fuel simulation.
     * 
     * @param robotPose Current robot pose (used for heading direction)
     * @param distanceMeters Distance to target (used to calculate speed and angle)
     * @return Translation3d velocity vector in m/s (field-relative)
     */
    public static Translation3d calculateLaunchVelocity(
            Pose2d robotPose, double distanceMeters) {
        
        // Get the launch angle (theta) - this is angle from horizontal
        // Note: calculateHoodAngle returns (PI/2 - theta), so we reverse it
        double hoodAngle = calculateHoodAngle(distanceMeters);
        double launchAngle = (Math.PI / 2) - hoodAngle;
        
        // Get initial velocity in m/s
        double velocityFPS = calculateInitialVelocityFPS(distanceMeters);
        double velocityMPS = velocityFPS * 0.3048; // feet to meters
        
        // Calculate horizontal and vertical components
        double horizontalSpeed = velocityMPS * Math.cos(launchAngle);
        double verticalSpeed = velocityMPS * Math.sin(launchAngle);
        
        // Get robot heading (direction the robot is facing)
        // Shooter is angled 11 degrees toward center (right side), so subtract the offset
        double shooterAngleOffset = -HeadingPID.shooterThetaOffset.get();
        double heading = robotPose.getRotation().getRadians() + shooterAngleOffset;
        
        // Split horizontal speed into X and Y based on robot heading
        double vx = horizontalSpeed * Math.cos(heading);
        double vy = horizontalSpeed * Math.sin(heading);
        double vz = verticalSpeed;
        
        return new Translation3d(vx, vy, vz);
    }
}
