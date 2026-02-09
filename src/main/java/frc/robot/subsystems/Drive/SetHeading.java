package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utility.ShootingCalculator;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.HeadingPID;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.VisionBase;

/**
 * Manages automatic heading control to face the hub while driving.
 * Similar to SetReefCenterHeading but targets the hub for shooting.
 */
public class SetHeading {

    private final VisionBase vision;
    
    private boolean faceHubEnabled = false;
    private boolean passingEnabled = false;
    private boolean autoModeActive = false;
    private boolean emergencyModeEnabled = false;
    private boolean emergencyShootingMode = false;
    private boolean emergencyPassingMode = false;
    private boolean emergencyHoodRetractMode = false;
    private boolean previousWasShootingMode = true; // Default to shooting mode
    private Rotation2d targetHeading = new Rotation2d();
    private Translation2d virtualGoal = null;
    private double distanceToHub = 0.0;

    public SetHeading(VisionBase vision) {
        this.vision = vision;
    }

   public void enableFaceHub() {
        if (emergencyModeEnabled) return; // Locked out in emergency mode
        faceHubEnabled = true;
        passingEnabled = false;
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }
    
    public void toggleFaceHub() {
        if (emergencyModeEnabled) return; // Locked out in emergency mode
        faceHubEnabled = !faceHubEnabled;
        if (faceHubEnabled) {
            passingEnabled = false;
            Logger.recordOutput("Passing/Enabled", passingEnabled);
        }
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }

    public void disableFaceHub() {
        faceHubEnabled = false;
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }

    public boolean isEnabled() {
        return faceHubEnabled;
    }

    // ==================== AUTOMATIC MODE SELECTION ====================

    /**
     * Enable automatic heading mode based on field position.
     * Automatically switches between hub facing and passing based on robot X position.
     * 
     * @param currentPose The current pose of the robot
     */
    public void enableAutoMode(Pose2d currentPose) {
        if (emergencyModeEnabled) return;
        
        autoModeActive = true;
        updateAutoMode(currentPose);
        Logger.recordOutput("AutoMode/Active", autoModeActive);
    }

    /**
     * Update the automatic mode selection based on current field position.
     * Call this periodically while auto mode is active.
     * 
     * @param currentPose The current pose of the robot
     */
    public void updateAutoMode(Pose2d currentPose) {
        if (!autoModeActive || emergencyModeEnabled) return;

        boolean isRed = vision.isRedAlliance();
        double robotX = currentPose.getX();
        
        // Determine if we're on our scoring side (shooting) or past the hub (passing)
        boolean shouldShoot;
        if (isRed) {
            // Red alliance: our side is X > 12.0 (hub X)
            shouldShoot = robotX > FieldPoses.redHub.getX();
        } else {
            // Blue alliance: our side is X < 4.5 (hub X)
            shouldShoot = robotX < FieldPoses.blueHub.getX();
        }

        if (shouldShoot) {
            faceHubEnabled = true;
            passingEnabled = false;
        } else {
            faceHubEnabled = false;
            passingEnabled = true;
        }

        Logger.recordOutput("AutoMode/ShouldShoot", shouldShoot);
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }

    /**
     * Disable automatic heading mode.
     */
    public void disableAutoMode() {
        autoModeActive = false;
        faceHubEnabled = false;
        passingEnabled = false;
        Logger.recordOutput("AutoMode/Active", autoModeActive);
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }

    /**
     * Check if automatic mode is active.
     */
    public boolean isAutoModeActive() {
        return autoModeActive;
    }

    /**
     * Check if heading is within tolerance of the target.
     * 
     * @param currentHeadingRadians Current robot heading in radians
     * @return true if heading is within tolerance
     */
    public boolean isHeadingAtTarget(double currentHeadingRadians) {
        double error = Math.abs(targetHeading.getRadians() - currentHeadingRadians);
        // Handle wraparound
        if (error > Math.PI) {
            error = 2 * Math.PI - error;
        }
        boolean atTarget = error < HeadingPID.headingToleranceRadians.get();
        Logger.recordOutput("AutoMode/HeadingError", Math.toDegrees(error));
        Logger.recordOutput("AutoMode/HeadingAtTarget", atTarget);
        return atTarget;
    } 

    /**
     * Updates the target heading to face the center of the hub.
     * Also calculates and stores distance to hub for shooting calculations.
     * 
     * @param currentPose The current pose of the robot
     */
    public void updateTargetHeading(Pose2d currentPose) {
        boolean isRed = vision.isRedAlliance();
        Pose2d hubCenter = isRed ? FieldPoses.redHub : FieldPoses.blueHub;
    
        double dx = hubCenter.getX() - currentPose.getX();
        double dy = hubCenter.getY() - currentPose.getY();
        
        // Face TOWARD the hub (shooter faces hub)
        // Include the shooter theta offset to match what SmoothFieldCentricFacingAngle uses
        double aimingHeading = Math.atan2(dy, dx) + HeadingPID.shooterThetaOffset.get();
        targetHeading = new Rotation2d(aimingHeading);
        
        // Calculate distance to hub
        distanceToHub = ShootingCalculator.getDistanceToHub(currentPose, isRed);
    
        // Log values
        Logger.recordOutput("FaceHub/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("FaceHub/HubCenter", hubCenter);
        Logger.recordOutput("FaceHub/DistanceToHub", distanceToHub);
    }
    /**
     * Get the target heading to face the hub.
     */
    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    /**
     * Get the current distance to the hub in meters.
     * Useful for shooting calculations.
     */
    public double getDistanceToHub() {
        return distanceToHub;
    }

    /**
     * Check if driver is manually rotating (to override auto-aim).
     */
    public boolean isDriverRotating(double rotationInput) {
        return Math.abs(rotationInput) > OperatorConstants.deadband;
    }

    /**
     * Updates the virtual goal position for shoot-while-moving.
     * Call this periodically when shoot-while-moving is active.
     * 
     * @param currentPose The current pose of the robot
     * @param fieldVelocity Field-relative velocity of the robot
     */
    public void updateVirtualGoal(Pose2d currentPose, ChassisSpeeds fieldVelocity) {
        boolean isRed = vision.isRedAlliance();
        
        virtualGoal = ShootingCalculator.calculateVirtualGoal(currentPose, fieldVelocity, isRed);
        
        // Calculate heading that points the shooter at the virtual goal
        // Include the shooter theta offset to match what SmoothFieldCentricFacingAngle uses
        double aimingHeading = ShootingCalculator.calculateAimingHeading(currentPose, virtualGoal) 
            + HeadingPID.shooterThetaOffset.get();
        targetHeading = new Rotation2d(aimingHeading);
        
        // Distance is now to virtual goal
        distanceToHub = virtualGoal.getDistance(currentPose.getTranslation());
        
        // Log values
        Logger.recordOutput("FaceHub/VirtualGoal", virtualGoal);
        Logger.recordOutput("FaceHub/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("FaceHub/DistanceToVirtualGoal", distanceToHub);
    }

     // ==================== PASSING MODE ====================
    
    public void enablePassing() {
        if (emergencyModeEnabled) return; // Locked out in emergency mode
        passingEnabled = true;
        faceHubEnabled = false;
        Logger.recordOutput("Passing/Enabled", passingEnabled);
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }
    
    public void togglePassing() {
        if (emergencyModeEnabled) return; // Locked out in emergency mode
        passingEnabled = !passingEnabled;
        if (passingEnabled) {
            faceHubEnabled = false;
            Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        }
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }
    
    public void disablePassing() {
        passingEnabled = false;
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }
    
    public boolean isPassingEnabled() {
        return passingEnabled;
    }
    
    /**
     * Updates the target heading for passing mode (face alliance wall).
     * 
     * @param currentPose The current pose of the robot (unused but kept for consistency)
     */
    public void updatePassingHeading(Pose2d currentPose) {
        boolean isRed = vision.isRedAlliance();
        
        // Include the shooter theta offset to match what SmoothFieldCentricFacingAngle uses
        double passingHeadingRad = ShootingCalculator.calculatePassingHeading(isRed) 
            + HeadingPID.shooterThetaOffset.get();
        targetHeading = new Rotation2d(passingHeadingRad);
        
        Logger.recordOutput("Passing/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("Passing/IsRedAlliance", isRed);
    }

    /**
     * Get the current virtual goal position.
     */
    public Translation2d getVirtualGoal() {
        return virtualGoal;
    }

    // ==================== EMERGENCY MODE ====================
    
    public void toggleEmergencyMode() {
        emergencyModeEnabled = !emergencyModeEnabled;
        if (emergencyModeEnabled) {
            // Disable all auto-heading when entering emergency mode
            faceHubEnabled = false;
            passingEnabled = false;
            emergencyShootingMode = false;
            emergencyPassingMode = false;
            emergencyHoodRetractMode = false;
            previousWasShootingMode = true; // Reset to default (shooting)
        }
        Logger.recordOutput("EmergencyMode/Enabled", emergencyModeEnabled);
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        Logger.recordOutput("Passing/Enabled", passingEnabled);
        Logger.recordOutput("EmergencyMode/ShootingMode", emergencyShootingMode);
        Logger.recordOutput("EmergencyMode/PassingMode", emergencyPassingMode);
        Logger.recordOutput("EmergencyMode/HoodRetractMode", emergencyHoodRetractMode);
    }
    
    public boolean isEmergencyModeEnabled() {
        return emergencyModeEnabled;
    }
    
    public void toggleEmergencyShooting() {
        if (!emergencyModeEnabled) return;
        emergencyShootingMode = !emergencyShootingMode;
        if (emergencyShootingMode) {
            emergencyPassingMode = false;
            emergencyHoodRetractMode = false;
            previousWasShootingMode = true;
        }
        Logger.recordOutput("EmergencyMode/ShootingMode", emergencyShootingMode);
        Logger.recordOutput("EmergencyMode/PassingMode", emergencyPassingMode);
        Logger.recordOutput("EmergencyMode/HoodRetractMode", emergencyHoodRetractMode);
    }
    
    public void toggleEmergencyPassing() {
        if (!emergencyModeEnabled) return;
        emergencyPassingMode = !emergencyPassingMode;
        if (emergencyPassingMode) {
            emergencyShootingMode = false;
            emergencyHoodRetractMode = false;
            previousWasShootingMode = false;
        }
        Logger.recordOutput("EmergencyMode/PassingMode", emergencyPassingMode);
        Logger.recordOutput("EmergencyMode/ShootingMode", emergencyShootingMode);
        Logger.recordOutput("EmergencyMode/HoodRetractMode", emergencyHoodRetractMode);
    }
    
    public boolean isEmergencyShootingMode() {
        return emergencyModeEnabled && emergencyShootingMode;
    }
    
    public boolean isEmergencyPassingMode() {
        return emergencyModeEnabled && emergencyPassingMode;
    }

    public void toggleEmergencyHoodRetract() {
        if (!emergencyModeEnabled) return;
        emergencyHoodRetractMode = !emergencyHoodRetractMode;
        if (emergencyHoodRetractMode) {
            emergencyShootingMode = false;
            emergencyPassingMode = false;
        }
        Logger.recordOutput("EmergencyMode/HoodRetractMode", emergencyHoodRetractMode);
        Logger.recordOutput("EmergencyMode/ShootingMode", emergencyShootingMode);
        Logger.recordOutput("EmergencyMode/PassingMode", emergencyPassingMode);
    }
    
    public boolean isEmergencyHoodRetractMode() {
        return emergencyModeEnabled && emergencyHoodRetractMode;
    }

    /**
     * Revert from hood retract mode to the previous shooting/passing mode.
     * Called after shooting completes.
     */
    public void revertFromHoodRetract() {
        if (!emergencyModeEnabled || !emergencyHoodRetractMode) return;
        
        emergencyHoodRetractMode = false;
        if (previousWasShootingMode) {
            emergencyShootingMode = true;
            emergencyPassingMode = false;
        } else {
            emergencyPassingMode = true;
            emergencyShootingMode = false;
        }
        
        Logger.recordOutput("EmergencyMode/HoodRetractMode", emergencyHoodRetractMode);
        Logger.recordOutput("EmergencyMode/ShootingMode", emergencyShootingMode);
        Logger.recordOutput("EmergencyMode/PassingMode", emergencyPassingMode);
    }
}