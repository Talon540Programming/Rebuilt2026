package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utility.ShootingCalculator;
import frc.robot.Constants.FieldPoses;
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
    private Rotation2d targetHeading = new Rotation2d();
    private Translation2d virtualGoal = null;
    private double distanceToHub = 0.0;

    public SetHeading(VisionBase vision) {
        this.vision = vision;
    }

    public void enableFaceHub() {
        faceHubEnabled = true;
        passingEnabled = false;
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }
    
    public void toggleFaceHub() {
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
        // If your shooter is on the back of the robot, add Rotation2d.k180deg
        targetHeading = new Rotation2d(Math.atan2(dy, dx));
        
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
        double aimingHeading = ShootingCalculator.calculateAimingHeading(currentPose, virtualGoal);
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
        passingEnabled = true;
        faceHubEnabled = false;
        Logger.recordOutput("Passing/Enabled", passingEnabled);
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }
    
    public void togglePassing() {
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
        
        double passingHeadingRad = ShootingCalculator.calculatePassingHeading(isRed);
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
}