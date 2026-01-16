package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.util.ShootingCalculator;

/**
 * Manages automatic heading control to face the hub while driving.
 * Similar to SetReefCenterHeading but targets the hub for shooting.
 */
public class SetHubHeading {

    private final VisionBase vision;
    
    private boolean faceHubEnabled = false;
    private Rotation2d targetHeading = new Rotation2d();
    private double distanceToHub = 0.0;

    public SetHubHeading(VisionBase vision) {
        this.vision = vision;
    }

    public void enableFaceHub() {
        faceHubEnabled = true;
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }
    
    public void toggleFaceHub() {
        faceHubEnabled = !faceHubEnabled;
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
}