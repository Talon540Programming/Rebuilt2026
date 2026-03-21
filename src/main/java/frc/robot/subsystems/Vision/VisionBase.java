package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import java.util.ArrayList;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import frc.robot.LimelightHelpers;

public class VisionBase extends SubsystemBase{

    private final VisionIO vision;
    private final VisionIOInputs limelightOne = new VisionIOInputs();
    private final VisionIOInputs limelightTwo = new VisionIOInputs();
    private final VisionIOInputs limelightThree = new VisionIOInputs();

    private final CommandSwerveDrivetrain drivetrain;
    private boolean hasInitializedGyro = false;
    private double setYawTagCount = 0;
    private double setYawTagDistance = 67;


    public VisionBase(VisionIO vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {

        setYawWithCameras(drivetrain);

        
        vision.updateLimelightYaw(drivetrain);
    
        
        //updating IO layer inputs from vision
        vision.updateVisionIOInputs(limelightOne, limelightTwo, limelightThree);

        // Process both cameras and apply given the standard deviations calcualted
        processVisionMeasurement(limelightOne);
        processVisionMeasurement(limelightTwo);
        processVisionMeasurement(limelightThree);

        Logger.recordOutput("Vision/EstimatedPoseOne", limelightOne.pose);
        Logger.recordOutput("Vision/EstimatedPoseTwo", limelightTwo.pose);
        Logger.recordOutput("Vision/EstimatedPoseThree", limelightThree.pose);
    }

    /*
     * calcualtes the bot pose in field space from each of the limelights
     * for each camera, a specific set of standard deviations are created and applied to that measurement
     * If the calculated position is too far away from the current pose and the camera is not confident the pose is rejected
     * The decrease in confidence scales exponentially with tag distance so that poor measurments will be weighted less
     * If the bot is spinning at a rate greater then 1/2 tau/sec then the pose is rejected 
     */
    private void processVisionMeasurement(VisionIOInputs input) {
        if (input.seenTagCount <= 0 || !input.hasTarget) {
            return;
        }

        // Reject if pose is outside the field
        if (input.pose.getX() < 0 || input.pose.getX() > 17.55 ||
            input.pose.getY() < 0 || input.pose.getY() > 8.05) {
            Logger.recordOutput("Vision/Rejected/" + input.cameraName, "Out of field bounds");
            return;
        }

        // Reject if pose jumped too far from current estimate (possible glitch)
        double poseDelta = drivetrain.getPose().getTranslation()
            .getDistance(input.pose.getTranslation());

        // Allow big corrections when vision is confident
        boolean visionIsConfident = input.seenTagCount > 1 && input.avgTagDistance < 3.0;

        if (poseDelta > 1.5 && !visionIsConfident) {
            Logger.recordOutput("Vision/Rejected/" + input.cameraName, "Large jump with low confidence");
            return;
        }

        // Reject during fast rotation (motion blur causes bad readings)
        if (Math.abs(drivetrain.getFieldVelocity().omegaRadiansPerSecond) > Math.toRadians(180)) {
            Logger.recordOutput("Vision/Rejected/" + input.cameraName, "Rotating too fast");
            return;
        }

        Matrix<N3, N1> stdDevs = calculateStdDevs(input);
        drivetrain.addVisionMeasurement(input.pose, input.limelightTimestamp, stdDevs);
        Logger.recordOutput("Vision/Accepted/" + input.cameraName, input.pose);
    }

    private Matrix<N3, N1> calculateStdDevs(VisionIOInputs input) {
        double xyStdDev = 0.5;  // Base trust
        double rotStdDev = 0.5;
        
        // More tags = more confidence
        if (input.seenTagCount == 1) {
            xyStdDev = 1.0;
            rotStdDev = 2.0;  // Single tag rotation is less reliable
        }
        
        // Scale trust by distance (exponential falloff)
        double distanceMultiplier = 1.0 + (input.avgTagDistance * input.avgTagDistance * VisionConstants.exponentialScalarMultiplier); // increase if bad values 
        xyStdDev *= distanceMultiplier;
        rotStdDev *= distanceMultiplier;

        double poseDelta = drivetrain.getPose().getTranslation()
            .getDistance(input.pose.getTranslation());

        if(poseDelta > 1){
            xyStdDev += poseDelta;
        }
        
        // Hard limits - don't trust too much or reject entirely
        xyStdDev = Math.max(0.3, Math.min(xyStdDev, 5.0));
        rotStdDev = Math.max(0.5, Math.min(rotStdDev, 10.0));
        
        return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }
    /*
     * Prematch sets yaw with MT1 with higher tollerance then turns on metatag 2
     * once gyro has been init then only check for gyro with cameras if very confident
     */
    public void setYawWithCameras(CommandSwerveDrivetrain drivetrain) {

        LimelightHelpers.PoseEstimate mt1EstimateCameraOne = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightOne);

        LimelightHelpers.PoseEstimate mt1EstimateCameraTwo = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightTwo);

        LimelightHelpers.PoseEstimate mt1EstimateCameraThree = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightThree);

        ArrayList<LimelightHelpers.PoseEstimate> poseList = new ArrayList<>();

        poseList.add(mt1EstimateCameraOne);
        poseList.add(mt1EstimateCameraTwo);
        poseList.add(mt1EstimateCameraThree);
        
        for(LimelightHelpers.PoseEstimate mt1Estimate : poseList){
            if (mt1Estimate == null || mt1Estimate.tagCount == 0) {
                continue;
            }
            if(DriverStation.isDisabled() && setYawTagCount >= mt1Estimate.tagCount && mt1Estimate.avgTagDist <= setYawTagDistance){
                Rotation2d visionYaw = mt1Estimate.pose.getRotation();
                drivetrain.resetRotation(visionYaw);
                hasInitializedGyro = true;
                setYawTagCount = mt1Estimate.tagCount;
                setYawTagDistance = mt1Estimate.avgTagDist;
                Logger.recordOutput("Vision/GyroInitializedFromVision", true);
                Logger.recordOutput("Vision/InitialYaw", visionYaw.getDegrees());
                Logger.recordOutput("Vision/InitTagCount", mt1Estimate.tagCount);
                Logger.recordOutput("Vision/InitAvgDist", mt1Estimate.avgTagDist);
                return;
            }
            else{
                if(mt1Estimate.tagCount >= VisionConstants.teleopYawTagCount || mt1Estimate.avgTagDist <= VisionConstants.teleopYawTagDistance){
                    Rotation2d visionYaw = mt1Estimate.pose.getRotation();
                    drivetrain.resetRotation(visionYaw);
                }
            }
        }
    }

    public void setGyroInitialized() {
        hasInitializedGyro = true;
        Logger.recordOutput("Vision/GyroManualOverride", true);
    }
    
    public VisionIOInputs getVisionIOInputsOne() {
        return limelightOne;
    }
    
    public VisionIOInputs getVisionIOInputsTwo() {
        return limelightTwo;
    }
    
    public boolean hasTarget() {
        return limelightOne.hasTarget || limelightTwo.hasTarget;
    }
    
    public double getTX() {
        // Return TX from whichever camera has a target, prioritizing camera one
        if (limelightOne.hasTarget) {
            return limelightOne.limelightTX;
        }
        return limelightTwo.limelightTX;
    }

    public boolean isRedAlliance() {
        // Both should have the same value, just pick one
        return limelightOne.isRedAlliance;
    }    

    /**
     * Final attempt to set yaw from cameras before teleop, regardless of trustworthiness.
     * Only used if gyro hasn't been initialized yet.
     * @return true if a pose was found and applied, false if no cameras saw tags
     */
    public boolean forceSetYawFromCameras(CommandSwerveDrivetrain drivetrain) {
        if (hasInitializedGyro) {
            return true; // Already initialized, no need to force
        }

        LimelightHelpers.PoseEstimate mt1EstimateCameraOne = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightOne);

        LimelightHelpers.PoseEstimate mt1EstimateCameraTwo = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightTwo);

        // Try camera one first
        if (mt1EstimateCameraOne != null && mt1EstimateCameraOne.tagCount > 0) {
            Rotation2d visionYaw = mt1EstimateCameraOne.pose.getRotation();
            drivetrain.resetRotation(visionYaw);
            hasInitializedGyro = true;
            Logger.recordOutput("Vision/GyroForcedFromVision", true);
            Logger.recordOutput("Vision/ForcedYaw", visionYaw.getDegrees());
            Logger.recordOutput("Vision/ForcedFromCamera", "One");
            return true;
        }

        // Fall back to camera two
        if (mt1EstimateCameraTwo != null && mt1EstimateCameraTwo.tagCount > 0) {
            Rotation2d visionYaw = mt1EstimateCameraTwo.pose.getRotation();
            drivetrain.resetRotation(visionYaw);
            hasInitializedGyro = true;
            Logger.recordOutput("Vision/GyroForcedFromVision", true);
            Logger.recordOutput("Vision/ForcedYaw", visionYaw.getDegrees());
            Logger.recordOutput("Vision/ForcedFromCamera", "Two");
            return true;
        }

        // No cameras saw any tags
        Logger.recordOutput("Vision/GyroForceAttemptFailed", true);
        return false;
    }

    public boolean isGyroInitialized() {
        return hasInitializedGyro;
    }

    public void finalGyroCheck(){
        if(!isGyroInitialized()){
            boolean success = forceSetYawFromCameras(drivetrain);
            if(!success){
                drivetrain.seedFieldCentric();
                setGyroInitialized();
            }
        }
    }

}
