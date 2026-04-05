package frc.robot.subsystems.Vision;

public class VisionConstants {

    public static final String limelightOne = "limelight-one";
    public static final String limelightTwo = "limelight-two";
    public static final String limelightThree = "limelight-three";

    public static final int teleopYawTagCount = 2; // increase if getting bad rotation values
    public static final double teleopYawTagDistance = 2;
    public static final double exponentialScalarMultiplier = 0.05;
    public static final double poseDeltaTrust = 3;
    public static final double confidenceTagDistance = 4.5;
    public static final double confidenceTagAmount = 1;
}
