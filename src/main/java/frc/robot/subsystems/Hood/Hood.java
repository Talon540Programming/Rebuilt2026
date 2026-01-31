package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hood.HoodIO.HoodIOInputs;

/**
 * Hood subsystem for controlling the shooter angle.
 * 
 * The hood controls the angle of the shooter using a rack and pinion mechanism
 * driven by one X44 Kraken motor with MotionMagic position control. It smoothly adjusts
 * between minimum angle (pi/8) and maximum angle (pi/2).
 * 
 * The encoder should be zeroed at the beginning of each match.
 */
public class Hood extends SubsystemBase {
    
    private final HoodIO io;
    private final HoodIOInputs inputs = new HoodIOInputs();
    
    /**
     * Creates a new Hood subsystem.
     * 
     * @param io The hardware interface implementation (real or simulated)
     */
    public Hood(HoodIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        
        // Additional logging for convenience
        Logger.recordOutput("Hood/AtTargetPosition", isAtTargetPosition());
        Logger.recordOutput("Hood/PositionError", getPositionError());
        Logger.recordOutput("Hood/AngleDegrees", getAngleDegrees());
    }
    
    /**
     * Sets the hood to minimum angle (pi/8 radians = 22.5 degrees).
     * Uses MotionMagic for smooth movement.
     */
    public void setMinAngle() {
        io.setPosition(HoodConstants.MIN_ANGLE_ROTATIONS);
    }
    
    /**
     * Sets the hood to maximum angle (pi/2 radians = 90 degrees).
     * Uses MotionMagic for smooth movement.
     */
    public void setMaxAngle() {
        io.setPosition(HoodConstants.MAX_ANGLE_ROTATIONS);
    }
    
    /**
     * Sets the hood to a specific position using MotionMagic control.
     * Position is automatically clamped to min/max limits.
     * 
     * @param positionRotations Target position in rotations
     */
    public void setPosition(double positionRotations) {
        io.setPosition(positionRotations);
    }
    
    /**
     * Sets the hood to a specific angle in radians.
     * Converts radians to rotations and uses MotionMagic control.
     * 
     * @param angleRadians Target angle in radians (pi/8 to pi/2)
     */
    public void setAngleRadians(double angleRadians) {
        double rotations = angleRadians / (2.0 * Math.PI);
        io.setPosition(rotations);
    }
    
    /**
     * Zeros the hood encoder at the current position.
     * This should be called at the beginning of each match when hood is at a known position.
     */
    public void zeroEncoder() {
        io.zeroEncoder();
    }
    
    /**
     * Stops the hood motor.
     * Holds the current position when using brake mode.
     */
    public void stop() {
        io.stop();
    }
    
    /**
     * Sets the voltage output for the hood (open-loop control).
     * Primarily used for characterization and testing.
     * 
     * @param volts Voltage to apply (-12 to +12V)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    /**
     * Checks if the hood is at its target position within tolerance.
     * 
     * @return true if hood is within tolerance of target
     */
    public boolean isAtTargetPosition() {
        double error = Math.abs(inputs.positionRotations - inputs.targetPositionRotations);
        return error < HoodConstants.POSITION_TOLERANCE;
    }
    
    /**
     * Gets the position error for the hood.
     * 
     * @return Position error in rotations
     */
    public double getPositionError() {
        return inputs.targetPositionRotations - inputs.positionRotations;
    }
    
    /**
     * Gets the current position of the hood.
     * 
     * @return Current position in rotations
     */
    public double getPosition() {
        return inputs.positionRotations;
    }
    
    /**
     * Gets the current angle of the hood in radians.
     * 
     * @return Current angle in radians
     */
    public double getAngleRadians() {
        return inputs.positionRotations * 2.0 * Math.PI;
    }
    
    /**
     * Gets the current angle of the hood in degrees.
     * 
     * @return Current angle in degrees
     */
    public double getAngleDegrees() {
        return Math.toDegrees(getAngleRadians());
    }
    
    /**
     * Gets the target position of the hood.
     * 
     * @return Target position in rotations
     */
    public double getTargetPosition() {
        return inputs.targetPositionRotations;
    }
    
    /**
     * Checks if the hood is at minimum angle.
     * 
     * @return true if hood is at minimum angle
     */
    public boolean isAtMinAngle() {
        return Math.abs(inputs.positionRotations - HoodConstants.MIN_ANGLE_ROTATIONS) 
               < HoodConstants.POSITION_TOLERANCE;
    }
    
    /**
     * Checks if the hood is at maximum angle.
     * 
     * @return true if hood is at maximum angle
     */
    public boolean isAtMaxAngle() {
        return Math.abs(inputs.positionRotations - HoodConstants.MAX_ANGLE_ROTATIONS) 
               < HoodConstants.POSITION_TOLERANCE;
    }
}