package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    // All below are constants
    private final TalonFX topFlywheel;
    private final TalonFX bottomFlywheel;

    private final VelocityVoltage topVelocityRequest;
    private final VelocityVoltage bottomVelocityRequest;

    // Needs to be changed
    private static final int topMotorID = 10;
    private static final int bottomMotorID = 11;

    // Needs to be changed 
    private static final double topFlyWheelDiameterInches = 0.0;
    private static final double bottomFlywheelDiameterInches = 0.0;
    private static final double topFlywheelRadiusInches = 0.0;
    private static final double bottomFlywheelRadiusInches= 0.0;

    
    private final double maxRPM = 6000;

    // PID gains (Need to be tuned)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;
    private static final double kV = 0.0;


    // Target Velocities (are going to be changing)
    private double targetTopRPM = 0.0;
    private double targetBottomRPM = 0.0;

    public ShooterSubsystem() {

        // Intializing Motors
        topFlywheel = new TalonFX(topMotorID);
        bottomFlywheel = new TalonFX(bottomMotorID);

        // Velocity Control Requests
        topVelocityRequest = new VelocityVoltage(0).withSlot(0);
        bottomVelocityRequest = new VelocityVoltage(0).withSlot(0);

        // Configuration
        configureMotor(topFlywheel);
        

    }
}
