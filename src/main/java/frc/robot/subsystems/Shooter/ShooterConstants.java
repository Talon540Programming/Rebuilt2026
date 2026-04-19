package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.utility.LoggedTunableNumber;

public class ShooterConstants {
    // ===================== CAN IDs =====================
    public static final int flywheelMotor1Id = 17;
    public static final int flywheelMotor2Id = 18;
    public static final int hoodMotorId = 19;

    // ===================== Flywheel Constants =====================

    // Flywheel velocity scalar - exponential function of distance
    // Testing tunable - set to non-zero to override exponential function
    public static final LoggedTunableNumber flywheelVelocityScalarTest = 
        new LoggedTunableNumber("Shooter/Flywheel/VelocityScalarTest");

    public static final LoggedTunableNumber flywheelRPMThreshould = 
        new LoggedTunableNumber("Shooter/FlywheelRPMThreshould");
    
    // Flywheel tolerances
    public static final LoggedTunableNumber flywheelVelToleranceRPM = 
        new LoggedTunableNumber("Shooter/Flywheel/VelToleranceRPM");
    
    // Motion Magic Velocity PID gains
    public static final LoggedTunableNumber flywheelkS = 
        new LoggedTunableNumber("Shooter/Flywheel/kS");
    public static final LoggedTunableNumber flywheelkV = 
        new LoggedTunableNumber("Shooter/Flywheel/kV");
    public static final LoggedTunableNumber flywheelkP = 
        new LoggedTunableNumber("Shooter/Flywheel/kP");
    
    // Motion Magic Velocity acceleration (rot/s^2)
    public static final LoggedTunableNumber flywheelMMAccelRotPerSec2 = 
        new LoggedTunableNumber("Shooter/Flywheel/MMAccelRotPerSec2");
    
    // Current limits
    public static final LoggedTunableNumber flywheelStatorCurrentLimit = 
        new LoggedTunableNumber("Shooter/Flywheel/StatorCurrentLimit");
    public static final LoggedTunableNumber flywheelSupplyCurrentLimit = 
        new LoggedTunableNumber("Shooter/Flywheel/SupplyCurrentLimit");

    // ===================== Hood Homing Constants =====================
    public static final LoggedTunableNumber hoodHomingDutyCycle = 
        new LoggedTunableNumber("Shooter/Hood/HomingDutyCycle");
    public static final LoggedTunableNumber hoodHomingTimeoutSeconds = 
        new LoggedTunableNumber("Shooter/Hood/HomingTimeoutSeconds");

    // ===================== Hood Constants =====================
    public static final LoggedTunableNumber hoodkP = 
        new LoggedTunableNumber("Shooter/Hood/kP");
    public static final LoggedTunableNumber hoodkI = 
        new LoggedTunableNumber("Shooter/Hood/kI");
    public static final LoggedTunableNumber hoodkD = 
        new LoggedTunableNumber("Shooter/Hood/kD");
    public static final LoggedTunableNumber hoodkS = 
        new LoggedTunableNumber("Shooter/Hood/kS");
    public static final LoggedTunableNumber hoodkV = 
        new LoggedTunableNumber("Shooter/Hood/kV");
    public static final LoggedTunableNumber hoodkA = 
        new LoggedTunableNumber("Shooter/Hood/kA");
    public static final LoggedTunableNumber hoodkG = 
        new LoggedTunableNumber("Shooter/Hood/kG");  
    public static final LoggedTunableNumber hoodOutOfBoundsDebounceSeconds = 
        new LoggedTunableNumber("Shooter/Hood/OutOfBoundsDebounceSeconds");

    // Motion Magic Position constraints
    public static final LoggedTunableNumber hoodMMCruiseVelRotPerSec = 
        new LoggedTunableNumber("Shooter/Hood/MMCruiseVelRotPerSec");
    public static final LoggedTunableNumber hoodMMAccelRotPerSec2 = 
        new LoggedTunableNumber("Shooter/Hood/MMAccelRotPerSec2");
    public static final LoggedTunableNumber hoodMMJerkRotPerSec3 = 
        new LoggedTunableNumber("Shooter/Hood/MMJerkRotPerSec3");
    
    public static final LoggedTunableNumber hoodSensorToMechanismRatio = 
        new LoggedTunableNumber("Shooter/Hood/SensorToMechanismRatio");

    // Hood position limits (in motor rotations after gear ratio)
    public static final LoggedTunableNumber hoodMinPositionRot = 
        new LoggedTunableNumber("Shooter/Hood/MinPositionRot");
    public static final LoggedTunableNumber hoodMaxPositionRot = 
        new LoggedTunableNumber("Shooter/Hood/MaxPositionRot");
    
    // Hood tolerances
    public static final LoggedTunableNumber hoodPosToleranceRot = 
        new LoggedTunableNumber("Shooter/Hood/PosToleranceRot");
    public static final LoggedTunableNumber hoodVelToleranceRotPerSec = 
        new LoggedTunableNumber("Shooter/Hood/VelToleranceRotPerSec");
    
    // Current limits
    public static final LoggedTunableNumber hoodStatorCurrentLimit = 
        new LoggedTunableNumber("Shooter/Hood/StatorCurrentLimit");
    public static final LoggedTunableNumber hoodSupplyCurrentLimit = 
        new LoggedTunableNumber("Shooter/Hood/SupplyCurrentLimit");

    // ===================== Kickup Constants =====================
    public static final class KickupConstants {
        public static final int kKickupMotorId = 16;
        
        public static final LoggedTunableNumber feedDutyCycle = 
            new LoggedTunableNumber("Shooter/Kickup/FeedDutyCycle");
        public static final LoggedTunableNumber reverseDutyCycle = 
            new LoggedTunableNumber("Shooter/Kickup/ReverseDutyCycle");
        public static final LoggedTunableNumber statorCurrentLimit = 
            new LoggedTunableNumber("Shooter/Kickup/StatorCurrentLimit");
        public static final LoggedTunableNumber supplyCurrentLimit = 
            new LoggedTunableNumber("Shooter/Kickup/SupplyCurrentLimit");
        
        static {
            switch (Constants.getRobot()) {
                case SIMBOT -> {
                    feedDutyCycle.initDefault(-0.8);
                    reverseDutyCycle.initDefault(-0.3);
                    statorCurrentLimit.initDefault(60);
                    supplyCurrentLimit.initDefault(40);
                    flywheelRPMThreshould.initDefault(250);
                }
                case COMPBOT -> {
                    feedDutyCycle.initDefault(-1);
                    reverseDutyCycle.initDefault(0.8);
                    statorCurrentLimit.initDefault(60);
                    supplyCurrentLimit.initDefault(40);
                    flywheelRPMThreshould.initDefault(250);
                }
            }
        }
    }

    // Static initializer - sets defaults based on robot type
    static {
        switch (Constants.getRobot()) {
            case SIMBOT -> {
                // ===== Flywheel Sim Defaults =====
                flywheelVelToleranceRPM.initDefault(10);
                flywheelStatorCurrentLimit.initDefault(80);
                flywheelSupplyCurrentLimit.initDefault(60);
                flywheelVelocityScalarTest.initDefault(0.0);  // 0 = use exponential function
                
                // Motion Magic Velocity PID gains (estimated values for Kraken X60)
                // kS: Static friction voltage (~0.1-0.3V typical)
                flywheelkS.initDefault(0.15);
                // kV: Velocity feedforward (V per rot/s) - calculate from motor specs
                // Kraken X60 Kv ~ 5330 RPM/V = 88.8 rot/s/V, so kV ~ 1/88.8 = 0.0113
                flywheelkV.initDefault(0.011);
                // kP: Proportional gain - start conservative, tune up
                flywheelkP.initDefault(0.3);
                // Acceleration: Very high for maximum acceleration (as fast as possible)
                flywheelMMAccelRotPerSec2.initDefault(1000);  // rot/s^2
                
                // ===== Hood Sim Defaults =====
                hoodkP.initDefault(4.8);
                hoodkI.initDefault(0.0);
                hoodkD.initDefault(0.1);
                hoodkS.initDefault(0.25);
                hoodkV.initDefault(0.12);
                hoodkA.initDefault(0.01);
                hoodkG.initDefault(0.0);
                
                hoodMMCruiseVelRotPerSec.initDefault(80);
                hoodMMAccelRotPerSec2.initDefault(160);
                hoodMMJerkRotPerSec3.initDefault(1600);
            
                hoodSensorToMechanismRatio.initDefault(7.1500);

                hoodMinPositionRot.initDefault(0.0);  
                hoodMaxPositionRot.initDefault(36.9896098);
                
                hoodPosToleranceRot.initDefault(0.5);
                hoodVelToleranceRotPerSec.initDefault(0.1);
                hoodStatorCurrentLimit.initDefault(40);
                hoodSupplyCurrentLimit.initDefault(30);
                
                // Hood homing
                hoodHomingDutyCycle.initDefault(0.4);
                hoodHomingTimeoutSeconds.initDefault(0.75);
                hoodOutOfBoundsDebounceSeconds.initDefault(0.25);
            }
            case COMPBOT -> {
                // ===== Flywheel Comp Defaults =====
                flywheelVelToleranceRPM.initDefault(20);
                flywheelStatorCurrentLimit.initDefault(80);
                flywheelSupplyCurrentLimit.initDefault(60);
                flywheelVelocityScalarTest.initDefault(0.0);  // 0 = use exponential function
                
                // Motion Magic Velocity PID gains (estimated values - TUNE ON ROBOT)
                // kS: Static friction voltage
                flywheelkS.initDefault(0.15);  // TODO: tune with SysId or manual testing
                // kV: Velocity feedforward (V per rot/s)
                flywheelkV.initDefault(0.011);  // TODO: tune - calculate from free speed test
                // kP: Proportional gain
                flywheelkP.initDefault(0.3);  // TODO: tune - start low, increase until responsive
                // Acceleration: Very high for maximum acceleration
                flywheelMMAccelRotPerSec2.initDefault(1000);  // TODO: tune - can go higher if no brownouts
                
                // ===== Hood Comp Defaults =====
                hoodkP.initDefault(5);
                hoodkI.initDefault(0.0);
                hoodkD.initDefault(1);
                hoodkS.initDefault(0.0);
                hoodkV.initDefault(0.0);
                hoodkA.initDefault(0.0);
                hoodkG.initDefault(1);
                
                hoodMMCruiseVelRotPerSec.initDefault(25);
                hoodMMAccelRotPerSec2.initDefault(50);
                hoodMMJerkRotPerSec3.initDefault(0);

                hoodMinPositionRot.initDefault(0.0); 
                hoodMaxPositionRot.initDefault(1.5);
                
                hoodSensorToMechanismRatio.initDefault(7.1500);
                
                hoodPosToleranceRot.initDefault(0.5);
                hoodVelToleranceRotPerSec.initDefault(0.1);
                hoodStatorCurrentLimit.initDefault(40);
                hoodSupplyCurrentLimit.initDefault(30);
                
                // Hood homing
                hoodHomingDutyCycle.initDefault(-0.3);
                hoodHomingTimeoutSeconds.initDefault(0.5);
                hoodOutOfBoundsDebounceSeconds.initDefault(0.25);
            }
        }
    }
}
