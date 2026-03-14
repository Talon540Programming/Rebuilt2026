package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.utility.LoggedTunableNumber;

public class ShooterConstants {
    // ===================== CAN IDs =====================
    public static final int flywheelMotor1Id = 17;
    public static final int flywheelMotor2Id = 18;
    public static final int hoodMotorId = 19;

    // ===================== Flywheel Constants =====================

   // ===================== Flywheel Constants =====================

    // Distance-based velocity scalars (for hub shooting only)
    // Short range: 0.0 - 2.4 meters
    public static final LoggedTunableNumber flywheelVelocityScalarShort = 
        new LoggedTunableNumber("Shooter/Flywheel/VelocityScalarShort");
    // Middle range: 2.4 - 3.8 meters
    public static final LoggedTunableNumber flywheelVelocityScalarMid = 
        new LoggedTunableNumber("Shooter/Flywheel/VelocityScalarMid");
    // Long range: > 3.8 meters
    public static final LoggedTunableNumber flywheelVelocityScalarLong = 
        new LoggedTunableNumber("Shooter/Flywheel/VelocityScalarLong");
    
   // Distance thresholds for scalar selection (meters)
    public static final double shortToMidDistanceThreshold = 2;
    public static final double midToLongDistanceThreshold = 3.2;
    
    // Additive hood angle offset for long shots (radians)
    // Added to calculated hood angle when distance > midToLongDistanceThreshold
    public static final double hoodLongShotOffsetRadians = Math.toRadians(0.0);  // TODO: tune

    public static final LoggedTunableNumber flywheelRPMThreshould = 
        new LoggedTunableNumber("Shooter/FlywheelRPMThreshould");
    
        // Flywheel tolerances
    public static final LoggedTunableNumber flywheelVelToleranceRPM = 
        new LoggedTunableNumber("Shooter/Flywheel/VelToleranceRPM");
    
    // Bang-bang controller constants
    public static final LoggedTunableNumber flywheelBangBangTorqueCurrent = 
        new LoggedTunableNumber("Shooter/Flywheel/BangBangTorqueCurrent");
    public static final LoggedTunableNumber flywheelTorqueCurrentTolerance = 
        new LoggedTunableNumber("Shooter/Flywheel/TorqueCurrentTolerance");
    public static final LoggedTunableNumber flywheelBangBangDebounceSeconds = 
        new LoggedTunableNumber("Shooter/Flywheel/BangBangDebounceSeconds");
    
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
    // These define the motor positions at min and max hood angles
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
                    feedDutyCycle.initDefault(-1);  // TODO: tune
                    reverseDutyCycle.initDefault(-0.3);  // TODO: tune
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
                flywheelBangBangTorqueCurrent.initDefault(40);  // Amps for torque current bang-bang
                flywheelTorqueCurrentTolerance.initDefault(100);  // RPM tolerance to switch to torque current mode
                flywheelBangBangDebounceSeconds.initDefault(0.025);
                flywheelVelocityScalarShort.initDefault(2.17);  // TODO: tune for short range
                flywheelVelocityScalarMid.initDefault(2.17);    // Current tuned value
                flywheelVelocityScalarLong.initDefault(2.17);   // TODO: tune for long range
                
                
                // ===== Hood Sim Defaults =====
                hoodkP.initDefault(4.8);
                hoodkI.initDefault(0.0);
                hoodkD.initDefault(0.1);
                hoodkS.initDefault(0.25);
                hoodkV.initDefault(0.12);
                hoodkA.initDefault(0.01);
                hoodkG.initDefault(0.0);  // TODO: tune - find voltage to hold hood horizontal
                
                hoodMMCruiseVelRotPerSec.initDefault(80);  // 80 rps cruise
                hoodMMAccelRotPerSec2.initDefault(160);    // 160 rps/s
                hoodMMJerkRotPerSec3.initDefault(1600);    // 1600 rps/s/s
            
                hoodSensorToMechanismRatio.initDefault(7.1500);

                hoodMinPositionRot.initDefault(0.0);  
                hoodMaxPositionRot.initDefault(36.9896098);
                
                hoodPosToleranceRot.initDefault(0.5);  // ~1 degrees
                hoodVelToleranceRotPerSec.initDefault(0.1);
                hoodStatorCurrentLimit.initDefault(40);
                hoodSupplyCurrentLimit.initDefault(30);
                
                // Hood homing
                hoodHomingDutyCycle.initDefault(0.1);
                hoodHomingTimeoutSeconds.initDefault(0.75);
                hoodOutOfBoundsDebounceSeconds.initDefault(0.25);
            }
            case COMPBOT -> {
                // ===== Flywheel Comp Defaults =====
                flywheelVelToleranceRPM.initDefault(50);
                flywheelStatorCurrentLimit.initDefault(80);
                flywheelSupplyCurrentLimit.initDefault(60);
                flywheelBangBangTorqueCurrent.initDefault(40);  // TODO: tune
                flywheelTorqueCurrentTolerance.initDefault(100);  // TODO: tune
                flywheelBangBangDebounceSeconds.initDefault(0.025);
                flywheelVelocityScalarShort.initDefault(1.92);  // TODO: tune for short range
                flywheelVelocityScalarMid.initDefault(2);    // Current tuned value
                flywheelVelocityScalarLong.initDefault(2.1);   // TODO: tune for long range
                
                
                // ===== Hood Comp Defaults =====
                hoodkP.initDefault(5);  // TODO: tune
                hoodkI.initDefault(0.0);
                hoodkD.initDefault(1);  // TODO: tune
                hoodkS.initDefault(0.0);  // TODO: tune
                hoodkV.initDefault(0.0);  // TODO: tune
                hoodkA.initDefault(0.0);  // TODO: tune
                hoodkG.initDefault(1);
                
                hoodMMCruiseVelRotPerSec.initDefault(25);  // TODO: tune
                hoodMMAccelRotPerSec2.initDefault(50);     // TODO: tune
                hoodMMJerkRotPerSec3.initDefault(0);      // TODO: tune

                hoodMinPositionRot.initDefault(0.0); 
                hoodMaxPositionRot.initDefault(1.5);   // TODO: Tuned CAD
                
                hoodSensorToMechanismRatio.initDefault(7.1500);  // TODO: Tuned CAD
                
                hoodPosToleranceRot.initDefault(0.5);
                hoodVelToleranceRotPerSec.initDefault(0.1);
                hoodStatorCurrentLimit.initDefault(40);
                hoodSupplyCurrentLimit.initDefault(30);
                
                // Hood homing
                hoodHomingDutyCycle.initDefault(-0.3); //TODO
                hoodHomingTimeoutSeconds.initDefault(0.5);
                hoodOutOfBoundsDebounceSeconds.initDefault(0.25);
            }
        }
    }
}