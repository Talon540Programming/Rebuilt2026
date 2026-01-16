package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RollerIOSim implements RollerIO {
    
    // Constants for simulation
    private static final double kGearing = 3.0; // Gear ratio
    private static final double kMOI = 0.01; // kg*m^2
    
    // Create the plant (linear system model) first
    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44(1),
        kMOI,
        kGearing
    );
    
    // FlywheelSim constructor in 2026: (LinearSystem plant, DCMotor gearbox, double... measurementStdDevs)
    private final FlywheelSim sim = new FlywheelSim(
        plant,
        DCMotor.getKrakenX44(1)
    );
    
    private double appliedVolts = 0.0;
    private boolean isRunning = false;
    private boolean simulatedGamePiece = false;
    
    public void simulateGamePieceContact(boolean contact) {
        this.simulatedGamePiece = contact;
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        sim.update(0.02);
        
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        
        // Simulate current spike when game piece contacts
        if (isRunning && simulatedGamePiece) {
            inputs.currentAmps = sim.getCurrentDrawAmps() + 20.0;  // Extra load
        } else {
            inputs.currentAmps = sim.getCurrentDrawAmps();
        }
        
        inputs.tempCelsius = 25.0;
        inputs.hasGamePiece = isRunning && simulatedGamePiece;
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        isRunning = Math.abs(dutyCycle) > 0.01;
        appliedVolts = dutyCycle * 12.0;
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop() {
        isRunning = false;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
    
    /** For testing - simulate a game piece being present */
    public void setSimulatedGamePiece(boolean hasGamePiece) {
        this.simulatedGamePiece = hasGamePiece;
    }

    public class SimConstants{
         // existing IDs...
        public static final int kPivotMotorId = 13;
        public static final int kRollerMotorId = 14;

        // Homing
        public static final double kPivotHomingDutyCycle = -0.15;       // Slow, toward stowed
        public static final double kPivotHomingVelThreshold = 0.05;     // rad/sec - "stopped"
        public static final double kPivotHomingCurrentThreshold = 5.0;  // Amps - confirms stall

        // ===================== Pivot Motion Magic =====================
        /** Rotor rotations per 1 mechanism rotation (gear ratio). REQUIRED for correct units. */
        public static final double kPivotSensorToMechanismRatio = 12; /* TODO */;

        /** Mechanism rotations for stowed/deployed. Pick either rotations OR compute from degrees. */
        public static final double kPivotStowedPosRot = 0; /* TODO */
        public static final double kPivotDeployedPosRot = 0.215; /* TODO */

        public static final double kDeployDutyCycle = 0.5;
        public static final double kRollerIntakeDutyCycle = 0.8;
        public static final double kRollerEjectDutyCycle = -0.5;

        /** How close is “at goal” */
        public static final double kPivotPosToleranceRot = 0.01; /* TODO e.g. 0.01 */
        public static final double kPivotVelToleranceRotPerSec = 0.5; /* TODO e.g. 0.05 */

        // Motion Magic constraints (mechanism units if SensorToMechanismRatio is set)
        public static final double kPivotMMCruiseVelRotPerSec = 15; /* TODO */
        public static final double kPivotMMAccelRotPerSec2 = 30; /* TODO */
        public static final double kPivotMMJerkRotPerSec3 = 0; /* TODO (optional) */

        // Slot0 PID (tune on real robot)
        public static final double kPivotkP = 100; /* TODO */
        public static final double kPivotkI = 0.1; 
        public static final double kPivotkD = 0; /* TODO */
        public static final double kPivotkS = 0.0;  // TODO - tune: voltage to overcome static friction
        public static final double kPivotkV = 0.0;  // TODO - tune: voltage per rotation per second

        // Optional gravity/feedforward support (if you use it)
        public static final double kPivotkG = 0; /* TODO maybe 0 to start */

        public static final double kGamePieceCurrentThreshold  = 25;

        // ===================== Crash / stall detection =====================
        public static final double kPivotCrashCurrentAmps = 50; 
        public static final double kPivotCrashMinErrorRot = 1; /* TODO e.g. 0.05 */;
        public static final double kPivotCrashMaxVelRotPerSec = 0.1; /* TODO e.g. 0.1 */;
        public static final double kPivotCrashDebounceSecs = 0.15; /* TODO e.g. 0.15 */;

        public static final double kPivotAllowedErrorRot = 0.1;
        public static final double kPivotAllowedVelRotPerSec = 5;
        public static final double kPivotCrashIgnoreAfterGoalChangeSecs = 0.5;
    }
}