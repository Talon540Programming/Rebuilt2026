package frc.robot.subsystems.LED;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDBase extends SubsystemBase {
    
    /**
     * LED States in priority order (highest to lowest)
     */
    public enum LEDState {
        ESTOP,              // Emergency stop - bright red/orange flowing
        EMERGENCY_MODE,     // Emergency mode active - cyan flowing
        DISABLED_FMS,       // Disabled + connected to FMS - alliance colors + white flowing
        DISABLED_NO_FMS,    // Disabled + no FMS - red/yellow flowing
        SHOOTING,           // Flywheel at speed - fire animation
        INTAKING_RED,
        INTAKING_BLUE,     
        IDLE                // Default state - alliance color larson
    }
    
    // Hardware
    private final CANdle candle;
    private final boolean isReal;
    
    // State
    private LEDState currentState = LEDState.IDLE;
    private LEDState previousState = null;
    private int kLEDcount = LEDConstants.kLEDCount;
    private double flickerTime = 0;
    
    // Control requests (reusable)
    private final ColorFlowAnimation colorFlowAnim = new ColorFlowAnimation(kLEDcount - (8 + kLEDcount), kLEDcount);
    private final FireAnimation fireAnim = new FireAnimation(kLEDcount - (8 + kLEDcount), kLEDcount);
    private final LarsonAnimation larsonAnim = new LarsonAnimation(kLEDcount - (8 + kLEDcount), kLEDcount);
    private final SolidColor solidColor = new SolidColor(kLEDcount - (8 + kLEDcount), kLEDcount);
    
    // Suppliers for state determination
    private final BooleanSupplier isIntakingSupplier;
    private final BooleanSupplier isShootingSupplier;
    private final BooleanSupplier isFlywheelReadySupplier;
    private final BooleanSupplier isEmergencyModeSupplier;
    
    // Simulation timing for animations
    private final Timer simAnimationTimer = new Timer();
    private boolean wasJustBlue = true;
    
    public LEDBase(
            BooleanSupplier isIntakingSupplier,
            BooleanSupplier isShootingSupplier,
            BooleanSupplier isFlywheelReadySupplier,
            BooleanSupplier isEmergencyModeSupplier) {
        
        this.isIntakingSupplier = isIntakingSupplier;
        this.isShootingSupplier = isShootingSupplier;
        this.isFlywheelReadySupplier = isFlywheelReadySupplier;
        this.isEmergencyModeSupplier = isEmergencyModeSupplier;
        
        this.isReal = RobotBase.isReal();
        
        if (isReal) {
            candle = new CANdle(LEDConstants.kCANdleId);
            configureCANdle();
        } else {
            candle = null;
            simAnimationTimer.start();
        }
    }
    
    private void configureCANdle() {
        if (candle == null) return;
        
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = LEDConstants.kBrightness;
        candle.getConfigurator().apply(config);
    }
    
    @Override
    public void periodic() {
        // Determine the current state based on priority
        LEDState newState = determineState();
        
        // Only update animation if state changed
        if (newState != previousState) {
            currentState = newState;
            applyAnimation(currentState);
            previousState = currentState;
        }

        logState();
    }
    
    /**
     * Determine the current LED state based on priority
     */
    private LEDState determineState() {
        final double currentTime = Timer.getFPGATimestamp();
        // Priority 1: Emergency Stop (highest)
        if (DriverStation.isEStopped()) {
            return LEDState.ESTOP;
        }
        
        // Priority 2: Emergency Mode
        if (isEmergencyModeSupplier.getAsBoolean()) {
            return LEDState.EMERGENCY_MODE;
        }
        
        // Priority 3: Disabled states
        if (DriverStation.isDisabled()) {
            if (DriverStation.isFMSAttached()) {
                return LEDState.DISABLED_FMS;
            } else {
                return LEDState.DISABLED_NO_FMS;
            }
        }
        
        // Priority 4: Shooting states (takes precedence over intake)
        if (isShootingSupplier.getAsBoolean()) {
            return LEDState.SHOOTING;
        }
        
        // Priority 5: Intaking (rollers running)
        if (isIntakingSupplier.getAsBoolean() && wasJustBlue == true) {
                if(currentTime - LEDConstants.flickerSpeed > flickerTime){
                    flickerTime = currentTime;
                    wasJustBlue = false;
                }
                return LEDState.INTAKING_RED;
        }
        else if (isIntakingSupplier.getAsBoolean() && wasJustBlue == false){
            if(currentTime - LEDConstants.flickerSpeed > flickerTime){
                    flickerTime = currentTime;
                    wasJustBlue = true;
                }
                return LEDState.INTAKING_BLUE;
        }
        
        // Priority 6: Idle (lowest)
        return LEDState.IDLE;
    }
    
    /**
     * Apply the appropriate animation for the given state
     */
    private void applyAnimation(LEDState state) {
        switch (state) {
            case ESTOP:
                // Bright red and orange flowing - DANGER
                setFlowAnimation(LEDConstants.RED);
                break;
                
            case EMERGENCY_MODE:
                // Cyan flowing
                setFlowAnimation(LEDConstants.CYAN);
                break;
                
            case DISABLED_FMS:
                // Alliance colors flowing
                if (isRedAlliance()) {
                    setFlowAnimation(LEDConstants.RED);
                } else {
                    setFlowAnimation(LEDConstants.BLUE);
                }
                break;
                
            case DISABLED_NO_FMS:
                // Yellow flowing
                setFlowAnimation(LEDConstants.YELLOW);
                break;
                
            case SHOOTING:
                // Fire animation when flywheel is ready
                setFireAnimation();
                break;
                
            case INTAKING_RED:
                //Strobe police lights red
                setAll(LEDConstants.RED);
                break;

            case INTAKING_BLUE:
                //Strobe police lights blue
                setAll(LEDConstants.BLUE);
                break;

            case IDLE:
                // Larson scanner in alliance color
                if (isRedAlliance()) {
                    setLarsonAnimation(LEDConstants.RED);
                } else {
                    setLarsonAnimation(LEDConstants.BLUE);
                }
                break;
        }
    }
    
    // ==================== ANIMATION METHODS ====================
    
    private void setFlowAnimation(RGBWColor color) {
        if (candle != null) {
            candle.setControl(colorFlowAnim
                .withDirection(AnimationDirectionValue.Forward)
                .withColor(color)
            );
        }
    }

    private void setAll(RGBWColor color){
        if (candle != null){
            candle.setControl(solidColor
                .withColor(color)
            );
        }
    }
    
    private void setFireAnimation() {
        if (candle != null) {
            candle.setControl(fireAnim
                .withBrightness(1.0)
                .withSparking(0.7)
                .withCooling(0.5)
            );
        }
    }
    
    private void setLarsonAnimation(RGBWColor color) {
        if (candle != null) {
            candle.setControl(larsonAnim
            .withSize(7)
            .withColor(color)
            );
        }
    }
    
    
    // ==================== LOGGING ====================
    
    private void logState() {
        Logger.recordOutput("LEDs/CurrentState", currentState.toString());
        
        // Log input states
        Logger.recordOutput("LEDs/Inputs/IsIntaking", isIntakingSupplier.getAsBoolean());
        Logger.recordOutput("LEDs/Inputs/IsShooting", isShootingSupplier.getAsBoolean());
        Logger.recordOutput("LEDs/Inputs/IsFlywheelReady", isFlywheelReadySupplier.getAsBoolean());
        Logger.recordOutput("LEDs/Inputs/IsEmergencyMode", isEmergencyModeSupplier.getAsBoolean());
        Logger.recordOutput("LEDs/Inputs/IsDisabled", DriverStation.isDisabled());
        Logger.recordOutput("LEDs/Inputs/IsFMSAttached", DriverStation.isFMSAttached());
        Logger.recordOutput("LEDs/Inputs/IsEStopped", DriverStation.isEStopped());
    }
    
    // ==================== HELPERS ====================
    
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
    
    // ==================== PUBLIC GETTERS ====================
    
    public LEDState getCurrentState() {
        return currentState;
    }
}