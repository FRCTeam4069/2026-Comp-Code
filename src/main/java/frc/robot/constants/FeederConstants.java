package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public final class FeederConstants {

    public static final int feederSmartCurrentLimit = 30;

    public static final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    static {
    
                feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                feederConfig.CurrentLimits.SupplyCurrentLimit = feederSmartCurrentLimit;
                feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                feederConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.75;
                feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.75;

                // feederConfig
                //     .inverted(true)
                //     .idleMode(IdleMode.kCoast)
                //     .smartCurrentLimit(feederSmartCurrentLimit)
                //     .openLoopRampRate(0.0)
                //     .closedLoopRampRate(0.0);
             
    }

}