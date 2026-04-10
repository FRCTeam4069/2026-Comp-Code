package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class HopperConstants {

    public static final int hopperSmartCurrentLimit = 30;

    public static final TalonFXConfiguration hopperOneConfig = new TalonFXConfiguration();


    static {
        hopperOneConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperOneConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hopperOneConfig.CurrentLimits.SupplyCurrentLimit = hopperSmartCurrentLimit;
        hopperOneConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopperOneConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.375;
        hopperOneConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.375;
                
    }


}