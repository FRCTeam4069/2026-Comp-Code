package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class FeederConstants {

    public static final int feederSmartCurrentLimit = 30;

    public static final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    static {
    
                feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                feederConfig.CurrentLimits.SupplyCurrentLimit = feederSmartCurrentLimit;
                feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
             
    }

}