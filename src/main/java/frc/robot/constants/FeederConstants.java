package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class FeederConstants {

    public static final int feederSmartCurrentLimit = 30;

    public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

    static {
    
                // feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                // feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                // feederConfig.CurrentLimits.SupplyCurrentLimit = feederSmartCurrentLimit;
                // feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                feederConfig
                    .inverted(true)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(feederSmartCurrentLimit)
                    .openLoopRampRate(0.0)
                    .closedLoopRampRate(0.0);
             
    }

}