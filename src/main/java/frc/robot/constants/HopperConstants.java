package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class HopperConstants {

    public static final int hopperSmartCurrentLimit = 30;

    public static final SparkMaxConfig hopperOneConfig = new SparkMaxConfig();
    public static final TalonFXConfiguration hopperTwoConfig = new TalonFXConfiguration();


    static {
        // hopperOneConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // hopperOneConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // hopperOneConfig.CurrentLimits.SupplyCurrentLimit = hopperSmartCurrentLimit;
        // hopperOneConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hopperOneConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(hopperSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);

                
    }

     static {
            hopperTwoConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            hopperTwoConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            hopperTwoConfig.CurrentLimits.SupplyCurrentLimit = hopperSmartCurrentLimit;
            hopperTwoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                
    }

}