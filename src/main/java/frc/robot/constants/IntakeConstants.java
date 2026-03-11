package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class IntakeConstants {


  public static final int intakeSmartCurrentLimit = 40;

  public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();


    static {
      intakeConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(intakeSmartCurrentLimit)
          .openLoopRampRate(0.0)
          .closedLoopRampRate(0.0);
    }

 }





