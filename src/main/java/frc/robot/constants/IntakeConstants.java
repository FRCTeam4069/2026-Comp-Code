package frc.robot.constants;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class IntakeConstants {

  public static final int intakeSmartCurrentLimit = 60;

  public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();

  static {
    intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeSmartCurrentLimit)
        .secondaryCurrentLimit(80)
        .openLoopRampRate(0.0)
        .closedLoopRampRate(0.0);
  }

}