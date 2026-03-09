package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class PivotConstants {

  public static final double UPPER_POSITION = -3;
  public static final double LOWER_POSITION = -37;
  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;

  public static final int pivotSmartCurrentLimit = 40;

  public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    static {
         pivotConfig
          .inverted(false)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(pivotSmartCurrentLimit)
          .openLoopRampRate(0.0)
          .closedLoopRampRate(0.0);

          pivotConfig.softLimit
          .forwardSoftLimit(-2.0)
          .reverseSoftLimit(-38)//PivotConstants.UPPER_POSITION
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);

          //Limit on other side

          pivotConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
      
    }

  public static PIDCoefficients upCoefficients = new PIDCoefficients(0.0625, 0.0, 0.001);
  public static PIDCoefficients downCoefficients = new PIDCoefficients(0.0625, 0.0, 0.001);

 }



