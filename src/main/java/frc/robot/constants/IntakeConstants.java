package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class IntakeConstants {

  public static final int UPPER_POSITION = 65;
  public static final int LOWER_POSITION = 12;

  public static final int feedSmartCurrentLimit = 40;
  public static final int pivotSmartCurrentLimit = 20;

  public static final SparkMaxConfig feedConfig = new SparkMaxConfig();
  public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();


    static {
      feedConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(feedSmartCurrentLimit)
          .openLoopRampRate(0.0)
          .closedLoopRampRate(0.0);
    }

    static {
         pivotConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(pivotSmartCurrentLimit)
          .openLoopRampRate(0.0)
          .closedLoopRampRate(0.0);

          pivotConfig.softLimit
          .forwardSoftLimit(0)
          .reverseSoftLimit(IntakeConstants.UPPER_POSITION)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);
    }


  public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0.0, 0.0);
  public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0);


  // NOTE: not sure if these are needed quite yet
  //public static Constraints constraints = new Constraints(100000.0, 100000.0);
  //public static double positionTolerance = 0.02;
  //public static double velocityTolerance = 1.00;


 }





