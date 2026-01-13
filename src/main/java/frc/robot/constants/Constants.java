// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

    public static final int motor1ID = 1;
    public static final int motor2ID = 2;

    public static TalonFXConfiguration motor1config;
    public static TalonFXConfiguration motor2config;

  
  static{
    motor1config = new TalonFXConfiguration();
    motor1config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor1config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor1config.CurrentLimits.SupplyCurrentLimitEnable=true;
    motor1config.CurrentLimits.SupplyCurrentLimit=40;

    motor2config = new TalonFXConfiguration();
    motor2config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor2config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor2config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor2config.CurrentLimits.SupplyCurrentLimit = 40;
  }


}
