package frc.robot.constants;

import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterConstants {

    public static final SparkFlexConfig shooterOneMotorOneConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterOneMotorTwoConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterTwoMotorOneConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterTwoMotorTwoConfig = new SparkFlexConfig();


    static{
        shooterOneMotorOneConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0);

    }

     static{
        shooterOneMotorTwoConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0)
            .follow(DeviceIDs.SHOOTER_ONE_MOTOR_ONE);


    }

    static{
        shooterTwoMotorOneConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0);


    }

     static{
        shooterTwoMotorTwoConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0)
            .follow(DeviceIDs.SHOOTER_TWO_MOTOR_ONE);


    }
  
    public record ShooterCoefficients(
        double kP,
        double kI,
        double kD,

        double kV,
        double kA

        ) {}

        public static ShooterCoefficients shooterCoefficients = new ShooterCoefficients(
            0.75,
            0.0,
            0.0,

            0.33,
            0.4

            //FIXME tune PID and check FF
        );
    }


