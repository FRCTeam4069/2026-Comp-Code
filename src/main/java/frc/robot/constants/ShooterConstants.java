package frc.robot.constants;

import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterConstants {

    public static final SparkFlexConfig shooterOneMotorOneConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterOneMotorTwoConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterTwoMotorOneConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterTwoMotorTwoConfig = new SparkFlexConfig();

    public static final double kP = 0.0000001;
    public static final double kI = 0.0;

    public static final double kV = 0.001875;


    static{
        shooterOneMotorOneConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(124)
            .closedLoop.pid(kP, kI, 0)
            .feedForward
                .kV(kV);
    }

     static{
        shooterOneMotorTwoConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(124)
            .closedLoop.pid(kP, kI, 0)
            .feedForward
                .kV(kV);
    }

    static{
        shooterTwoMotorOneConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(124)
            .closedLoop.pid(kP, kI, 0)
            .feedForward
                .kV(kV);
    }

     static{
        shooterTwoMotorTwoConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(124)
            .closedLoop.pid(kP, kI, 0)
            .feedForward
                .kV(kV);
            //.follow(DeviceIDs.SHOOTER_TWO_MOTOR_ONE);
    }
  
    // public record ShooterCoefficients(
     

    //     double kV,
    //     double kA

    //     ) {}

    //     public static ShooterCoefficients shooterCoefficientsOne = new ShooterCoefficients(

    //         0.33,
    //         0.0

    //     );
    }