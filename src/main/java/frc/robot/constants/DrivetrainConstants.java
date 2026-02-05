package frc.robot.constants;


import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

    public static final int driveCurrentLimit = 50;
    public static final int steerCurrentLimit = 25;
    public volatile static double wheelDiameter = 3.9; // 3.94in
    //bl 3.955 -> 3.912
    //fl 3.9545
    //fr 3.950
    //br 3.945
    public static final double driveConversionFactor = ((wheelDiameter * Math.PI) * 0.0254) / 6.12;
    public static final double steerConversionFactor = 16.8;

    public static final boolean driveInverted = false;
    public static final boolean steerInverted = true;
    public static final boolean encoderInverted = true;

    public static final double flEncoderOffset = -0.02929722222; 
    public static final double frEncoderOffset = -0.91626111111;
    public static final double blEncoderOffset = -0.0708;
    public static final double brEncoderOffset = -0.1739;

    public static final Pigeon2Configuration gyroConfig = new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(0.0));

    public static final double moduleOffset = Units.inchesToMeters(10.375);
    
    public static final double maxVelocity = Units.feetToMeters(19.3);
    public static final double maxAngularVelocity = maxVelocity / new Rotation2d(moduleOffset, moduleOffset).getRadians();

    public static final double angularVelocityCoefficient = 0.04;
    public static final double angularVelocityDeadband = 0.01;
    public static final double headingCorrectionDeadband = 0.05;

    public static final double mass = Pounds.of(123.0).in(Kilograms);


     public static final Pose2d[] redShooterPoses = new Pose2d[]{
        new Pose2d(3.69, 3.02, Rotation2d.fromDegrees(60.0)), //red left //TODO
        new Pose2d(3.22, 4.21, Rotation2d.fromDegrees(0.0)), //red right //TODO
    };

    public static final Pose2d[] blueShooterPoses = new Pose2d[]{
        new Pose2d(2.304, 7.402, Rotation2d.fromDegrees(-52.5)), //blue left
        new Pose2d(2.304, 0.668, Rotation2d.fromDegrees(52.5)), //blue right
    };


    public static final Pose2d[] humanPlayerPoses = new Pose2d[]{
        new Pose2d(1.25, 0.700, Rotation2d.fromDegrees(180)), // red.  //TODO
        new Pose2d(0.489, 0.659, Rotation2d.fromDegrees(180)), // blue.  //TODO
    };

    public static Pose2d getHumanPlayerPose(HumanPlayerStations station) {
        switch (station) {
            case Red:
                return humanPlayerPoses[0];
            case Blue:
                return humanPlayerPoses[1];
            default:
            throw new IllegalArgumentException("Invalid station: " + station);}

        }

    public static enum ShooterPoses{
       RedLeft,
       RedRight,
       BlueLeft,
       BlueRight
    };

    public static enum HumanPlayerStations{
        Red,
        Blue
    };



    public static Pose2d getShooterPose(HumanPlayerStations station, ShooterPoses shooterpose) {
        Pose2d[] array;
        
        switch (station) {
            case Red:
                array = redShooterPoses;
                break;
            case Blue:
                array = blueShooterPoses;
                break;
            default:
                throw new IllegalArgumentException("Invalid HumanPlayerStation: " + station);
        }

        Pose2d result = new Pose2d();

        switch (shooterpose) {
            case RedLeft:
                result = array[0];
                break;
            case RedRight:
                result = array[1];
                break;
            case BlueLeft:
                result = array[2];
                break;
            case BlueRight:
                result = array[3];
                break;
            default:
                throw new IllegalArgumentException("InvalidShooterPose: " + shooterpose);
        }

        return result;
    }

    // public static final Pose2d[] blueClimbPoses = new Pose2d[]{
    //     new Pose2d(7.8, 7.26, Rotation2d.fromDegrees(180.0)),//. //TODO
    //     new Pose2d(7.8, 6.160, Rotation2d.fromDegrees(180.0)),//. //TODO
    //     new Pose2d(7.8, 5.15, Rotation2d.fromDegrees(180.0))//. //TODO
    // };

    // public static final Pose2d[] redClimbPoses = new Pose2d[]{
    //     new Pose2d(9.75, 3.06, Rotation2d.fromDegrees(0.0)),//. //TODO
    //     new Pose2d(9.75, 2.01, Rotation2d.fromDegrees(0.0)),//. //TODO
    //     new Pose2d(9.75, 0.94, Rotation2d.fromDegrees(0.0))//. //TODO
    // };

    // offset = 0.164m

    public static RobotConfig config;

    public static final PIDConstants translationPIDConstants = new PIDConstants(0.5, 0.0, 0.0);
    public static final PIDConstants rotationPIDConstants = new PIDConstants(0.1, 0.0, 0.0);

    public static double yScalar = 0.93;

    /*
     * 26ft 5in x 57ft 6(7/8)in
     * 8.052m x 17.548m
     */
    

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(moduleOffset, moduleOffset),//FL
        new Translation2d(moduleOffset, -moduleOffset),//FR
        new Translation2d(-moduleOffset, moduleOffset),//BL
        new Translation2d(-moduleOffset, -moduleOffset) //BR
    );

    public record PIDCoefficients (
        double kP,
        double kI,
        double kD
    ) {}

    public record FFCoefficients (
        double kS,
        double kV,
        double kA,
        double kG
    ) {}

    public record Tolerances (
        double position,
        double velocity
    ) {}

    public record DrivetrainPIDConstants (
        PIDCoefficients translationCoefficients,
        PIDCoefficients rotationCoefficients,
        Constraints translationConstraints,
        Constraints rotationConstraints,
        Tolerances translationTolerances,
        Tolerances rotationTolerances
    ) {}

    public static volatile DrivetrainPIDConstants pidToPositionConstants = new DrivetrainPIDConstants(
        new PIDCoefficients(8.0, 0.0, 0.0), 
        new PIDCoefficients(10.0, 0.0, 0.4), 
        new Constraints(5.0, 3.0), 
        new Constraints(10.0, 10.0), 
        new Tolerances(0.01, 0.20), 
        new Tolerances(0.02, 0.20));

    public static volatile DrivetrainPIDConstants autoPidToPositionConstants = new DrivetrainPIDConstants(
        new PIDCoefficients(4.0, 0.0, 0.01), 
        new PIDCoefficients(8.0, 0.0, 0.6), 
        new Constraints(5.0, 4.0), 
        new Constraints(10.0, 10.0), 
        new Tolerances(0.02, 0.30), 
        new Tolerances(0.02, 0.20));

    public static volatile DrivetrainPIDConstants autoCloseEnoughConstants = new DrivetrainPIDConstants(
        new PIDCoefficients(4.0, 0.0, 0.01), 
        new PIDCoefficients(8.0, 0.0, 0.6), 
        new Constraints(5.0, 4.0), 
        new Constraints(10.0, 10.0), 
        new Tolerances(0.04, 0.40), 
        new Tolerances(0.04, 0.30));
    
    public static final PIDCoefficients teleOpHeadingCoefficients = new PIDCoefficients(8.0, 0.0, 0.0);

    public record ModuleCoefficients(
        double steerKS,
        double steerKV,
        double steerKA,

        double steerKP,
        double steerKI,
        double steerKD,

        double driveKS,
        double driveKV,
        double driveKA,

        double driveKP,
        double driveKI,
        double driveKD
    ) {}

    public static ModuleCoefficients flCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.061811,
        // 2.311,
        // 0.23706,

        // 0.07424,
        // 2.2829,
        // 0.39404,

        0.47,
        2.20,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients frCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.061811,
        // 2.3088,
        // 0.26326,

        // 0.25117,
        // 2.2867,
        // 0.14551,

        0.01,
        2.14,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients blCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.066838,
        // 2.3095,
        // 0.25683,

        // 0.011768,
        // 2.3304,
        // 0.35736,

        0.17,
        2.18,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients brCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.081577,
        // 2.3094,
        // 0.23706,

        // 0.12165,
        // 2.2853,
        // 0.30262,

        0.00,
        2.14,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public record ModuleConfig(
        int driveId, 
        boolean driveInverted,
        int steerId, 
        boolean steerInverted, 
        int encoderId, 
        double encoderOffset, 
        boolean encoderInverted
    ) {};

    public static final ModuleConfig flConfig = new ModuleConfig(
        DeviceIDs.DRIVE_FL, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_FL, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_FL, DrivetrainConstants.flEncoderOffset, DrivetrainConstants.encoderInverted
    );

    public static final ModuleConfig frConfig = new ModuleConfig(
        DeviceIDs.DRIVE_FR, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_FR, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_FR, DrivetrainConstants.frEncoderOffset, DrivetrainConstants.encoderInverted
    );

    public static final ModuleConfig blConfig = new ModuleConfig(
        DeviceIDs.DRIVE_BL, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_BL, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_BL, DrivetrainConstants.blEncoderOffset, DrivetrainConstants.encoderInverted
    );

    public static final ModuleConfig brConfig = new ModuleConfig(
        DeviceIDs.DRIVE_BR, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_BR, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_BR, DrivetrainConstants.brEncoderOffset, DrivetrainConstants.encoderInverted
    );
}