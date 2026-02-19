package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DrivetrainConstants;

public class SwerveDrivetrain extends SubsystemBase {

    private SwerveModule fl, fr, bl, br;
    private Pigeon2 gyro;
    private SwerveDriveKinematics kinematics = DrivetrainConstants.kinematics;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> desiredSwervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("DesiredSwerveModuleStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("VisionPose", Pose2d.struct).publish();
    private StructPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotSpeeds", ChassisSpeeds.struct).publish();
    private StructPublisher<Pose3d> leftVision3dPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("leftVisionPose3d", Pose3d.struct).publish();
    private StructPublisher<Pose3d> rightVision3dPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("rightVisionPose3d", Pose3d.struct).publish();
    private StructPublisher<Pose3d> leftBackVision3dPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("leftBackVisionPose3d", Pose3d.struct).publish();
    private StructPublisher<Pose3d> rightBackVision3dPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("rightBackVisionPose3d", Pose3d.struct).publish();
    
    
    private SwerveModuleState[] desiredState = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    public Pose2d startingPose = new Pose2d(7.210, 0.490, Rotation2d.fromDegrees(0.0));//TODO


    private final Transform3d leftFrontTransform = new Transform3d(
                new Translation3d(Units.inchesToMeters(7.75),
                                Units.inchesToMeters(10.25),
                                Units.inchesToMeters(7.0)),
                new Rotation3d(Math.toRadians(0.0), Math.toRadians(-22.5), Math.toRadians(-35)));

    private final Transform3d rightFrontTransform = new Transform3d(
                new Translation3d(Units.inchesToMeters(7.75),
                                Units.inchesToMeters(-10.25),
                                Units.inchesToMeters(7.0)),
                new Rotation3d(Math.toRadians(0.0), Math.toRadians(-63.0), Math.toRadians(34)));

    private final Transform3d leftBackTransform = new Transform3d(
                new Translation3d(Units.inchesToMeters(-9.3),
                                Units.inchesToMeters(9.224),
                                Units.inchesToMeters(19.112)),
                new Rotation3d(Math.toRadians(-1.45), Math.toRadians(-20), Math.toRadians(-23.0))); // yaw was -1.45

    private final Transform3d rightBackTransform = new Transform3d(
                new Translation3d(Units.inchesToMeters(-9.3),
                                Units.inchesToMeters(-9.224),
                                Units.inchesToMeters(19.112)),
                new Rotation3d(Math.toRadians(1), Math.toRadians(-20), Math.toRadians(18.0)));  // yaw was -1.45

  //  private Vision visionLF;
   // private Vision visionRF;
    private Vision visionLB;
    private Vision visionRB;

    public SwerveDrivetrain() {
        fl = new SwerveModule(DrivetrainConstants.flConfig, DrivetrainConstants.flCoefficients);

        fr = new SwerveModule(DrivetrainConstants.frConfig, DrivetrainConstants.frCoefficients);

        bl = new SwerveModule(DrivetrainConstants.blConfig, DrivetrainConstants.blCoefficients);

        br = new SwerveModule(DrivetrainConstants.brConfig, DrivetrainConstants.brCoefficients);

        gyro = new Pigeon2(0, "rio");
        gyro.getConfigurator().apply(DrivetrainConstants.gyroConfig);

        swerveOdometry = new SwerveDriveOdometry(kinematics, getRawRotation2d(), getModulePositions(), startingPose);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRawRotation2d(), getModulePositions(), startingPose, VecBuilder.fill(0.8, 0.8, 0.05), VecBuilder.fill(0.5, 0.5, 2.0));
       // visionLF = new Vision("Left_Front", leftFrontTransform, startingPose);
       // visionRF = new Vision("Right_Front", rightFrontTransform, startingPose);
        visionLB = new Vision("Back_Left", leftBackTransform, startingPose);
        visionRB = new Vision("Back_Right", rightBackTransform, startingPose);

        try{
            DrivetrainConstants.config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> drive(speeds), 
            new PPHolonomicDriveController(DrivetrainConstants.translationPIDConstants, DrivetrainConstants.rotationPIDConstants), 
            DrivetrainConstants.config, 
            () -> {
                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //     return alliance.get() == DriverStation.Alliance.Red;
                // }
                return false;
            }, 
            this);
        
        SmartDashboard.putNumber("drive wheel diameter", DrivetrainConstants.wheelDiameter);
    }

    public SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volt.per(Second).of(2.0) , Volt.of(7.0), Second.of(10.0)), 
        new SysIdRoutine.Mechanism(
            this::setDriveVoltage, 
            null, 
            this));

    public SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volt.per(Second).of(1.0) , Volt.of(7.0), Second.of(20.0)), 
        new SysIdRoutine.Mechanism(
            this::setSteerVoltage, 
            null, 
            this));

    public Command alignForward() {
        return Commands.deadline(Commands.waitSeconds(1.0), run(() -> setDriveVoltage(Volt.of(0.0))));
    }

    public Rotation2d getRawRotation2d() {
        return gyro.getRotation2d();
    }

    public double getAngularVelocity() {
        return gyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180.0);
    }

    public Rotation2d getRotation2d() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            fl.getModulePosition(),
            fr.getModulePosition(),
            bl.getModulePosition(),
            br.getModulePosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        };
    }

    public void setDriveVoltage(Voltage voltage) {
        fl.setDriveVoltage(voltage.magnitude());
        fr.setDriveVoltage(voltage.magnitude());
        bl.setDriveVoltage(voltage.magnitude());
        br.setDriveVoltage(voltage.magnitude());
    }

    public void setSteerVoltage(Voltage voltage) {
        fl.setSteerVoltage(voltage.in(Volt));
        fr.setSteerVoltage(voltage.in(Volt));
        bl.setSteerVoltage(voltage.in(Volt));
        br.setSteerVoltage(voltage.in(Volt));
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        desiredState = moduleStates;
        fl.setDesiredState(moduleStates[0]);
        fr.setDesiredState(moduleStates[1]);
        bl.setDesiredState(moduleStates[2]);
        br.setDesiredState(moduleStates[3]);
    }

    /**
     * drive with robot relatve chassis speeds. corrects for angular velocity
     * @param speeds robot relative
     */
    public void drive(ChassisSpeeds speeds) {
        speeds = angularVelocitySkewCorrection(speeds);
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void fieldOrientedDrive(ChassisSpeeds speeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2d()));
    }

    public void fieldOrientedDrive(ChassisSpeeds speeds, Rotation2d heading) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading));
    }

    public ChassisSpeeds angularVelocitySkewCorrection(ChassisSpeeds robotRelativeVelocity) {
        var rawAngularVelocity = getAngularVelocity();
        var angularVelocity = new Rotation2d(rawAngularVelocity * DrivetrainConstants.angularVelocityCoefficient);
        if (angularVelocity.getRadians() != 0.0) {
            ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, getRotation2d());
            robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity, getRotation2d().plus(angularVelocity));
        }

        robotRelativeVelocity = ChassisSpeeds.discretize(robotRelativeVelocity, 0.02);

        return robotRelativeVelocity;
    }

    /**
     * Drive the robot field centric. Because joysticks have a different axis 
     * convention, all inputs need to be negative and the x and y axis need 
     * to be swapped. (this is intentional)
     * @param xVelocity velocity in the x axis
     * @param yVelocity velocity in the y axis
     * @param angleVelocity angular velocity around z axis (ccw positive)
     * @return Command
     */
    public Command driveCommand(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angleVelocity) {
        return run(() -> {
            fieldOrientedDrive(
                new ChassisSpeeds(
                    Math.pow(xVelocity.getAsDouble(), 3) * DrivetrainConstants.maxVelocity,
                    Math.pow(yVelocity.getAsDouble(), 3) * DrivetrainConstants.maxVelocity,
                    Math.pow(angleVelocity.getAsDouble(), 3) * DrivetrainConstants.maxAngularVelocity));
        });
    }

    public Command stopCommand() {
        return run(() -> stop());
    }
    
    public Command stopOnceCommand() {
        return runOnce(() -> stop());
    }

    public void stop() {
        drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public Pose2d getPose() {
        // var pose = poseEstimator.getPoseMeters();
        var pose = poseEstimator.getEstimatedPosition();
        return pose;
    }


    public void resetHeading(Rotation2d newHeading) {
        resetPose(new Pose2d(getPose().getTranslation(), newHeading));
    }

    public void resetDrivePose(Pose2d pose) {
        swerveOdometry.resetPosition(getRawRotation2d(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(getRawRotation2d(), getModulePositions(), pose);
        poseEstimator.resetPosition(getRawRotation2d(), getModulePositions(), pose);
        startingPose = pose;
    }

    public Command resetHeadingCommand() {
        return runOnce(() -> resetHeading(new Rotation2d(0.0)));
    }

    public Command increaseOffset(Rotation2d heading) {
        return runOnce(() -> resetHeading(getRotation2d().plus(heading)));
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public Command followPathCommand(String pathName) {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return new FollowPathCommand(
                path,
                this::getPose, 
                this::getRobotRelativeSpeeds, 
                (speeds, feedforwards) -> drive(speeds), 
                new PPHolonomicDriveController(DrivetrainConstants.translationPIDConstants, DrivetrainConstants.rotationPIDConstants), 
                DrivetrainConstants.config, 
                () -> {
                    return false;
                }, 
                this
            );

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }


    @Override
    public void periodic() {
        swerveOdometry.update(getRawRotation2d(), getModulePositions());
        poseEstimator.update(getRawRotation2d(), getModulePositions());

        // var estimatedLF = visionLF.getEstimatedGlobalPose();
        // if (estimatedLF.isPresent()) {
        //     leftVision3dPosePublisher.set(estimatedLF.get().estimatedPose);
        //     addVisionMeasurement(estimatedLF.get().estimatedPose.toPose2d(), estimatedLF.get().timestampSeconds, visionLF.getStdDeviations());
        // }

        // var estimatedRF = visionRF.getEstimatedGlobalPose();
        // if (estimatedRF.isPresent()) {
        //     rightVision3dPosePublisher.set(estimatedRF.get().estimatedPose);
        //     addVisionMeasurement(estimatedRF.get().estimatedPose.toPose2d(), estimatedRF.get().timestampSeconds, visionRF.getStdDeviations());
        // }

        var estimatedLB = visionLB.getEstimatedGlobalPose();
        if (estimatedLB.isPresent()) {
            leftBackVision3dPosePublisher.set(estimatedLB.get().estimatedPose);
            addVisionMeasurement(estimatedLB.get().estimatedPose.toPose2d(), estimatedLB.get().timestampSeconds, visionLB.getStdDeviations());
        }

        var estimatedRB = visionRB.getEstimatedGlobalPose();
        if (estimatedRB.isPresent()) {
            rightBackVision3dPosePublisher.set(estimatedRB.get().estimatedPose);
            addVisionMeasurement(estimatedRB.get().estimatedPose.toPose2d(), estimatedRB.get().timestampSeconds, visionRB.getStdDeviations());
        }

        visionPosePublisher.set(poseEstimator.getEstimatedPosition());
        swervePublisher.set(getModuleStates());
        desiredSwervePublisher.set(desiredState);
        posePublisher.set(swerveOdometry.getPoseMeters());
        var speeds = getRobotRelativeSpeeds();
        speedsPublisher.set(speeds);

        // for the purpose of odometry fixing
       
        

    }


}