package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FieldCentricDrive extends Command {
    private final SwerveDrivetrain drive;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier turnSpeed;
    private final BooleanSupplier autoAlign;
    private final BooleanSupplier resetOdometry;
    private BooleanSupplier lockHeading;
    private BooleanSupplier lockClosest;
    private final BooleanSupplier missWalls;
    private final DoubleSupplier snapModulesAxis;
    private final BooleanSupplier leftSnap;
    private final BooleanSupplier rightSnap;
    private final BooleanSupplier frontSnap;
    private final BooleanSupplier backSnap;
    private final BooleanSupplier fastTurn;

    private Pose2d withVision;
    private Pose2d encoderOnly;
    private Pose2d odometryError;

    private final double FIELD_MIN_X = 1;
    private final double FIELD_MAX_X = 15.54;
    private final double FIELD_MIN_Y = 1;
    private final double FIELD_MAX_Y = 7;

    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(110.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(110.0);

    private PIDController higherHeadingController = new PIDController(
            DrivetrainConstants.higherHeadingCoefficients.kP(),
            DrivetrainConstants.higherHeadingCoefficients.kI(),
            DrivetrainConstants.higherHeadingCoefficients.kD());

    private PIDController middleHeadingController = new PIDController(
            DrivetrainConstants.middleHeadingCoefficients.kP(),
            DrivetrainConstants.middleHeadingCoefficients.kI(),
            DrivetrainConstants.middleHeadingCoefficients.kD());

    private PIDController lowerHeadingController = new PIDController(
            DrivetrainConstants.lowerHeadingCoefficients.kP(),
            DrivetrainConstants.lowerHeadingCoefficients.kI(),
            DrivetrainConstants.lowerHeadingCoefficients.kD());

    private Pose2d currentPosition;
    private double desiredHeading = 0.0;
    private Alliance alliance = Alliance.Blue;

    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);

    private double deltaX = 0.0;
    private double deltaY = 0.0;

    private double tolerance = 0.75;
    double rotationalSpeed = 0.0;

    // private final BooleanSupplier throughTrench;
    // private final ThroughTrench controller;
    // private boolean trenchActive = false;
    // private ChassisSpeeds trenchSpeeds;

    private ChassisSpeeds driveSpeeds;
    private boolean lockHeadingActive = false;
    private double lockHeadingTarget = 0.0;
    private boolean lockClosestActive = false;
    private double lockClosestTarget;

    private double buttonTarget = 0.0;
    private boolean buttonActive = false;

    private DoublePublisher desiredHeadingPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("desiredHeading").publish();
    private DoublePublisher deltaXDoublePublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("deltaX").publish();
    private DoublePublisher deltaYDoublePublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("deltaY").publish();
    private StructPublisher<Pose2d> setPointPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("setPoint", Pose2d.struct).publish();

    private StructPublisher<Pose2d> odometryErrorPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("odometryError", Pose2d.struct).publish();

    private static double controllerDeadband = 0.05;
    private static final double SNAP_MODULE_ANGLE_THRESH_0 = 0.65;
    private static final double SNAP_MODULE_ANGLE_THRESH_180 = -0.65;

    /**
     * Teleop drive command
     * 
     * @param drive        swerve drivetrain
     * @param forwardSpeed -1.0 to 1.0
     * @param strafeSpeed  -1.0 to 1.0
     * @param turnSpeed    -1.0 to 1.0
     * @param autoAlign    true for align to goal
     */
    public FieldCentricDrive(
            SwerveDrivetrain drive,
            DoubleSupplier forwardSpeed,
            DoubleSupplier strafeSpeed,
            DoubleSupplier turnSpeed,
            BooleanSupplier autoAlign,
            BooleanSupplier resetOdometry,
            BooleanSupplier lockClosest,
            // BooleanSupplier throughTrench,
            BooleanSupplier lockHeading,
            BooleanSupplier missWalls,
            DoubleSupplier snapModuleAxis,
            BooleanSupplier leftSnap,
            BooleanSupplier rightSnap,
            BooleanSupplier frontSnap,
            BooleanSupplier backSnap,
            BooleanSupplier fastTurn) {

        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.autoAlign = autoAlign;
        this.resetOdometry = resetOdometry;
        this.lockClosest = lockClosest;
        // this.throughTrench = throughTrench;
        this.lockHeading = lockHeading;
        this.missWalls = missWalls;
        this.snapModulesAxis = snapModuleAxis;
        this.leftSnap = leftSnap;
        this.rightSnap = rightSnap;
        this.frontSnap = frontSnap;
        this.backSnap = backSnap;
        this.fastTurn = fastTurn;
        // this.controller = new ThroughTrench(drive);

        higherHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        middleHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        lowerHeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        currentPosition = drive.getPose();

        var result = DriverStation.getAlliance();

        if (result.isPresent()) {
            alliance = result.get();
        }

        // if (throughTrench.getAsBoolean()){

        // if (!trenchActive) {
        // controller.initialize();
        // trenchActive = true;
        // }

        // trenchSpeeds = controller.getSpeeds();
        // trenchSpeeds.omegaRadiansPerSecond = - trenchSpeeds.omegaRadiansPerSecond;

        // }

        // else {
        // trenchActive = false;
        // }

        // RESET ODOMETRY
        if (resetOdometry.getAsBoolean()) {
            drive.resetDrivePose(currentPosition);
        }

        var outputSpeeds = new ChassisSpeeds(
                xSlewRateLimiter.calculate(joystickToVelocity(forwardSpeed.getAsDouble())),
                ySlewRateLimiter.calculate(joystickToVelocity(strafeSpeed.getAsDouble())),
                (Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3) / 1.5)
                        * DrivetrainConstants.maxAngularVelocity);

        if (fastTurn.getAsBoolean()) {
            outputSpeeds.omegaRadiansPerSecond = (Math
                    .pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3))
                    * DrivetrainConstants.maxAngularVelocity;

        } else {
            outputSpeeds.omegaRadiansPerSecond = (Math
                    .pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3) / 1.25)
                    * DrivetrainConstants.maxAngularVelocity;

        }

        // SNAP MODULES 180/0 WITH JOYSTICK
        if (snapModulesAxis.getAsDouble() < SNAP_MODULE_ANGLE_THRESH_180) {

            if (alliance == Alliance.Blue) {
                double target = 0;
                rotationalSpeed = lowerHeadingController.calculate(drive.getRotation2d().getRadians(),
                        Math.toRadians(target));
                outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
            } else if (alliance == Alliance.Red) {
                double target = 180;
                rotationalSpeed = lowerHeadingController.calculate(drive.getRotation2d().getRadians(),
                        Math.toRadians(target));
                outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
            }

        } else if (snapModulesAxis.getAsDouble() > SNAP_MODULE_ANGLE_THRESH_0) {

            if (alliance == Alliance.Blue) {
                double target = 180;
                rotationalSpeed = lowerHeadingController.calculate(drive.getRotation2d().getRadians(),
                        Math.toRadians(target));
                outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
            } else if (alliance == Alliance.Red) {
                double target = 0;
                rotationalSpeed = lowerHeadingController.calculate(drive.getRotation2d().getRadians(),
                        Math.toRadians(target));
                outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
            }
        }

        // SNAP MODULES FOR WALLPICKUP left and right
        if (leftSnap.getAsBoolean()) {
            double currentDeg = drive.getRotation2d().getDegrees();
            double target = 0;

            if (alliance == Alliance.Blue) {

                if ((currentDeg < 90 && currentDeg > 0) || (currentDeg > -90 && currentDeg < 0)) {
                    target = 18;
                } else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg < -90 && currentDeg > -180)) {
                    target = 162;
                }

            }

            else if (alliance == Alliance.Red) {
                if ((currentDeg < 90 && currentDeg > 0) || (currentDeg > -90 && currentDeg < 0)) {
                    target = -18;
                } else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg < -90 && currentDeg > -180)) {
                    target = -162;
                }

            }

            rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                    Math.toRadians(target));
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;

        } else if (rightSnap.getAsBoolean()) {

            double currentDeg = drive.getRotation2d().getDegrees();
            double target = 0;

            if (alliance == Alliance.Blue) {

                if ((currentDeg < 90 && currentDeg > 0) || currentDeg > -90 && currentDeg < 0) {
                    target = -18;
                }

                else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg < -90 && currentDeg > -180)) {
                    target = -162;
                }
            } else if (alliance == Alliance.Red) {

                if ((currentDeg < 90 && currentDeg > 0) || currentDeg > -90 && currentDeg < 0) {
                    target = 18;
                }

                else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg < -90 && currentDeg > -180)) {
                    target = 162;
                }
            }
            rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                    Math.toRadians(target));
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;

        }

        // front and back pmo. you know what you did.
        else if (frontSnap.getAsBoolean()) {

            double currentDeg = drive.getRotation2d().getDegrees();
            double target = 0;

            if (alliance == Alliance.Red) {

                if ((currentDeg < 0 && currentDeg > -90) || (currentDeg < -90 && currentDeg > -180)) {
                    target = -108;
                } else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg > 0 && currentDeg < 90)) {
                    target = 108;
                }
            }

            else if (alliance == Alliance.Blue) {

                if ((currentDeg < 0 && currentDeg > -90) || (currentDeg < -90 && currentDeg > -180)) {
                    target = -72;
                } else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg > 0 && currentDeg < 90)) {
                    target = 72;
                }
            }

            SmartDashboard.getNumber("target", target);

            rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                    Math.toRadians(target));
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;

        } else if (backSnap.getAsBoolean()) {

            double currentDeg = drive.getRotation2d().getDegrees();
            double target = 0;

            if (alliance == Alliance.Red) {

                if ((currentDeg < 0 && currentDeg > -90) || (currentDeg < -90 && currentDeg > -180)) {
                    target = -72;
                } else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg > 0 && currentDeg < 90)) {
                    target = 72;
                }
            }

            else if (alliance == Alliance.Blue) {

                if ((currentDeg < 0 && currentDeg > -90) || (currentDeg < -90 && currentDeg > -180)) {
                    target = -108;
                } else if ((currentDeg > 90 && currentDeg < 180) || (currentDeg > 0 && currentDeg < 90)) {
                    target = 108;
                }
            }

            SmartDashboard.getNumber("target", target);

            rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                    Math.toRadians(target));
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
        }

        if (alliance == Alliance.Blue) {
            deltaX = blueHubX - currentPosition.getX();
            deltaY = blueHubY - currentPosition.getY();
        } else {
            deltaX = redHubX - currentPosition.getX();
            deltaY = redHubY - currentPosition.getY();
        }

        desiredHeading = Math.atan(deltaY / deltaX);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        desiredHeadingPublisher.set(Math.toDegrees(-desiredHeading));

        if (autoAlign.getAsBoolean()) {
            if (alliance == Alliance.Blue) {
                rotationalSpeed = higherHeadingController.calculate(drive.getRotation2d().getRadians(), desiredHeading);

                if (Math.abs(higherHeadingController.getError()) >= Math.toRadians(tolerance)) {
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
                }
            } else {
                rotationalSpeed = higherHeadingController.calculate(
                        drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians(), desiredHeading);

                if (Math.abs(higherHeadingController.getError()) >= Math.toRadians(tolerance)) {
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
                }
            }

        }

        if (lockHeading.getAsBoolean() && !lockClosestActive) {
            if (!lockHeadingActive) {
                lockHeadingTarget = drive.getRotation2d().getRadians();
                lockHeadingActive = true;
            }

            rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(), lockHeadingTarget);
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
        } else {
            lockHeadingActive = false;
        }

        if (lockClosest.getAsBoolean() && !lockHeadingActive) {
            if (!lockClosestActive) {
                lockClosestTarget = drive.getRotation2d().getDegrees();
                if (alliance == Alliance.Blue) {

                    deltaX = blueHubX - currentPosition.getX();
                    deltaY = blueHubY - currentPosition.getY();
                }

                else {

                    deltaX = redHubX - currentPosition.getX();
                    deltaY = redHubY - currentPosition.getY();
                }

                desiredHeading = Math.atan(deltaY / deltaX);

                SmartDashboard.putNumber("desiredHeading", desiredHeading);

                desiredHeadingPublisher.set(Math.toDegrees(-desiredHeading));

                // deltaXDoublePublisher.set(deltaX);
                // deltaYDoublePublisher.set(deltaY);

                if (autoAlign.getAsBoolean()) {

                    if (alliance == Alliance.Blue) {
                        rotationalSpeed = higherHeadingController.calculate(drive.getRotation2d().getRadians(),
                                desiredHeading);

                        if (Math.abs(higherHeadingController.getError()) >= Math.toRadians(tolerance)) {
                            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
                        }
                    } else {
                        rotationalSpeed = higherHeadingController.calculate(
                                drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians(),
                                desiredHeading);

                        if (Math.abs(higherHeadingController.getError()) >= Math.toRadians(tolerance)) {
                            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
                        }
                    }

                }

                if (lockHeading.getAsBoolean() && !lockClosestActive) {

                    if (!lockHeadingActive) {
                        lockHeadingTarget = drive.getRotation2d().getRadians();
                        lockHeadingActive = true;
                    }

                    rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                            lockHeadingTarget);
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
                }

                else {
                    lockHeadingActive = false;
                }

                if (lockClosest.getAsBoolean() && !lockHeadingActive) {
                    if (!lockClosestActive) {
                        lockClosestTarget = drive.getRotation2d().getDegrees();

                        lockClosestTarget = Math.toRadians(90); // keep for test

                        // if (lockClosestTarget >90 || lockClosestTarget <-90) {
                        // lockClosestTarget = Math.toRadians(180);
                        // desiredModuleState.set(Math.toRadians(180));

                        // }
                        // else {
                        // lockClosestTarget = Math.toRadians(0);
                        // desiredModuleState.set(Math.toRadians(0));

                        // }

                        lockClosestActive = true;
                    }

                    rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                            lockClosestTarget);
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
                }

                else {
                    lockClosestActive = false;
                }

                if (lockClosestTarget > 90 || lockClosestTarget < -90) {
                    lockClosestTarget = Math.toRadians(180);
                } else {
                    lockClosestTarget = Math.toRadians(0);
                }

                lockClosestActive = true;
            }

            rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(), lockClosestTarget);
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
        } else {
            lockClosestActive = false;
        }

        outputSpeeds.omegaRadiansPerSecond = -outputSpeeds.omegaRadiansPerSecond;

        // if(throughTrench.getAsBoolean()){
        // driveSpeeds = trenchSpeeds;
        // }

        // else{
        driveSpeeds = outputSpeeds;
        // }

        if (missWalls.getAsBoolean()) {
            if (currentPosition.getX() <= FIELD_MIN_X && driveSpeeds.vxMetersPerSecond > 0) {
                driveSpeeds.vxMetersPerSecond = 0;
            }
            if (currentPosition.getX() >= FIELD_MAX_X && driveSpeeds.vxMetersPerSecond < 0) {
                driveSpeeds.vxMetersPerSecond = 0;
            }
            if (currentPosition.getY() <= FIELD_MIN_Y && driveSpeeds.vyMetersPerSecond > 0) {
                driveSpeeds.vyMetersPerSecond = 0;
            }
            if (currentPosition.getY() >= FIELD_MAX_Y && driveSpeeds.vyMetersPerSecond < 0) {
                driveSpeeds.vyMetersPerSecond = 0;
            }
        }

        if (alliance == Alliance.Blue) {
            drive.fieldOrientedDrive(driveSpeeds);
        } else {
            drive.fieldOrientedDrive(driveSpeeds, drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)));
        }

        withVision = drive.poseEstimator.getEstimatedPosition();
        encoderOnly = drive.swerveOdometry.getPoseMeters();

        odometryError = encoderOnly.relativeTo(withVision);
        odometryErrorPublisher.set(odometryError);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    private double joystickToVelocity(double n) {
        return Math.pow(MathUtil.applyDeadband(n, controllerDeadband), 3) * DrivetrainConstants.maxVelocity;
    }

}
