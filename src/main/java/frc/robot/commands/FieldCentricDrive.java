package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(100.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(100.0);
    private PIDController headingController = new PIDController(
        DrivetrainConstants.teleOpHeadingCoefficients.kP(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kI(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kD());
    private Pose2d currentPosition;
    private double desiredHeading = 0.0;
    private Alliance alliance = Alliance.Blue;
    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);
    private double deltaX = 0.0;
    private double deltaY = 0.0;
    private double tolerance = 2.0;
    double rotationalSpeed = 0.0;

    private DoublePublisher desiredHeadingPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("desiredHeading").publish();
    private DoublePublisher deltaXDoublePublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("deltaX").publish();
    private DoublePublisher deltaYDoublePublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("deltaY").publish();



    private static double controllerDeadband = 0.05;
    /**
     * Teleop drive command
     * @param drive swerve drivetrain
     * @param forwardSpeed -1.0 to 1.0
     * @param strafeSpeed -1.0 to 1.0
     * @param turnSpeed -1.0 to 1.0
     * @param autoAlign true for align to goal
     */
    public FieldCentricDrive(
        SwerveDrivetrain drive, 
        DoubleSupplier forwardSpeed, 
        DoubleSupplier strafeSpeed, 
        DoubleSupplier turnSpeed, 
        BooleanSupplier autoAlign,
        BooleanSupplier resetOdometry) {

        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.autoAlign = autoAlign;
        this.resetOdometry = resetOdometry;

        addRequirements(drive);
    }
    @Override
    public void initialize() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        currentPosition= drive.getPose();

        var result = DriverStation.getAlliance();
        if (result.isPresent()) {
            alliance = result.get();
        }
    }

    @Override
    public void execute() {

        var outputSpeeds = new ChassisSpeeds(
            xSlewRateLimiter.calculate(joystickToVelocity(forwardSpeed.getAsDouble())),
            ySlewRateLimiter.calculate(joystickToVelocity(strafeSpeed.getAsDouble())),
            Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3) * DrivetrainConstants.maxAngularVelocity);
        
        currentPosition= drive.getPose();

        if (alliance == Alliance.Blue){

            deltaX = blueHubX - currentPosition.getX();
            deltaY = blueHubY - currentPosition.getY();
        }

        else{

            deltaX = redHubX - currentPosition.getX();
            deltaY = redHubY - currentPosition.getY();
        }

        desiredHeading = Math.atan(deltaY / deltaX);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        desiredHeadingPublisher.set(Math.toDegrees(-desiredHeading));
        deltaXDoublePublisher.set(deltaX);
        deltaYDoublePublisher.set(deltaY);

            
        if (autoAlign.getAsBoolean()){
            rotationalSpeed = headingController.calculate(drive.getRotation2d().getRadians(), desiredHeading);

            if (Math.abs(headingController.getError()) >= Math.toRadians(tolerance)){
                outputSpeeds.omegaRadiansPerSecond = rotationalSpeed; 
            }
        }

             outputSpeeds.omegaRadiansPerSecond= -outputSpeeds.omegaRadiansPerSecond; 


        if (alliance == Alliance.Blue) {
          drive.fieldOrientedDrive(outputSpeeds);

        } else {
            drive.fieldOrientedDrive(outputSpeeds, drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)));


        }

       
        if (resetOdometry.getAsBoolean()){
            drive.resetDrivePose(currentPosition);
       }     

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