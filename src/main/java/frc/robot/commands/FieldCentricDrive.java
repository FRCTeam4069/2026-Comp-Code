package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(100.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(100.0);
    private PIDController headingController = new PIDController(
        DrivetrainConstants.teleOpHeadingCoefficients.kP(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kI(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kD());
    private Pose2d currentPosition;
    private double desiredHeading = 0.0;
    private Alliance alliance = Alliance.Blue;
    private final double redHubX = 469.1;
    private final double redHubY = 158.85;
    private final double blueHubX = 182.1;
    private final double blueHubY = 158.85;
    private double deltaX = 0.0;
    private double deltaY = 0.0;


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
        BooleanSupplier autoAlign) {

        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.autoAlign = autoAlign;

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
        
        SmartDashboard.putNumber("desiredHeading", desiredHeading);

        if (autoAlign.getAsBoolean()){
            currentPosition= drive.getPose();

            if (alliance == Alliance.Blue){

                deltaX = blueHubX - currentPosition.getX();
                deltaY = blueHubY - currentPosition.getY();
            }

            else{

                deltaX = redHubX - currentPosition.getX();
                deltaY = redHubY - currentPosition.getY();
            }

            desiredHeading = Math.atan2(deltaY, deltaX);
            

            outputSpeeds.omegaRadiansPerSecond = headingController.calculate(drive.getRotation2d().getRadians(), desiredHeading);
        }

        if (alliance == Alliance.Blue) {
            drive.fieldOrientedDrive(outputSpeeds);
        } else {
            drive.fieldOrientedDrive(outputSpeeds, drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)));
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