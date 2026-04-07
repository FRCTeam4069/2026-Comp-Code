package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AlignNeg90 extends Command {
    private final SwerveDrivetrain drive;


    private PIDController middleHeadingController = new PIDController(
            DrivetrainConstants.middleHeadingCoefficients.kP(),
            DrivetrainConstants.middleHeadingCoefficients.kI(),
            DrivetrainConstants.middleHeadingCoefficients.kD());

    private Pose2d currentPosition;
    private double desiredHeading = -90.0;

    private double tolerance = 0.5;
    double rotationalSpeed = 0.0;

    private ChassisSpeeds driveSpeeds;

    /**
     * Teleop drive command
     * 
     * @param drive swerve drivetrain
     */

    public AlignNeg90(
            SwerveDrivetrain drive) {

        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        middleHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        currentPosition = drive.getPose();

    }

    @Override
    public void execute() {

        currentPosition = drive.getPose();

        var outputSpeeds = new ChassisSpeeds(0, 0, 0);
        // Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband),
        // 3) * DrivetrainConstants.maxAngularVelocity);

        rotationalSpeed = middleHeadingController.calculate(drive.getRotation2d().getRadians(),
                Math.toRadians(desiredHeading));

        if (Math.abs(middleHeadingController.getError()) >= Math.toRadians(tolerance)) {
            outputSpeeds.omegaRadiansPerSecond = rotationalSpeed;
        }

        rotationalSpeed = middleHeadingController
                .calculate(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians(), desiredHeading);

        outputSpeeds.omegaRadiansPerSecond = -outputSpeeds.omegaRadiansPerSecond;

        driveSpeeds = outputSpeeds;

        drive.fieldOrientedDrive(driveSpeeds);

    }

    @Override
    public boolean isFinished() {

        if (Math.abs(middleHeadingController.getError()) < Math.toRadians(tolerance)) {
            drive.stop();
            return true;
        }

        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

}
