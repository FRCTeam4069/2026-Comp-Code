package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DriveToShootPosition extends Command {
    public enum ClimbTarget {
        LEFT,
        CENTER,
        RIGHT,
        UP
    }

    private final SwerveDrivetrain drive;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;
    private Alliance alliance;
    private final ClimbTarget target;
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("target pose", Pose2d.struct).publish();
    private StructPublisher<Translation2d> vecPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("translation", Translation2d.struct).publish();

    public DriveToShootPosition(SwerveDrivetrain drive, ClimbTarget target) {
        this.drive = drive;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);
        this.target = target;

        addRequirements(drive);
    }

    public double getDistance(Pose2d currentPose, Pose2d closestPose) {
        var deltaX = currentPose.getX() - closestPose.getX();
        var deltaY = currentPose.getY() - closestPose.getY();
        return Math.hypot(deltaX, deltaY);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        } else {
            alliance = Alliance.Blue;
        }

        ArrayList<Pose2d> climbPoses = new ArrayList<>();

        if (alliance == Alliance.Blue) {
            for (Pose2d pose : DrivetrainConstants.blueClimbPoses) { 
                climbPoses.add(pose);
            }
        } else {
            for (Pose2d pose : DrivetrainConstants.redClimbPoses) { 
                climbPoses.add(pose);
            }
        }
        int index = switch (target) {
            case LEFT -> 0;
            case CENTER -> 1;
            case RIGHT -> 2;
            case UP -> 3;
        };
        setpoint = climbPoses.get(index);
        posePublisher.set(setpoint);

        controller.reset(drive.getPose(),
                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRotation2d()));
        controller.calculate(drive.getPose(), setpoint);
    }

    @Override
    public void execute() {
        drive.fieldOrientedDrive(controller.calculate(drive.getPose(), setpoint));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}