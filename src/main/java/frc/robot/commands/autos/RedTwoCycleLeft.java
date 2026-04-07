package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootWithTimeout;
import frc.robot.commands.PIDsAndThings.PIDToPositionSpline;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RedTwoCycleLeft extends SequentialCommandGroup {

    private ArrayList<Pose2d> points = new ArrayList<Pose2d>();

    public RedTwoCycleLeft(
            SwerveDrivetrain drive,
            FeederSubsystem feeder,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            ShooterController shooter,
            PivotSubsystem pivot) {

        addRequirements(drive, feeder, hopper, intake, shooter, pivot);

        Pose2d startPosition = new Pose2d(11.546, 0.472, Rotation2d.fromDegrees(90));
        Pose2d shootPosition = new Pose2d(10.416, 1.683, Rotation2d.fromDegrees(120));
        Pose2d inTrenchPosition = new Pose2d(11.909, 0.493, Rotation2d.fromDegrees(90));
        Pose2d pickUpPosition = new Pose2d(8.887, 3.765, Rotation2d.fromDegrees(90));

        // drive.resetDrivePose(startPosition);
        // drive.resetPose(startPosition);

        addCommands(
                new InstantCommand(() -> drive.resetPose(startPosition)),
                new InstantCommand(() -> drive.resetDrivePose(startPosition)),
                Commands.parallel(
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(10.675, 0.9, Rotation2d.fromDegrees(90)),
                                        new Pose2d(9.044, 1.253, Rotation2d.fromDegrees(90)),
                                        pickUpPosition)),
                                new ArrayList<Double>(List.of(0.4, 0.4, 0.1)),
                                new ArrayList<Boolean>(List.of(false, false, true))),
                        Commands.sequence(
                                Commands.waitSeconds(0.75),
                                pivot.intakeDown(),
                                intake.intakeOn())),
                intake.intakeOff(),
                Commands.parallel(
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(10.671, 0.632, Rotation2d.fromDegrees(90)),
                                        new Pose2d(13.799, 0.632, Rotation2d.fromDegrees(110)))),
                                new ArrayList<Double>(List.of(0.15, 0.1)),
                                new ArrayList<Boolean>(List.of(true, true)))),
                Commands.sequence(
                    
                )
                
 
        );

    }

}
