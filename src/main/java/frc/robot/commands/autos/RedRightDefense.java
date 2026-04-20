package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignAutoCommand;
import frc.robot.commands.AutoAlignInfinite;
import frc.robot.commands.ShootWithTimeout;
import frc.robot.commands.PIDsAndThings.PIDToPositionSpline;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RedRightDefense extends SequentialCommandGroup {

    AutoAlignInfinite alignInfinite1;
    AutoAlignInfinite alignInfinite2;

    AutoAlignAutoCommand autoAlign1;
    AutoAlignAutoCommand autoAlign2;



    ShootWithTimeout shoot1;
    ShootWithTimeout shoot2;

    public RedRightDefense(
            SwerveDrivetrain drive,
            FeederSubsystem feeder,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            ShooterController shooter,
            PivotSubsystem pivot) {

        addRequirements(drive, feeder, hopper, intake, shooter, pivot);

        alignInfinite1 = new AutoAlignInfinite(drive);
        alignInfinite2 = new AutoAlignInfinite(drive);

        shoot1 = new ShootWithTimeout(shooter, feeder, hopper, pivot);
        shoot2 = new ShootWithTimeout(shooter, feeder, hopper, pivot);

        autoAlign1 = new AutoAlignAutoCommand(drive);
        autoAlign2 = new AutoAlignAutoCommand(drive);


        Pose2d startPosition = new Pose2d(12.993, 0.474, Rotation2d.fromDegrees(90));
        Pose2d defensePosition = new Pose2d(8.387, 4.203, Rotation2d.fromDegrees(0)); //4.303

        addCommands(
                new InstantCommand(() -> drive.resetPose(startPosition)),
                new InstantCommand(() -> drive.resetDrivePose(startPosition)),
                Commands.waitSeconds(4),
                Commands.parallel( 
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(8.731, 1.348, Rotation2d.fromDegrees(90)),
                                        new Pose2d(8.931, 1.55, Rotation2d.fromDegrees(90)),
                                        new Pose2d(8.887, 3.767, Rotation2d.fromDegrees(90)),
                                        defensePosition)),
                                new ArrayList<Double>(List.of(0.2, 0.4, 0.4,0.5)),
                                new ArrayList<Boolean>(List.of(false, false, false, true))),
                        intake.intakeOn(),
                        Commands.sequence(
                                Commands.waitSeconds(1.5),
                                pivot.intakeDown())
                                ),
                        new InstantCommand(() -> drive.resetPose(drive.getPose())),
                        new InstantCommand(() -> drive.resetDrivePose(drive.getPose())),
                        intake.intakeOff()
                
        );

    }

}