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

public class RedMiddle extends SequentialCommandGroup {

    AutoAlignInfinite alignInfinite;

    AutoAlignAutoCommand autoAlign;



    ShootWithTimeout shoot;

    public RedMiddle(
            SwerveDrivetrain drive,
            FeederSubsystem feeder,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            ShooterController shooter,
            PivotSubsystem pivot) {

        addRequirements(drive, feeder, hopper, intake, shooter, pivot);

        alignInfinite = new AutoAlignInfinite(drive);

        shoot = new ShootWithTimeout(shooter, feeder, hopper, pivot);

        autoAlign = new AutoAlignAutoCommand(drive);


        Pose2d startPosition = new Pose2d(12.950, 3.996, Rotation2d.fromDegrees(180));
        Pose2d shootPosition = new Pose2d(14.399, 3.996, Rotation2d.fromDegrees(180));

        addCommands(
                new InstantCommand(() -> drive.resetPose(startPosition)),
                new InstantCommand(() -> drive.resetDrivePose(startPosition)),

                    new PIDToPositionSpline(
                        drive,
                        new ArrayList<Pose2d>(List.of(shootPosition)), 
                        new ArrayList<Double>(List.of(0.4)),
                        new ArrayList<Boolean>(List.of(true))),
                autoAlign,
                intake.intakeOn(),
                Commands.deadline(
                    shoot,
                    alignInfinite
                ),
                intake.intakeOff()
        );

    }

}
