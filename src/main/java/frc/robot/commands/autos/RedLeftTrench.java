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

public class RedLeftTrench extends SequentialCommandGroup {

    AutoAlignInfinite alignInfinite1;
    AutoAlignInfinite alignInfinite2;

    AutoAlignAutoCommand autoAlign1;
    AutoAlignAutoCommand autoAlign2;



    ShootWithTimeout shoot1;
    ShootWithTimeout shoot2;

    public RedLeftTrench(
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


        Pose2d startPosition = new Pose2d(11.546, 0.472, Rotation2d.fromDegrees(90));
        Pose2d pickUpPosition = new Pose2d(8.887, 3.765, Rotation2d.fromDegrees(90));

        addCommands(
                new InstantCommand(() -> drive.resetPose(startPosition)),
                new InstantCommand(() -> drive.resetDrivePose(startPosition)),
                Commands.parallel(
                        new PIDToPositionSpline( //spiral setup
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(8.731, 1.046, Rotation2d.fromDegrees(90)),
                                        pickUpPosition)),
                                new ArrayList<Double>(List.of(0.4, 0.4)),
                                new ArrayList<Boolean>(List.of(false, true))),
                        intake.intakeOn(),
                        Commands.sequence(
                                Commands.waitSeconds(0.75),
                                pivot.intakeDown())),
                Commands.parallel( 
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        //new Pose2d(10.541, 0.583, Rotation2d.fromDegrees(90)),
                                        new Pose2d(9.952,0.605, Rotation2d.fromDegrees(90)),
                                        new Pose2d(13.281, 0.583,Rotation2d.fromDegrees(90)),
                                        new Pose2d(13.581,0.583,Rotation2d.fromDegrees(115)))),
                                new ArrayList<Double>(List.of(0.1, 0.4, 0.4)),
                                new ArrayList<Boolean>(List.of(true, false, true)))),
                autoAlign1,
                Commands.deadline(
                        Commands.waitSeconds(5),
                        intake.intakeOff(),
                        alignInfinite1,
                        shoot1 //TODO check if timeout actually works, should??? 
                ),

                // second cycle
                 Commands.parallel(
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(10.669, 0.583,Rotation2d.fromDegrees(90)),
                                        new Pose2d(10.882,3.606,Rotation2d.fromDegrees(90)))),
                                new ArrayList<Double>(List.of(0.4, 0.4, 0.1)),
                                new ArrayList<Boolean>(List.of(false, true))),
                        Commands.sequence(
                                Commands.waitSeconds(0.5),
                                pivot.intakeDown(),
                                intake.intakeOn())),
                Commands.parallel(
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(9.952,3.606,Rotation2d.fromDegrees(90)),
                                        new Pose2d(9.652,3.066,Rotation2d.fromDegrees(-90)),                            
                                        new Pose2d(9.652,0.598,Rotation2d.fromDegrees(-90)),
                                        new Pose2d(9.652,1.0, Rotation2d.fromDegrees(0)),
                                        new Pose2d(12.885, 0.585, Rotation2d.fromDegrees(0)),
                                        new Pose2d(13.581, 0.583,Rotation2d.fromDegrees(115)))),
                                new ArrayList<Double>(List.of(0.4, 0.2,0.4,0.4,0.1,0.1)),
                                new ArrayList<Boolean>(List.of(false, false, false, false, true, true)))),
                autoAlign2,
                Commands.deadline(
                        Commands.waitSeconds(5),
                        intake.intakeOff(),
                        alignInfinite2,
                        shoot2 //TODO check if timeout actually works, should??? 
                )
        );

    }

}
