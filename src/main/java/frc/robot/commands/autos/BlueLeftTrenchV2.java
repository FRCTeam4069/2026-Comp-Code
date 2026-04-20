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

public class BlueLeftTrenchV2 extends SequentialCommandGroup {

    AutoAlignInfinite alignInfinite1;
    AutoAlignInfinite alignInfinite2;

    AutoAlignAutoCommand autoAlign1;
    AutoAlignAutoCommand autoAlign2;



    ShootWithTimeout shoot1;
    ShootWithTimeout shoot2;

    public BlueLeftTrenchV2(
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


        Pose2d startPosition = new Pose2d(4.444, 7.596, Rotation2d.fromDegrees(-90));
        Pose2d pickUpPosition = new Pose2d(7.653, 4.303, Rotation2d.fromDegrees(-90)); //4.303

        addCommands(
                new InstantCommand(() -> drive.resetPose(startPosition)),
                new InstantCommand(() -> drive.resetDrivePose(startPosition)),
                Commands.parallel( //middle of field, pick up path 
                        new PIDToPositionSpline( //spiral setup
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(7.809, 6.722, Rotation2d.fromDegrees(-90)),
                                        new Pose2d(7.609, 6.52, Rotation2d.fromDegrees(-90)),
                                        pickUpPosition)),
                                new ArrayList<Double>(List.of(0.2, 0.4, 0.5)),
                                new ArrayList<Boolean>(List.of(false, false, false))),
                        intake.intakeOn(),
                        Commands.sequence(
                                Commands.waitSeconds(1.0),
                                pivot.intakeDown())
                                ),
                        new InstantCommand(() -> drive.resetPose(drive.getPose())),
                        new InstantCommand(() -> drive.resetDrivePose(drive.getPose())),
                Commands.race( 
                        new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(6.34, 7.468, Rotation2d.fromDegrees(180)),
                                        new Pose2d(3.54, 7.518, Rotation2d.fromDegrees(180)),
                                        new Pose2d(3.298,7.318,Rotation2d.fromDegrees(-78)))),
                                new ArrayList<Double>(List.of(0.4, 0.3, 0.1)),
                                new ArrayList<Boolean>(List.of(false, false, true))),
                                Commands.waitSeconds(5)),
                // intake.intakeOff(),
                Commands.race(
                        Commands.waitSeconds(0.5),
                        autoAlign1),
                Commands.deadline(
                        Commands.waitSeconds(4.75),
                        shoot1,                        
                        alignInfinite1
                          
                ),

                // second cycle
                 Commands.parallel(
                        new PIDToPositionSpline(
                                drive, //drive out of trench
                                new ArrayList<Pose2d>(List.of(
                                        new Pose2d(6.04, 7.485,Rotation2d.fromDegrees(-90)),
                                        new Pose2d(6.04, 4.762, Rotation2d.fromDegrees(-90)))),
                                new ArrayList<Double>(List.of(0.3, 0.3)),
                                new ArrayList<Boolean>(List.of(false, true))),
                        Commands.sequence(
                                Commands.waitSeconds(1.0),
                                pivot.intakeDown(),
                                intake.intakeOn())
                        ),
                Commands.sequence(
                        // new PIDToPositionSpline(
                        //         drive,
                        //         new ArrayList<Pose2d>(List.of( 
                        //                 new Pose2d(6.06, 7.168, Rotation2d.fromDegrees(-90)),
                        //                 new Pose2d(6.06, 7.418, Rotation2d.fromDegrees(-90)),
                        //                 new Pose2d(5.74, 7.418,Rotation2d.fromDegrees(-90)),
                        //                 new Pose2d(3.54,7.418,Rotation2d.fromDegrees(-77)))),
                        //         new ArrayList<Double>(List.of( 0.3,0.3, 0.3, 0.1)),
                        //         new ArrayList<Boolean>(List.of(false, false,false,true))),
                        //         Commands.waitSeconds(5)),

                         new PIDToPositionSpline(
                                drive,
                                new ArrayList<Pose2d>(List.of( 
                                        new Pose2d(6.06, 7.168, Rotation2d.fromDegrees(-90)),
                                        new Pose2d(6.06, 7.418, Rotation2d.fromDegrees(-90)))),
                                new ArrayList<Double>(List.of( 0.3, 0.1)),
                                new ArrayList<Boolean>(List.of(false,true)))),
                // intake.intakeOff(),
                 Commands.race(
                        Commands.waitSeconds(0.5),
                        autoAlign2
                 ),
                Commands.deadline(
                        Commands.waitSeconds(4.75),
                        alignInfinite2,
                        shoot2  
                ),
                intake.intakeOff()
        );

    }

}
