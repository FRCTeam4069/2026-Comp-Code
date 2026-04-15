// package frc.robot.commands.autos;

// import java.util.ArrayList;
// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoAlignAutoCommand;
// import frc.robot.commands.AutoAlignInfinite;
// import frc.robot.commands.ShootWithTimeout;
// import frc.robot.commands.PIDsAndThings.PIDToPositionSpline;
// import frc.robot.subsystems.FeederSubsystem;
// import frc.robot.subsystems.HopperSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.ShooterController;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;

// public class RedLeftTrenchV3 extends SequentialCommandGroup {

//     AutoAlignInfinite alignInfinite1;
//     AutoAlignInfinite alignInfinite2;

//     AutoAlignAutoCommand autoAlign1;
//     AutoAlignAutoCommand autoAlign2;



//     ShootWithTimeout shoot1;
//     ShootWithTimeout shoot2;

//     public RedLeftTrenchV3(
//             SwerveDrivetrain drive,
//             FeederSubsystem feeder,
//             HopperSubsystem hopper,
//             IntakeSubsystem intake,
//             ShooterController shooter,
//             PivotSubsystem pivot) {

//         addRequirements(drive, feeder, hopper, intake, shooter, pivot);

//         alignInfinite1 = new AutoAlignInfinite(drive);
//         alignInfinite2 = new AutoAlignInfinite(drive);

//         shoot1 = new ShootWithTimeout(shooter, feeder, hopper, pivot);
//         shoot2 = new ShootWithTimeout(shooter, feeder, hopper, pivot);

//         autoAlign1 = new AutoAlignAutoCommand(drive);
//         autoAlign2 = new AutoAlignAutoCommand(drive);


//         Pose2d startPosition = new Pose2d(12.125, 0.475, Rotation2d.fromDegrees(90));
//         Pose2d pickUpPosition = new Pose2d(9.3, 3.75, Rotation2d.fromDegrees(0));

//         addCommands(
//                 new InstantCommand(() -> drive.resetPose(startPosition)),
//                 new InstantCommand(() -> drive.resetDrivePose(startPosition)),
//                 Commands.parallel( //to middle of field, pick up path before intake
//                         new PIDToPositionSpline(
//                                 drive, //drive out of trench
//                                 new ArrayList<Pose2d>(List.of(
//                                         new Pose2d(10.0, 0.6, Rotation2d.fromDegrees(90)),
//                                         new Pose2d(8.8, 1.0, Rotation2d.fromDegrees(90)),
//                                         new Pose2d(8.6, 3.0, Rotation2d.fromDegrees(90)),
//                                         new Pose2d(8.8, 3.6, Rotation2d.fromDegrees(45)),
//                                         pickUpPosition)),
//                                 new ArrayList<Double>(List.of(0.4, 0.4, 0.4, 0.4)),
//                                 new ArrayList<Boolean>(List.of(false, false,  false, true)))
//                         // intake.intakeOn(),
//                         // Commands.sequence(
//                         //         Commands.waitSeconds(0.75),
//                         //         pivot.intakeDown())
//                                 ),
//                 Commands.parallel( 
//                         new PIDToPositionSpline(
//                                 drive,
//                                 new ArrayList<Pose2d>(List.of(
                                        
                                       
//                                         new Pose2d(9.8,3.6, Rotation2d.fromDegrees(-45)),
//                                         new Pose2d(10.0,3.0, Rotation2d.fromDegrees(-90)),
//                                         new Pose2d(10.0,1.0, Rotation2d.fromDegrees(-60)),
//                                         new Pose2d(11.0,0.65, Rotation2d.fromDegrees(0)),
//                                         new Pose2d(13.75, 0.65,Rotation2d.fromDegrees(45)),
//                                         new Pose2d(13.5,1.5,Rotation2d.fromDegrees(125)))),
//                                 new ArrayList<Double>(List.of(0.1, 0.1, 0.4, 0.2, 0.2, 0.2)),
//                                 new ArrayList<Boolean>(List.of(true, false, false, false, false, true)))),
//                 // autoAlign1,
//                 // Commands.deadline(
//                 //         Commands.waitSeconds(5),
//                 //         intake.intakeOff(),
//                 //         alignInfinite1,
//                 //         shoot1, //TODO check if timeout actually works, should??? 
//                 // ),

//                 // second cycle
//                  Commands.parallel(//to middle of field, pick up path before intake
//                         new PIDToPositionSpline(
//                                 drive, //drive out of trench
//                                 new ArrayList<Pose2d>(List.of(
//                                         new Pose2d(13.5, 1.5,Rotation2d.fromDegrees(125)),
//                                         new Pose2d(14.0, 0.75,Rotation2d.fromDegrees(-160)),
//                                         new Pose2d(12.5, 0.65, Rotation2d.fromDegrees(-175)))),
//                                 new ArrayList<Double>(List.of(0.4, 0.4, 0.2)),
//                                 new ArrayList<Boolean>(List.of(false, false, true))
//                         // Commands.sequence(
//                         //         Commands.waitSeconds(0.5),
//                         //         pivot.intakeDown(),
//                         //         intake.intakeOn())
//                         )),
//                 Commands.parallel(
//                         new PIDToPositionSpline(
//                                 drive,
//                                 new ArrayList<Pose2d>(List.of(                           
//                                         new Pose2d(11.0, 0.65, Rotation2d.fromDegrees(160)),
//                                         new Pose2d(10.65,1.5,Rotation2d.fromDegrees(90)),
//                                         new Pose2d(10.65,3.5,Rotation2d.fromDegrees(90)),
//                                         new Pose2d(10.5,4.0,Rotation2d.fromDegrees(135)),
//                                         new Pose2d(10.0,4.2,Rotation2d.fromDegrees(180)),
//                                         new Pose2d(9.5,4.0,Rotation2d.fromDegrees(-135)),
//                                         new Pose2d(9.4,3.0,Rotation2d.fromDegrees(-80)),
//                                         new Pose2d(10.5,0.65,Rotation2d.fromDegrees(-20)),
//                                         new Pose2d(12.0,0.65,Rotation2d.fromDegrees(0)),
//                                         new Pose2d(13.75,0.7,Rotation2d.fromDegrees(45)),
//                                         new Pose2d(13.5,1.5, Rotation2d.fromDegrees(125)))),
//                                 new ArrayList<Double>(List.of(0.1, 0.4, 0.2, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4)),
//                                 new ArrayList<Boolean>(List.of(true, false,false, false, false, false, false, false, false, false, true)))
//                 // autoAlign2,
//                 // Commands.deadline(
//                 //         Commands.waitSeconds(5),
//                 //         intake.intakeOff(),
//                 //         alignInfinite2,
//                 //         shoot2 //TODO check if timeout actually works, should??? 
//         )
//         );

//     }

// }