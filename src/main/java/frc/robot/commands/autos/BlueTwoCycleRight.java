// package frc.robot.commands.autos;

// import java.util.ArrayList;
// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.PIDsAndThings.PIDToPositionClamped;
// import frc.robot.commands.PIDsAndThings.PIDToPositionSpline;
// import frc.robot.constants.DrivetrainConstants;
// import frc.robot.subsystems.FeederSubsystem;
// import frc.robot.subsystems.HopperSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.ShooterController;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;

// public class BlueTwoCycleRight extends SequentialCommandGroup {

//     private ArrayList<Pose2d> points = new ArrayList<Pose2d>();

//     public BlueTwoCycleRight(
//     SwerveDrivetrain drive, 
//         FeederSubsystem feeder, 
//         HopperSubsystem hopper, 
//         IntakeSubsystem intake,
//         ShooterController shooter, 
//         PivotSubsystem pivot){

//             addRequirements(drive, feeder, hopper, intake, shooter, pivot);

//             Pose2d startPosition = new Pose2d(4.994, 0.472, Rotation2d.fromDegrees(90));
//             Pose2d shootPosition = new Pose2d(6.124, 1.683, Rotation2d.fromDegrees(120));
//             Pose2d inTrenchPosition = new Pose2d(4.631,0.493, Rotation2d.fromDegrees(90));
//             Pose2d pickUpPosition = new Pose2d(7.953, 3.810, Rotation2d.fromDegrees(90));

//             addCommands(
//                 new InstantCommand(() -> drive.resetDrivePose(startPosition)), 
//                 Commands.parallel(
//                     new PIDToPositionSpline(
//                         drive,
//                         new ArrayList<Pose2d>( List.of(
//                         new Pose2d(5.865, 0.648, Rotation2d.fromDegrees(90)), 
//                         new Pose2d(7.727, 1.239, Rotation2d.fromDegrees(90)),
//                         pickUpPosition))),
//                     pivot.intakeDown(),
//                     intake.intakeOn()     
//                 ),
//                 Commands.parallel(
//                     new PIDToPositionSpline(
//                         drive,
//                         new ArrayList<Pose2d>( List.of(
//                         new Pose2d(7.727, 1.239, Rotation2d.fromDegrees(90)),
//                         new Pose2d(5.865, 0.648, Rotation2d.fromDegrees(90)), 
//                         inTrenchPosition))),
//                     intake.intakeOff()
//                 ),
//                 new PIDToPositionSpline(
//                     drive,
//                     new ArrayList<Pose2d>( List.of(
//                     new Pose2d(3.493, 0.569, Rotation2d.fromDegrees(90)),
//                     new Pose2d(2.515, 1.923, Rotation2d.fromDegrees(70)),
//                     shootPosition)))
                    
                
                

//             );
            
            
//         }

    
// } 
