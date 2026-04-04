// package frc.robot.commands.autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.PIDsAndThings.PIDToPositionClamped;
// import frc.robot.commands.PIDsAndThings.PIDToPositionSpline;
// import frc.robot.subsystems.FeederSubsystem;
// import frc.robot.subsystems.HopperSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterController;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;

// public class BlueTwoCycleRight extends SequentialCommandGroup {

//     public BlueTwoCycleRight(
//     SwerveDrivetrain drive, 
//         FeederSubsystem feeder, 
//         HopperSubsystem hopper, 
//         IntakeSubsystem intake,
//         ShooterController shooter){

//             addRequirements(drive, feeder, hopper, intake, shooter);

//             Pose2d startPosition = new Pose2d(4.994, 0.472, Rotation2d.fromDegrees(90));
//             Pose2d shootPosition = new Pose2d(6.124, 1.683, Rotation2d.fromDegrees(120));
//             Pose2d inTrenchPosition = new Pose2d(4.631,0.493, Rotation2d.fromDegrees(90));
//             Pose2d pickUpPosition = new Pose2d(7.953, 3.810, Rotation2d.fromDegrees(90));

//             addCommands(
//                 new InstantCommand(() -> drive.resetDrivePose(startPosition())), //TODO look at this
//                 Commands.parallel(
//                     new PIDToPositionSpline(
//                         drive,
//                         Pose2d(5.865, 0.648, Rotation2d.fromDegrees(90)), 
//                         Pose2d(7.727, 1.239, Rotation2d.fromDegrees(90)))
//                 )
                
//             );
            
            
//         }

    
// } 
