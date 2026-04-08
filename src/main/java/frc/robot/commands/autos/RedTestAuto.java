package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.commands.PIDsAndThings.PIDToPositionSpline;

public class RedTestAuto extends SequentialCommandGroup {


    public RedTestAuto(
        SwerveDrivetrain drive, 
        FeederSubsystem feeder, 
        HopperSubsystem hopper, 
        IntakeSubsystem intake,
        ShooterController shooter, 
        PivotSubsystem pivot){

        addRequirements(drive, feeder, hopper, intake, shooter, pivot);

        addCommands(
            new InstantCommand(() -> drive.resetPose(drive.getPose())),
            new InstantCommand(() -> drive.resetDrivePose(drive.getPose())),
              new PIDToPositionSpline(
                        drive,
                        new ArrayList<Pose2d>( List.of(
                        new Pose2d(13.689, 1.623, Rotation2d.fromDegrees(90)), 
                        new Pose2d(14.151, 4.137, Rotation2d.fromDegrees(90)),
                        new Pose2d(15.294,6.995, Rotation2d.fromDegrees(90)))),
                        new ArrayList<Double>(List.of(0.4,0.4,0.1)),
                        new ArrayList<Boolean>(List.of(false,false, true)))
        );
        
  }
}                       

