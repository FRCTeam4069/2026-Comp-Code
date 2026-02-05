
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;

import static frc.robot.constants.DrivetrainConstants.getShooterPose;

import java.util.function.BooleanSupplier;

import frc.robot.constants.DrivetrainConstants.ShooterPoses;
import frc.robot.constants.DrivetrainConstants.HumanPlayerStations;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class ThroughTrench extends Command {

    private final SwerveDrivetrain drive;
    private final double YBottomTrench =0.62;
    private final double YTopTrench =7.44;
    private Pose2d setPoint;
    private Pose2d trenchAlign;
    private double trenchY=0.0;
    private double deltaYTrench = 0.0;

    private final BooleanSupplier throughTrench;



    private final DrivetrainPIDController controller;   

    Pose2d redLeftShoot = getShooterPose( HumanPlayerStations.Red, ShooterPoses.RedLeft);//FIXME in constants
    Pose2d redRightShoot = getShooterPose( HumanPlayerStations.Red, ShooterPoses.RedRight); //FIXME in constants
    Pose2d blueLeftShoot = getShooterPose( HumanPlayerStations.Blue, ShooterPoses.BlueLeft);
    Pose2d blueRightShoot = getShooterPose( HumanPlayerStations.Blue, ShooterPoses.BlueRight);


    Pose2d redLeftPickup = new Pose2d(6.20, 4.02, Rotation2d.fromDegrees(180.0)); //FIXME
    Pose2d redRightPickup = new Pose2d(5.91, 4.02, Rotation2d.fromDegrees(180.0));//FIXME
    Pose2d blueLeftPickup = new Pose2d(6.2, 7.402, Rotation2d.fromDegrees(-52.5));
    Pose2d blueRightPickup = new Pose2d(6.2, 0.668, Rotation2d.fromDegrees(52.5));

    private Pose2d currentPosition;
    private Alliance alliance = Alliance.Blue;
    private double X = 0.0;
    private double Y = 0.0;


  public ThroughTrench(
    SwerveDrivetrain drive,
    BooleanSupplier throughTrench
    ){

        this.drive = drive;
        this.throughTrench = throughTrench;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);
    addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        
        var result = DriverStation.getAlliance();
        if (result.isPresent()) {
            alliance = result.get();
        }

        currentPosition= drive.getPose();
         X = currentPosition.getX();
         Y = currentPosition.getY();

        if(alliance==Alliance.Blue){ 

            if (X > 2 && X < 4 && Y > 6.75 && Y < 8){ // Blue right shooter to pickup
                trenchY= YTopTrench;
                setPoint = blueRightPickup;
                return;

        }

          else if(X > 5.25 && X < 7.25 && Y > 6.75 && Y < 8){ // Blue right pickup to shooter
                trenchY= YTopTrench;
                setPoint = blueRightShoot; 
                return;
  
            }

            else if(X > 2 && X < 4 && Y > 0 && Y < 1.3){ //Blue left shooter to pickup
                trenchY= YBottomTrench;
                setPoint= blueLeftPickup;
                return;

            }

            else if(X > 5.25 && X < 7.25 && Y > 0 && Y < 1.3){ // blue left pickup to shooter
                trenchY= YBottomTrench;
                setPoint = blueLeftShoot;
                return;
            }

        }


        //FIXME add points for red cause right now are for blue
        else{

            if (X > 12.54 && X < 14.54 && Y > 6.75 && Y < 8){ // red left shooter to pickup
                trenchY= YTopTrench;
                setPoint = redLeftPickup;
                return;
            }

            else if(X > 9.29 && X < 11.29 && Y > 6.75 && Y < 8){ // red left pickup to shooter
                trenchY= YTopTrench;
                setPoint = redLeftShoot;
                return;

            }

            else if(X > 12.54 && X < 14.54 && Y > 0 && Y < 1.3){ //red right shooter to pickup
                trenchY= YBottomTrench;
                setPoint = redRightPickup;
                return;

            }

            else if(X > 9.29 && X < 11.29 && Y > 0 && Y < 1.3){ // red right pickup to shooter 
                trenchY= YBottomTrench;
                setPoint = redRightShoot;
                return;

            }
        }

        currentPosition = drive.getPose();
        trenchAlign = new Pose2d(currentPosition.getX(), trenchY, currentPosition.getRotation());
    }
   
    @Override
    public void execute() {

        if (!throughTrench.getAsBoolean()) {
            return;
        }

        deltaYTrench = currentPosition.getY() - trenchY;
        boolean yAlign = Math.abs(deltaYTrench) < 0.1;

        controller.reset(
            drive.getPose(),
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRotation2d())
        );

        if (!yAlign) {

            ChassisSpeeds speeds = controller.calculate(drive.getPose(), trenchAlign);
            drive.drive(speeds);
         } 

        ChassisSpeeds speeds = controller.calculate(drive.getPose(), setPoint);
        drive.drive(speeds);    }
}