package frc.robot.commands.PIDsAndThings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.commands.DrivetrainPIDController;
import frc.robot.constants.DrivetrainConstants;




public class ThroughTrench extends Command {
    private final SwerveDrivetrain drive;
    private final double YBottomTrench =0.62;
    private final double YTopTrench =7.44;
    public Pose2d setPoint;
    private double trenchY=0.0;

    private final DrivetrainPIDController controller;   

    Pose2d redLeftShoot =new Pose2d(3.69, 3.02, Rotation2d.fromDegrees(52.5)); //red left //TODO
    Pose2d redRightShoot =new Pose2d(3.22, 4.21, Rotation2d.fromDegrees(-52.5)); //red right //TODO

     Pose2d blueLeftShoot =  new Pose2d(2.304, 7.402, Rotation2d.fromDegrees(-52.5)); //blue left
     Pose2d blueRightShoot = new Pose2d(2.304, 0.668, Rotation2d.fromDegrees(52.5)); //blue right


    Pose2d redLeftPickup = new Pose2d(6.20, 4.02, Rotation2d.fromDegrees(180.0)); //FIXME
    Pose2d redRightPickup = new Pose2d(5.91, 4.02, Rotation2d.fromDegrees(180.0));//FIXME
    Pose2d blueLeftPickup = new Pose2d(6.2, 7.402, Rotation2d.fromDegrees(0));
    Pose2d blueRightPickup = new Pose2d(6.2, 0.668, Rotation2d.fromDegrees(0));

    private Pose2d currentPosition;
    private Alliance alliance = Alliance.Blue;
    private double X = 0.0;
    private double Y = 0.0;
    private double rotation = 0.0;


    public ThroughTrench(SwerveDrivetrain drive) {
        this.drive = drive;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);
    }

    public void initialize() {
        var result = DriverStation.getAlliance();
        setPoint = null;

        if (result.isPresent()) {
            alliance = result.get();
        }

        currentPosition= drive.getPose();
         X = currentPosition.getX();
         Y = currentPosition.getY();

         controller.reset(currentPosition, drive.getRobotRelativeSpeeds());

        if(alliance==Alliance.Blue){ 
            rotation =0.0;

            if (X > 2 && X < 4 && Y > 6.75 && Y < 8){ // Blue right shooter to pickup
                trenchY= YTopTrench;
                setPoint = blueLeftPickup;
                return;

        }

          else if(X > 5.25 && X < 7.25 && Y > 6.75 && Y < 8){ // Blue right pickup to shooter
                trenchY= YTopTrench;
                setPoint = blueLeftShoot; 
                return;
  
            }

            else if(X > 2 && X < 4 && Y > 0 && Y < 1.3){ //Blue left shooter to pickup
                trenchY= YBottomTrench;
                setPoint= blueRightPickup;
                return;

            }

            else if(X > 5.25 && X < 7.25 && Y > 0 && Y < 1.3){ // blue left pickup to shooter
                trenchY= YBottomTrench;
                setPoint = blueRightShoot;
                return;
            }   
        }

            else{
            rotation= 180.00;

            if (X > 12.54 && X < 14.54 && Y > 6.75 && Y < 8){ // red left shooter to pickup
                trenchY= YTopTrench;
                setPoint = redRightPickup;
                return;
            }

            else if(X > 9.29 && X < 11.29 && Y > 6.75 && Y < 8){ // red left pickup to shooter
                trenchY= YTopTrench;
                setPoint = redRightShoot;
                return;

            }

            else if(X > 12.54 && X < 14.54 && Y > 0 && Y < 1.3){ //red right shooter to pickup
                trenchY= YBottomTrench;
                setPoint = redLeftPickup;
                return;

            }

            else if(X > 9.29 && X < 11.29 && Y > 0 && Y < 1.3){ // red right pickup to shooter 
                trenchY= YBottomTrench;
                setPoint = redLeftShoot;
                return;
            }

            
        }
    }

    public ChassisSpeeds getSpeeds() {
        if (setPoint == null) {
            return new ChassisSpeeds();
        }

        Pose2d current = drive.getPose();
        double deltaY = current.getY() - trenchY;
        boolean aligned = Math.abs(deltaY) < 0.1;

        Pose2d trenchAlign = new Pose2d(current.getX(), trenchY, Rotation2d.fromDegrees(rotation));
        return aligned
            ? controller.calculate(current, setPoint)
            : controller.calculate(current, trenchAlign);
            
    }
      @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
