package frc.robot.commands;

import java.time.LocalDate;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.ThroughTrench;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoAlignAutoCommand extends Command {
    private final SwerveDrivetrain drive;
   

    private Pose2d withVision;
    private Pose2d encoderOnly;
    private Pose2d odometryError;

    private PIDController headingController = new PIDController(
        DrivetrainConstants.teleOpHeadingCoefficients.kP(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kI(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kD());



    private Pose2d currentPosition;
    private double desiredHeading = 0.0;
    private Alliance alliance = Alliance.Blue;
    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);
  
    private double deltaX = 0.0;
    private double deltaY = 0.0;
    private double tolerance = 1;
    double rotationalSpeed = 0.0;
   
    private ChassisSpeeds driveSpeeds;
    
        private DoublePublisher desiredHeadingPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("desiredHeading").publish();
        private DoublePublisher deltaXDoublePublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("deltaX").publish();
        private DoublePublisher deltaYDoublePublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("deltaY").publish();
        private StructPublisher<Pose2d> setPointPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("setPoint", Pose2d.struct).publish();
    
        private StructPublisher<Pose2d> odometryErrorPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("odometryError", Pose2d.struct).publish();
    
    
    
    
    
    
        /**
         * Teleop drive command
         * @param drive swerve drivetrain
         */

        public AutoAlignAutoCommand(
            SwerveDrivetrain drive
            ) {
    
            this.drive = drive;
    
            // this.controller = new ThroughTrench(drive);
    
    
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            headingController.enableContinuousInput(-Math.PI, Math.PI);
            currentPosition= drive.getPose();
    
            var result = DriverStation.getAlliance();
            if (result.isPresent()) {
                alliance = result.get();
            }
    
        }
    
        @Override
        public void execute() {
    
            currentPosition= drive.getPose();

            var outputSpeeds = new ChassisSpeeds(0,0,0);
            //Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3) * DrivetrainConstants.maxAngularVelocity);
            
            if (alliance == Alliance.Blue){
    
                deltaX = blueHubX - currentPosition.getX();
                deltaY = blueHubY - currentPosition.getY();
            }
    
            else{
    
                deltaX = redHubX - currentPosition.getX();
                deltaY = redHubY - currentPosition.getY();
            }
    
            desiredHeading = Math.atan(deltaY / deltaX);
            SmartDashboard.putNumber("desiredHeading", desiredHeading);
            desiredHeadingPublisher.set(Math.toDegrees(-desiredHeading));
           
           
            // deltaXDoublePublisher.set(deltaX);
            //deltaYDoublePublisher.set(deltaY);
    
        
            if(alliance == Alliance.Blue){
                rotationalSpeed = headingController.calculate(drive.getRotation2d().getRadians(), desiredHeading);

                if (Math.abs(headingController.getError()) >= Math.toRadians(tolerance)){
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed; 
                    }
                }
            else {
                rotationalSpeed = headingController.calculate(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians(), desiredHeading);

                
                if (Math.abs(headingController.getError()) >= Math.toRadians(tolerance)){
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed; 
                    }
            }
            

             outputSpeeds.omegaRadiansPerSecond= -outputSpeeds.omegaRadiansPerSecond; 

                driveSpeeds = outputSpeeds;

            if (alliance == Alliance.Blue) {
            
            drive.fieldOrientedDrive(driveSpeeds);

            } 

            else {
                drive.fieldOrientedDrive(driveSpeeds, drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)));

            }

        //test for odometry fix
        // setPointPublisher.set(controller.setPoint);

        withVision = drive.poseEstimator.getEstimatedPosition();
        encoderOnly = drive.swerveOdometry.getPoseMeters();


        odometryError = encoderOnly.relativeTo(withVision);
        odometryErrorPublisher.set(odometryError);

    }

    
    
    @Override
    public boolean isFinished() {


        if(Math.abs(headingController.getError()) < Math.toRadians(tolerance)) {
            drive.stop();
            return true;
        }
        
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {}

}
