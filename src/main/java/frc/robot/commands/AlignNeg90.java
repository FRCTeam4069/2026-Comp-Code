package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AlignNeg90 extends Command {
    private final SwerveDrivetrain drive;
   
    private Pose2d withVision;
    private Pose2d encoderOnly;
    private Pose2d odometryError;

    private PIDController lowerHeadingController = new PIDController(
        DrivetrainConstants.lowerHeadingCoefficients.kP(), 
        DrivetrainConstants.lowerHeadingCoefficients.kI(), 
        DrivetrainConstants.lowerHeadingCoefficients.kD());

    private Pose2d currentPosition;
    private double desiredHeading = -90.0;
    private Alliance alliance = Alliance.Blue;

    private double tolerance = 0.5;
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

        public AlignNeg90(
            SwerveDrivetrain drive
            ) {
    
            this.drive = drive;    
    
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            lowerHeadingController.enableContinuousInput(-Math.PI, Math.PI);
            currentPosition= drive.getPose();
    
            
    
        }
    
        @Override
        public void execute() {
    
            currentPosition= drive.getPose();

            var outputSpeeds = new ChassisSpeeds(0,0,0);
            //Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3) * DrivetrainConstants.maxAngularVelocity);
        
                rotationalSpeed = lowerHeadingController.calculate(drive.getRotation2d().getRadians(), Math.toRadians(desiredHeading));

                if (Math.abs(lowerHeadingController.getError()) >= Math.toRadians(tolerance)){
                    outputSpeeds.omegaRadiansPerSecond = rotationalSpeed; 
                    }
        
                rotationalSpeed = lowerHeadingController.calculate(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians(), desiredHeading);
            

             outputSpeeds.omegaRadiansPerSecond= -outputSpeeds.omegaRadiansPerSecond; 

             driveSpeeds = outputSpeeds;
            
            drive.fieldOrientedDrive(driveSpeeds);

    }

    
    
    @Override
    public boolean isFinished() {


        if(Math.abs(lowerHeadingController.getError()) < Math.toRadians(tolerance)) {
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
