package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ShooterConstants;


public class ShooterController extends SubsystemBase {
    SparkFlex shooterOneMotorOne, shooterOneMotorTwo, shooterTwoMotorOne, shooterTwoMotorTwo;

    public double targetRPMOne = 0.0;
    public double currentRPMOne = 0.0;

    public  double targetRPMTwo = 0.0;
    public double currentRPMTwo = 0.0;

    private final double a = 0.0; //FIXME
    private  double distance = 0.0; //FIXME
    private final double b = 0.0; //FIXME

    double pidOutOne = 0.0;
    double ffOutOne = 0.0;
    double voltsOne = 0.0;

    double pidOutTwo = 0.0;
    double ffOutTwo = 0.0;
    double voltsTwo = 0.0;

    private Pose2d currentRobotPose;

    PIDController shooterPID = new PIDController( 
        ShooterConstants.shooterCoefficients.kP(),
        ShooterConstants.shooterCoefficients.kI(), 
        ShooterConstants.shooterCoefficients.kD());

    SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(ShooterConstants.shooterCoefficients.kV(),ShooterConstants.shooterCoefficients.kA()); 


    public ShooterController(){

        shooterOneMotorOne = new SparkFlex(DeviceIDs.SHOOTER_ONE_MOTOR_ONE, MotorType.kBrushless);
        shooterOneMotorTwo = new SparkFlex(DeviceIDs.SHOOTER_ONE_MOTOR_TWO, MotorType.kBrushless);
        shooterTwoMotorOne = new SparkFlex(DeviceIDs.SHOOTER_TWO_MOTOR_ONE, MotorType.kBrushless);
        shooterTwoMotorTwo = new SparkFlex(DeviceIDs.SHOOTER_TWO_MOTOR_TWO, MotorType.kBrushless);


        shooterOneMotorOne.configure(ShooterConstants.shooterOneMotorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterOneMotorTwo.configure(ShooterConstants.shooterOneMotorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTwoMotorOne.configure(ShooterConstants.shooterTwoMotorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTwoMotorTwo.configure(ShooterConstants.shooterTwoMotorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        currentRobotPose = new Pose2d();
    }


    public void stop(){
        shooterOneMotorOne.set(0);
        shooterTwoMotorOne.set(0);
    }

    public void shoot(double distance){

        targetRPMOne = (a * Math.pow(b, distance)); //FIXME make less bad, regression, maybe quadratic
        currentRPMOne = (shooterOneMotorOne.getEncoder().getVelocity());

        pidOutOne = shooterPID.calculate(currentRPMOne, targetRPMOne);
        ffOutOne = shooterFF.calculate(targetRPMOne);
        voltsOne = MathUtil.clamp (pidOutOne + ffOutOne, -12.0, 12.0); 



        targetRPMTwo = (a * Math.pow(b, distance)); //FIXME make less bad, regression, maybe quadratic
        currentRPMTwo = (shooterTwoMotorOne.getEncoder().getVelocity());

        pidOutTwo = shooterPID.calculate(currentRPMTwo, targetRPMTwo);
        ffOutTwo = shooterFF.calculate(targetRPMTwo);
        voltsTwo = MathUtil.clamp (pidOutTwo + ffOutTwo, -12.0, 12.0); 


        shooterOneMotorOne.setVoltage(voltsOne);
        shooterTwoMotorOne.setVoltage(voltsTwo);
    }

    public void setCurrentRobotPose(Pose2d updatedRobotPose){
        currentRobotPose = updatedRobotPose;
    }

    public Pose2d getCurrentRobotPose(){
        return currentRobotPose;
    }
}