package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShooterConstants;


public class ShooterController extends SubsystemBase {
    SparkFlex shooterOneMotorOne, shooterOneMotorTwo, shooterTwoMotorOne, shooterTwoMotorTwo;
    SparkMax hoodArticulate;

    private double targetDeg = 0.0;
    private double hoodPos = 0.0;
    private double pidOutHood = 0.0;
    private static final double hoodTolerace = 0.5;
    private static final double maxDistance = 4.36483;
    private static final double minHoodDistance = 2.95;

    PIDController hoodController = new PIDController(
    HoodConstants.hoodCoefficients.kP(),
    HoodConstants.hoodCoefficients.kI(),
    HoodConstants.hoodCoefficients.kD());

    public double targetRPMOne = 0.0;
    public double currentRPMOne = 0.0;
    public double autoRPM = 4000.0; 

    public  double targetRPMTwo = 0.0;
    public double currentRPMTwo = 0.0;

    public double MetersPerSecondOne = 0.0;
    public double MetersPerSecondTwo = 0.0;

    public double passRPM = 3500.0; //FIXME

    double pidOutOne = 0.0;
    double ffOutOne = 0.0;
    double voltsOne = 0.0;

    double pidOutTwo = 0.0;
    double ffOutTwo = 0.0;
    double voltsTwo = 0.0;

    private boolean inPosition = false;

    private Pose2d currentRobotPose;

    PIDController shooterPIDOne = new PIDController( 
        ShooterConstants.shooterCoefficientsOne.kP(),
        ShooterConstants.shooterCoefficientsOne.kI(), 
        ShooterConstants.shooterCoefficientsOne.kD());

     PIDController shooterPIDTwo = new PIDController( 
        ShooterConstants.shooterCoefficientsTwo.kP(),
        ShooterConstants.shooterCoefficientsTwo.kI(), 
        ShooterConstants.shooterCoefficientsTwo.kD());

    SimpleMotorFeedforward shooterFFOne = new SimpleMotorFeedforward(0, ShooterConstants.shooterCoefficientsOne.kV(),ShooterConstants.shooterCoefficientsOne.kA()); 

    SimpleMotorFeedforward shooterFFTwo = new SimpleMotorFeedforward(0, ShooterConstants.shooterCoefficientsTwo.kV(),ShooterConstants.shooterCoefficientsTwo.kA()); 



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

        hoodArticulate = new SparkMax(DeviceIDs.HOOD, MotorType.kBrushless);

        hoodArticulate.configure(HoodConstants.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodArticulate.getEncoder().setPosition(HoodConstants.AWAY);
    }


    public void stop(){

        targetRPMOne = 0;
        targetRPMTwo = 0;

    }

    public void autoShoot(){
        targetRPMOne = 3500;
        targetRPMTwo = 3500;
        targetDeg = 4;
    }

    // public void runShooter(){
    //     // shooterOneMotorOne.setVoltage(7);        
    //     shooterOneMotorTwo.setVoltage(7);

    //     // shooterTwoMotorOne.setVoltage(7);
    //     shooterTwoMotorTwo.setVoltage(7);


    // }

    public void shoot(double distance){

        if(distance > maxDistance){
            targetRPMOne = 0;
            targetRPMTwo = 0;
        }

        else{
        targetRPMOne = (552.083971 * Math.pow(distance, 5)) - (7515.328409 * Math.pow(distance, 4)) + (39805.52653 * Math.pow(distance, 3))
         - (102322.966701 * Math.pow(distance, 2)) + (127617.991848 * distance) - 58567.737702;

         targetRPMTwo = (552.083971 * Math.pow(distance, 5)) - (7515.328409 * Math.pow(distance, 4)) + (39805.52653 * Math.pow(distance, 3))
         - (102322.966701 * Math.pow(distance, 2)) + (127617.991848 * distance) - 58567.737702;

        }

        if(distance > maxDistance){
            targetDeg = 0;
        }

        else if (distance < minHoodDistance){
            targetDeg = 0;
        }

        else{
            targetDeg = (21.44172 * Math.pow(distance, 3)) - (227.32753 * Math.pow(distance, 2)) + (802.95969 * distance) - 940.86642;
        }
      
    }

    public void hoodAway(){
        targetDeg = HoodConstants.AWAY;
    }

    // public void stopHood(){
    //     hoodArticulate.set(0);
    // }

    // public void testHood( double hoodPower){
    //     double power = MathUtil.clamp(hoodPower, -0.7, 0.7);
    //     hoodArticulate.set(power);
    // }

    //  public void passPosition(){
    //     targetDeg = HoodConstants.PASS;
    // }

    public void setCurrentRobotPose(Pose2d updatedRobotPose){
        currentRobotPose = updatedRobotPose;
    }

    public Pose2d getCurrentRobotPose(){
        return currentRobotPose;
    }

    public void pass(){

        targetRPMOne = passRPM;
        targetRPMTwo = passRPM;
        targetDeg = HoodConstants.PASS;
    }

     public double getHoodPos(){
        hoodPos = hoodArticulate.getEncoder().getPosition();
         return hoodPos;

     }

     public boolean hoodInPosition(){
        hoodPos = getHoodPos();

        if (Math.abs(targetDeg - hoodPos) < 2){
            inPosition = true;
        }

        else{
            inPosition = false;
        }

        return inPosition;

     }

    public Command stopCommand(){
        return run (()->stop());
    }

    public void autoRamp(){
        targetRPMOne = autoRPM;
        targetRPMTwo = autoRPM;
    }

    public Command autoRampCommand(){
        return runOnce(()->autoRamp());
    }

    public void periodic(){

        // MetersPerSecondOne = (targetRPMOne / 60) * 0.31918581;
        // MetersPerSecondTwo = (targetRPMTwo / 60) * 0.31918581;

        
        currentRPMOne = (shooterOneMotorOne.getEncoder().getVelocity());

        // pidOutOne = shooterPIDOne.calculate(currentRPMOne, targetRPMOne);//targetRPMOne
        // ffOutOne = shooterFFOne.calculate(MetersPerSecondOne);
        // voltsOne = MathUtil.clamp (0 + ffOutOne, 0.0, 12.0); 

        currentRPMTwo = -(shooterTwoMotorOne.getEncoder().getVelocity());

        // pidOutTwo = shooterPIDTwo.calculate(currentRPMTwo, targetRPMTwo );
        // ffOutTwo = shooterFFTwo.calculate(MetersPerSecondTwo);
        // voltsTwo = MathUtil.clamp (0 + ffOutTwo, 0.0, 12.0); 


        if(currentRPMOne < targetRPMOne){
            voltsOne = 10.5;
        }
        else{
            voltsOne = 0;
        }


        if(currentRPMTwo < targetRPMTwo){
            voltsTwo = 10.5;
        }
        else{
            voltsTwo = 0;
        }

        if(targetRPMOne <= 200){

            shooterOneMotorOne.setVoltage(0);
            shooterOneMotorTwo.setVoltage(0);
        }

         else{

            shooterOneMotorOne.setVoltage(voltsOne);
            shooterOneMotorTwo.setVoltage(voltsOne);
         }

         if (targetRPMTwo <= 200){

            shooterTwoMotorOne.setVoltage(0);
            shooterTwoMotorTwo.setVoltage(0);

        }

        else{

            shooterTwoMotorOne.setVoltage(voltsTwo);
            shooterTwoMotorTwo.setVoltage(voltsTwo);
        }

    

       
        hoodPos = getHoodPos();

        targetDeg = MathUtil.clamp(targetDeg, HoodConstants.lowerLimit, HoodConstants.upperLimit);

        pidOutHood = hoodController.calculate (hoodPos, targetDeg);
        pidOutHood = MathUtil.clamp(pidOutHood, -12, 12);
        

       if (Math.abs(hoodPos - targetDeg) < hoodTolerace){
            hoodArticulate.setVoltage(0.0);

        }

        else{
            hoodArticulate.setVoltage(pidOutHood);
        }




        SmartDashboard.putNumber("target RPM 1",targetRPMOne);
        SmartDashboard.putNumber("target RPM 2",targetRPMTwo);

        SmartDashboard.putNumber("RPM One", currentRPMOne);
        SmartDashboard.putNumber("RPM Two", currentRPMTwo);

        SmartDashboard.putNumber("volts One", voltsOne);
        SmartDashboard.putNumber("volts two", voltsTwo);

        SmartDashboard.putNumber("current one one",shooterOneMotorOne.getOutputCurrent());
        SmartDashboard.putNumber("current two one", shooterTwoMotorOne.getOutputCurrent());
        SmartDashboard.putNumber("current one two",shooterOneMotorTwo.getOutputCurrent());
         SmartDashboard.putNumber("current two two",shooterTwoMotorTwo.getOutputCurrent());

        SmartDashboard.putNumber("Hood Position", hoodArticulate.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood Target", targetDeg);

        //drive better and shoot better and in the net, do what driver says 


    }
}