package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShooterConstants;


public class ShooterController extends SubsystemBase {

        private DoublePublisher ShooterOneTopRPM = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterOneTopRPM").publish();

        private DoublePublisher ShooterOneBottomRPM = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterOneBottomRPM").publish();

        private DoublePublisher ShooterTwoTopRPM = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterTwoTopRPM").publish();
        
        private DoublePublisher ShooterTwoBottomRPM = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterTwoBottomRPM").publish();

            private DoublePublisher ShooterOneTopVoltage = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterOneTopVoltage").publish();

        private DoublePublisher ShooterOneBottomVoltage = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterOneBottomVoltage").publish();

        private DoublePublisher ShooterTwoTopVoltage = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterTwoTopVoltage").publish();
        
        private DoublePublisher ShooterTwoBottomVoltage = NetworkTableInstance.getDefault()
            .getDoubleTopic("ShooterTwoBottomVoltage").publish();
        
    SparkFlex shooterOneMotorOne, shooterOneMotorTwo, shooterTwoMotorOne, shooterTwoMotorTwo;
    SparkMax hoodArticulate;

    private double targetDeg = 0.0;
    private double hoodPos = 0.0;
    private double pidOutHood = 0.0;
    private static final double hoodTolerace = 0.5;
    private static final double maxDistance = 4.8;
    private static final double minHoodDistance = 2.95;
    // private static final double  minShootDistance = 1.3;

    PIDController hoodController = new PIDController(
    HoodConstants.hoodCoefficients.kP(),
    HoodConstants.hoodCoefficients.kI(),
    HoodConstants.hoodCoefficients.kD());

    public double targetRPM = 0.0;
    public double currentRPM = 0.0;
    public double autoRPM = 4000.0; 

    public double MetersPerSecondOne = 0.0;

    public double closePassRPM = 3200.0; //FIXME
    public double farPassRPM = 4200.0;

    double pidOut = 0.0;
    
    
    double volts = 0.0;

    double RPMDiff = 0.0;

    private boolean inPosition = false;

    private Pose2d currentRobotPose;

    

    SparkClosedLoopController shooterOneTopPID;
    SparkClosedLoopController shooterOneBottomPID;
    
    SparkClosedLoopController shooterTwoTopPID;
    SparkClosedLoopController shooterTwoBottomPID;



    //SimpleMotorFeedforward shooterFFOne = new SimpleMotorFeedforward(0, ShooterConstants.shooterCoefficientsOne.kV(),ShooterConstants.shooterCoefficientsOne.kA()); 


    public ShooterController(){

        shooterOneMotorOne = new SparkFlex(DeviceIDs.SHOOTER_ONE_MOTOR_ONE, MotorType.kBrushless);
        shooterOneMotorTwo = new SparkFlex(DeviceIDs.SHOOTER_ONE_MOTOR_TWO, MotorType.kBrushless);
        shooterTwoMotorOne = new SparkFlex(DeviceIDs.SHOOTER_TWO_MOTOR_ONE, MotorType.kBrushless);
        shooterTwoMotorTwo = new SparkFlex(DeviceIDs.SHOOTER_TWO_MOTOR_TWO, MotorType.kBrushless);

        shooterOneTopPID = shooterOneMotorOne.getClosedLoopController();
        shooterOneBottomPID = shooterOneMotorTwo.getClosedLoopController();
        shooterTwoTopPID = shooterTwoMotorOne.getClosedLoopController();
        shooterTwoBottomPID = shooterTwoMotorTwo.getClosedLoopController();

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

        targetRPM = 0;
    }

    public void towerShoot(){
        targetRPM = 3000;
        targetDeg = 6;

    }

    public void manualCloseShoot(){

        targetRPM = 2800;

        targetDeg = 0;

    }

    public void autoShoot(){
        targetRPM = 3350;
        targetDeg = 0;
    }

    public void autoShootMiddle(){
        targetRPM = 3100;
        targetDeg = 0;
    }

    // public void runShooter(){
    //     // shooterOneMotorOne.setVoltage(7);        
    //     shooterOneMotorTwo.setVoltage(7);

    //     // shooterTwoMotorOne.setVoltage(7);
    //     shooterTwoMotorTwo.setVoltage(7);


    // }

    public void shoot(double distance){

        if(distance > maxDistance){ 
            targetRPM = 0;
        }

        //  else if (distance < minHoodDistance){
        //     targetRPM = 0;
        // }


        else{ 
        targetRPM = (83.112158 * Math.pow(distance, 5)) - (1305.949681 * Math.pow(distance, 4)) + (7929.250003 * Math.pow(distance, 3))
         - ( 23315.101833 * Math.pow(distance, 2)) + (33676.298956 * distance) - 16642.169996;

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

    public void farPass(){

        targetRPM = farPassRPM;
        targetDeg = HoodConstants.PASS;
    }

    public void closePass(){

        targetRPM = closePassRPM;
        targetDeg = HoodConstants.PASS;
    }

     public double getHoodPos(){
        hoodPos = hoodArticulate.getEncoder().getPosition();
         return hoodPos;

     }

     public double getCurrentRPM(){

        currentRPM = shooterOneMotorOne.getEncoder().getVelocity();
        return currentRPM;
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
        targetRPM = autoRPM;
    }

    public Command autoRampCommand(){
        return runOnce(()->autoRamp());
    }

    public void periodic(){

        MetersPerSecondOne = (targetRPM / 60) * 0.31918581;




        if(targetRPM <= 200){

            shooterOneMotorOne.setVoltage(0);
            shooterOneMotorTwo.setVoltage(0);
            shooterTwoMotorOne.setVoltage(0);
            shooterTwoMotorTwo.setVoltage(0);

        }

         else{

            shooterOneTopPID.setSetpoint(targetRPM, ControlType.kVelocity);
            shooterOneBottomPID.setSetpoint(targetRPM, ControlType.kVelocity);
            shooterTwoTopPID.setSetpoint(targetRPM, ControlType.kVelocity);
            shooterTwoBottomPID.setSetpoint(targetRPM, ControlType.kVelocity);

            // shooterOneMotorOne.setVoltage(volts);
            // shooterOneMotorTwo.setVoltage(volts);
            // shooterTwoMotorOne.setVoltage(volts);
            // shooterTwoMotorTwo.setVoltage(volts);

            // shooterOneMotorOne.setVoltage(5);
            // shooterOneMotorTwo.setVoltage(5);
            // shooterTwoMotorOne.setVoltage(5);
            // shooterTwoMotorTwo.setVoltage(5);
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


        ShooterOneTopRPM.set(shooterOneMotorOne.getEncoder().getVelocity());
        ShooterOneBottomRPM.set(shooterOneMotorTwo.getEncoder().getVelocity());
        ShooterTwoTopRPM.set(shooterOneMotorTwo.getEncoder().getVelocity());
        ShooterTwoBottomRPM.set(shooterTwoMotorTwo.getEncoder().getVelocity());

        ShooterOneTopVoltage.set(shooterOneMotorOne.getAppliedOutput());
        ShooterOneBottomVoltage.set(shooterOneMotorTwo.getAppliedOutput());
        ShooterTwoTopVoltage.set(shooterTwoMotorOne.getAppliedOutput());
        ShooterTwoBottomVoltage.set(shooterTwoMotorTwo.getAppliedOutput());




        SmartDashboard.putNumber("target RPM",targetRPM);

        SmartDashboard.putNumber("RPM One", currentRPM);

        SmartDashboard.putNumber("volts One", volts);

        SmartDashboard.putNumber("current one one",shooterOneMotorOne.getOutputCurrent());
        SmartDashboard.putNumber("current two one", shooterTwoMotorOne.getOutputCurrent());
        SmartDashboard.putNumber("current one two",shooterOneMotorTwo.getOutputCurrent());
         SmartDashboard.putNumber("current two two",shooterTwoMotorTwo.getOutputCurrent());

        SmartDashboard.putNumber("RPM one top",shooterOneMotorOne.getEncoder().getVelocity());
        SmartDashboard.putNumber("RPM one Bottom", shooterTwoMotorOne.getEncoder().getVelocity());
        SmartDashboard.putNumber("RPM Two top",shooterOneMotorTwo.getEncoder().getVelocity());
        SmartDashboard.putNumber("RPM two bottom",shooterTwoMotorTwo.getEncoder().getVelocity());

        SmartDashboard.putNumber("Hood Position", hoodArticulate.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood Target", targetDeg);

        //drive better and shoot better and in the net, do what driver says 


    }
}