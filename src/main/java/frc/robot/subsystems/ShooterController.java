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
    //SparkMax hoodArticulate;

    // private double targetDeg = 0.0;
    // private double hoodPos = 0.0;
    //private double pidOutHood = 0.0;
    //private static final double farShootPos = 0.0;

    //private final AnalogEncoder hoodEncoder = new AnalogEncoder(HoodConstants.LAMPREY_PORT);// FIXME FOR PORT


    // PIDController hoodController = new PIDController(
    // HoodConstants.hoodCoefficients.kP(),
    // HoodConstants.hoodCoefficients.kI(),
    // HoodConstants.hoodCoefficients.kD());

    public double targetRPMOne = 0.0;
    public double currentRPMOne = 0.0;
    public double autoRPM = 4000.0; 

    public  double targetRPMTwo = 0.0;
    public double currentRPMTwo = 0.0;

    public double MetersPerSecondOne = 0.0;
    public double MetersPerSecondTwo = 0.0;

    public double passRPM = 3000.0; //FIXME

    private final double a = 0.0; //FIXME
    private final double b = 0.0; //FIXME

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

    SimpleMotorFeedforward shooterFFOne = new SimpleMotorFeedforward(ShooterConstants.shooterCoefficientsOne.kV(),ShooterConstants.shooterCoefficientsOne.kA()); 

    SimpleMotorFeedforward shooterFFTwo = new SimpleMotorFeedforward(ShooterConstants.shooterCoefficientsTwo.kV(),ShooterConstants.shooterCoefficientsTwo.kA()); 



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

        // hoodArticulate = new SparkMax(DeviceIDs.HOOD, MotorType.kBrushless);

        // hoodArticulate.configure(HoodConstants.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void stop(){
        // shooterOneMotorOne.stopMotor();
        // shooterTwoMotorOne.stopMotor();

        targetRPMOne = 0;
        targetRPMTwo = 0;
        //shooterOneMotorTwo.stopMotor();
        //shooterTwoMotorTwo.stopMotor();

        // hoodArticulate.stopMotor();

    }

    // public void runShooter(){
    //     // shooterOneMotorOne.setVoltage(7);        
    //     shooterOneMotorTwo.setVoltage(7);

    //     // shooterTwoMotorOne.setVoltage(7);
    //     shooterTwoMotorTwo.setVoltage(7);


    // }

    public void shoot(double distance){

        // if (distance > farShootPos){
        //     targetDeg = HoodConstants.FAR_SHOOT;
        // }

        // else{
        //     targetDeg = HoodConstants.CLOSE_SHOOT;
        // }

        // // probably need two equations, one for close hood up to that point and then another one for far hood position
        // targetRPMOne = (a * Math.pow(b, distance)); //FIXME make less bad, regression, maybe quadratic
        // targetRPMTwo = (a * Math.pow(b, distance)); //FIXME make less bad, regression, maybe quadratic

        targetRPMOne = 3000;
        targetRPMTwo = 3000;


    }

    // public void hoodAway(){
    //     targetDeg = HoodConstants.AWAY;
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
        //targetDeg = HoodConstants.PASS;
    }

    //  public double getHoodPos(){
    //      hoodPos = (hoodEncoder.get() * 360.0) - HoodConstants.LAMPREY_OFFSET; // FIXME I question this
    //      return hoodPos;

    //  }

    //  public boolean hoodInPosition(){
    //     hoodPos = getHoodPos();

    //     if (Math.abs(targetDeg - hoodPos) < 2){
    //         inPosition = true;
    //     }

    //     else{
    //         inPosition = false;
    //     }

    //     return inPosition;

    //  }

    public Command stopCommand(){
        return run (()->stop());
    }

    public void autoRamp(){
        targetRPMOne = autoRPM;
        targetRPMTwo = autoRPM;
    }

    public Command autoRampCommand(){
        return run(()->autoRamp());
    }

    public void periodic(){

        MetersPerSecondOne = (targetRPMOne / 60) * 0.31918581;
        MetersPerSecondTwo = (targetRPMTwo / 60) * 0.31918581;

        
        currentRPMOne = (shooterOneMotorOne.getEncoder().getVelocity());

        pidOutOne = shooterPIDOne.calculate(currentRPMOne, targetRPMOne);//targetRPMOne
        ffOutOne = shooterFFOne.calculate(MetersPerSecondOne);
        voltsOne = MathUtil.clamp (pidOutOne + ffOutOne, 0.0, 12.0); 

        currentRPMTwo = -(shooterTwoMotorOne.getEncoder().getVelocity());

        pidOutTwo = shooterPIDTwo.calculate(currentRPMTwo, targetRPMTwo );
        ffOutTwo = shooterFFTwo.calculate(MetersPerSecondTwo);
        voltsTwo = MathUtil.clamp (pidOutTwo + ffOutTwo, 0.0, 12.0); 

        // hoodPos = getHoodPos();

        // targetDeg = MathUtil.clamp(targetDeg, HoodConstants.lowerLimit, HoodConstants.upperLimit);

        // pidOutHood = hoodController.calculate (hoodPos, targetDeg);
        // pidOutHood = MathUtil.clamp(pidOutHood, -12.0, 12.0);
        // if (hoodPos <= HoodConstants.lowerLimit && pidOutHood < 0) pidOutHood = 0;
        // if (hoodPos >= HoodConstants.upperLimit && pidOutHood > 0) pidOutHood = 0;


        if(targetRPMOne <= 200){
            stop();
            shooterOneMotorOne.setVoltage(0);
            shooterTwoMotorOne.setVoltage(0);
            shooterOneMotorTwo.setVoltage(0);
            shooterTwoMotorTwo.setVoltage(0);


        }

        else{
            shooterOneMotorOne.setVoltage(voltsOne);
            shooterOneMotorTwo.setVoltage(voltsOne);
            shooterTwoMotorOne.setVoltage(voltsTwo);
            shooterTwoMotorTwo.setVoltage(voltsTwo);
        }

        // hoodArticulate.setVoltage(pidOutHood);

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

        


    }
}