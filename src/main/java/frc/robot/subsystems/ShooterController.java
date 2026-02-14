package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ShooterConstants;


public class ShooterController extends SubsystemBase {
    SparkFlex shooterOneLeft, shooterOneRight, shooterTwoLeft, shooterTwoRight;

    //TODO ask tyler how to set follow

    private  double targetRPM = 0.0;
    double currentRPM = 0.0;
    

    private final double a = 0.0; //FIXME
    private  double distance = 0.0; //FIXME
    private final double b = 0.0; //FIXME

    double pidOut=0.0;
    double ffOut = 0.0;
    double volts =0.0;

    PIDController shooterPID = new PIDController( 
        ShooterConstants.shooterCoefficients.kP(),
        ShooterConstants.shooterCoefficients.kI(), 
        ShooterConstants.shooterCoefficients.kD());

    SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(ShooterConstants.shooterCoefficients.kS(),ShooterConstants.shooterCoefficients.kV(),ShooterConstants.shooterCoefficients.kA()); 
    //TODO ask tyler the variables


    public ShooterController(){

        shooterOneLeft = new SparkFlex(DeviceIDs.SHOOTER_ONE_LEFT, MotorType.kBrushless);
        shooterOneRight = new SparkFlex(DeviceIDs.SHOOTER_ONE_RIGHT, MotorType.kBrushless);
        shooterTwoLeft = new SparkFlex(DeviceIDs.SHOOTER_TWO_LEFT, MotorType.kBrushless);
        shooterTwoRight = new SparkFlex(DeviceIDs.SHOOTER_TWO_RIGHT, MotorType.kBrushless);

        shooterOneLeft.configure(ShooterConstants.shooterOneLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterOneRight.configure(ShooterConstants.shooterOneRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTwoLeft.configure(ShooterConstants.shooterTwoLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTwoRight.configure(ShooterConstants.shooterTwoRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void stop(){
        shooterOneLeft.set(0);
        shooterOneRight.set(0);
        shooterTwoLeft.set(0);
        shooterTwoRight.set(0);
    }

    public void shoot(double distance){

        targetRPM = (a * Math.pow(b, distance)); //FIXME find a and b
        currentRPM = (shooterOneLeft.getEncoder().getVelocity());

        pidOut = shooterPID.calculate(currentRPM, targetRPM);
        ffOut = shooterFF.calculate(targetRPM);
        volts = MathUtil.clamp (pidOut + ffOut, -12.0, 12.0);

        

        shooterOneLeft.setVoltage(volts);
        shooterOneRight.setVoltage(volts);
        shooterTwoLeft.setVoltage(volts);
        shooterTwoRight.setVoltage(volts);  

    }
}