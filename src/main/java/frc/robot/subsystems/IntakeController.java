package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;

public class IntakeController extends SubsystemBase {
    SparkMax feedMotor, artMotor1;

    private SlewRateLimiter limit;

    private final double LOWER = IntakeConstants.LOWER_POSITION - 6;
    private final double UPPER = IntakeConstants.UPPER_POSITION - 3;


    public IntakeController(){
        feedMotor = new SparkMax(DeviceIDs.INTAKE_FEED, MotorType.kBrushless);
        artMotor1 = new SparkMax(DeviceIDs.INTAKE_ARTICULATE, MotorType.kBrushless);

        feedMotor.configure(IntakeConstants.feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        artMotor1.configure(IntakeConstants.artConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //feedMotor.setIdleMode(IdleMode.kCoast);
        //artMotor1.setIdleMode(IdleMode.kBrake);
        // artMotor1.setSoftLimit(SoftLimitDirection.kForward, 0);
        // artMotor1.setSoftLimit(SoftLimitDirection.kReverse, IntakeConstants.UPPER_POSITION);

        artMotor1.getEncoder().setPosition(UPPER);

        limit = new SlewRateLimiter(.94);

        //artMotor1.burnFlash();
        // feedMotor.burnFlash();
    }

    public void driveFeedIn(){
        feedMotor.set((-0.85));
    }

    public void driveFeedOut(){
        feedMotor.set((0.85));
    }

    public void backIntake(){
        feedMotor.set(limit.calculate(1));
    }
    public void stopFeed(){
        feedMotor.stopMotor();
    }
    public void driveArt(double speed){
        artMotor1.set(speed);
        //System.out.println("encoder" + getEncoder());
    }
    public void stopArt(){
        artMotor1.stopMotor();
    }

    public double getEncoder(){
        return artMotor1.getEncoder().getPosition();
    }

    public double getFeedSpeed(){
        return feedMotor.getAppliedOutput();
    }

    public void setIntakeSpeed(double speed) {
        feedMotor.set(speed);
    }

    //hey! its leticia! good luck coding pleas emake it good 
    
    positions p = positions.UPPER;

    public double getPositionValue(){
        return p == positions.LOWER ? LOWER : UPPER;
    }

    public Command setPosition(positions po){
        return this.runOnce(() -> p = po);
    }

    //TODO 
    
    public Command defaultCommand(BooleanSupplier in, BooleanSupplier out) {
    return run(() -> {
        if (in.getAsBoolean()) {
            driveFeedIn();
        } else if (out.getAsBoolean()) {
            driveFeedOut();
        }
        else {
            stopFeed();
        }
    });
}


    public positions getPosition(){
        return p;
    }
    
    public enum positions{
        UPPER,
        LOWER
    }

    public void setBrakeState(int index){
    SparkMaxConfig tempConfig = new SparkMaxConfig();
    tempConfig.idleMode(index == 1 ? IdleMode.kCoast : IdleMode.kBrake);

    artMotor1.configure(tempConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    }

    //TODO hi what is this

    public void ResetEncoder(){
        artMotor1.getEncoder().setPosition(UPPER);
    }

    // public boolean getArtSensorError() {
    //     return artMotor1.getFaults().sensor;
    // }

    // public boolean getArtMotorError() {
    //     return artMotor1.getFaults().motorType;
    // }

    // public Faults getArtErrors() {
    //     return artMotor1.getFaults();
    // }

    // public boolean getFeedSensorError() {
    //     return feedMotor.getFaults().sensor;
    // }

    // public boolean getFeedMotorError() {
    //     return feedMotor.getFaults().motorType;
    // }

    // public Faults getFeedErrors() {
    //     return feedMotor.getFaults();
    // }



}