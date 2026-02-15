package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.TestConstants;

public class TestSubsystem extends SubsystemBase {
    private SparkMax left, right;
    private DutyCycleEncoder encoder;

    private final double kP = 4.5, kI = 0, kD = 0.01;

    private boolean isClimbing = false;
    private PIDController controller = new PIDController(kP, kI, kD);


    public TestSubsystem(){
        left = new SparkMax(5, MotorType.kBrushless );
        right = new SparkMax(6, MotorType.kBrushless);


        left.configure(TestConstants.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        right.configure(TestConstants.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new DutyCycleEncoder(0);


    }

    @Override
    public void periodic() {
        
        System.out.println(getTargetAngle());
        System.out.println("right: " + right.get());
        System.out.println("left: " + left.get());
        System.out.println("encoder" + encoder.get());
        

    }

    public void drive(double speed) {
        left.set(-speed);
        right.set(speed);
    }

    // public void runWithSpeed(double speed){
    //     right.set(speed);
    // }
    public Command angleCommand(){
        return run(() -> stop());
    }

    public void goToAngle(){

        right.set(controller.calculate(getAngle(), Math.toRadians(70)));
        left.set(-controller.calculate(getAngle(), Math.toRadians(70)));

    
    }

    public double getEncoder(){
        return encoder.get();
    }

    public double getAngle(){
        return (encoder.get() - TestConstants.ShooterOffset) * (2*Math.PI);
    }

    shooterAngles angle;

    public Command setAngle(shooterAngles a){
        return this.runOnce(() -> angle = a);
    }

    public Command setSpeed(DoubleSupplier speed) {
        return this.run(() -> drive(speed.getAsDouble()));
    }

    public double getTargetAngle(){
        if(angle == shooterAngles.NINTEY) return (Math.PI/2);
        else if(angle == shooterAngles.NEG_NINTEY) return -Math.PI/2;
        return 0;
    }
    public enum shooterAngles{
        NINTEY,
        ZERO,
        NEG_NINTEY
    }

    double deg = 0;

    public void setCustomAngle(double angdeg){
        deg = angdeg;

        var speed = 0.0;
        if (angdeg < 90 && angdeg > 0) {
            speed = controller.calculate(getAngle(), Math.toRadians(angdeg));
        }
        else {
            right.set(0);
            left.set(0);
        }

        right.set(speed);
        left.set(-speed);   
        
    }

    public boolean isClimbing(){
        return isClimbing;
    }

    public boolean atPosition(){
        if (Math.abs(getAngle() - Math.toRadians(deg)) < 5) return true;
        return false;
    }

    public Command changeClimbStatus(){
        return this.runOnce(() -> isClimbing = true);
    }

    public void stop(){
        left.stopMotor();
        right.stopMotor();
    }

    boolean down = false;

    public Command setDown(){
        return this.runOnce(() -> down = true);
    }

    public boolean NotDown(){
        return down;
    }

    //  public boolean getLeftSensorError(){
    //     return left.getFault(FaultID.kSensorFault);
    // }
    // public boolean getLeftMotorError(){
    //     return left.getFault(FaultID.kMotorFault);
    // }

    // public double getLeftErrors(){
    //     return left.getFaults();
    // }   
    
    // public boolean getRightSensorError(){
    //     return right.getFault(FaultID.kSensorFault);
    // }
    // public boolean getRightMotorError(){
    //     return right.getFault(FaultID.kMotorFault);
    // }

    // public double getRightErrors(){
    //     return right.getFaults();
    // }   

}