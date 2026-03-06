package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {

    SparkMax pivotMotor;

    public enum positions{
      UPPER,
      LOWER
    }

    positions p = positions.UPPER;

    private final double LOWER_POS = PivotConstants.LOWER_POSITION;
    private final double UPPER_POS = PivotConstants.UPPER_POSITION;


    PIDController upController;
    PIDController downController;

    private static final double pivotTolerance = 1.;

    private boolean inPosition;
    private double error;

    double pidOutput;

    public PivotSubsystem(){

      pivotMotor = new SparkMax(DeviceIDs.PIVOT, MotorType.kBrushless);
      pivotMotor.configure(PivotConstants.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      pivotMotor.getEncoder().setPosition(UPPER_POS);

      upController = new PIDController(PivotConstants.upCoefficients.kP(), PivotConstants.upCoefficients.kI(), PivotConstants.upCoefficients.kD());
      downController = new PIDController(PivotConstants.downCoefficients.kP(), PivotConstants.downCoefficients.kI(), PivotConstants.downCoefficients.kD());

      pidOutput = 0.0;
  }

  public void pivotPower(double pivotPower){
    pivotMotor.set(pivotPower);
  }

  public void stopPivot(){
      pivotMotor.stopMotor();
  }

  public double getPivotEncoder(){
      return pivotMotor.getEncoder().getPosition();
  }

  public void goUpper() {
    p = positions.UPPER;
  }

  public void goLower() {
    p = positions.LOWER;
  }

  public positions getPosition(){
      return p;
  }

  public Command intakeDown(){ //ANNIE LOOK AT THIS AND TELL ME IF IT WORKS
    goLower();
    return run(()-> getPositionValue());
  }

  public Command intakeUp(){
    goUpper();
    return run(()->getPositionValue());

  }

  public double getPositionValue(){
      return p == positions.LOWER ? LOWER_POS : UPPER_POS;
  }

  public void periodic() {
      SmartDashboard.putNumber("PivotPosition", getPivotEncoder());


      if (getPosition() == positions.UPPER) {

        pidOutput = upController.calculate(getPivotEncoder(), UPPER_POS);

      } 
      
      else if (getPosition()== positions.LOWER) {

        pidOutput = downController.calculate(getPivotEncoder(), LOWER_POS);
      }  

      error = getPivotEncoder() - getPositionValue();

      if( Math.abs(error) < pivotTolerance){
        stopPivot();
      }

      else{
        pidOutput = MathUtil.clamp(pidOutput, -0.7, 0.7);
        pivotMotor.set(pidOutput);
      }
    }

  public Command setPosition(positions po){
      return this.runOnce(() -> p = po);
  }

  
  //  public void setBrakeState(int index){
  //     SparkMaxConfig tempConfig = new SparkMaxConfig();
  //     tempConfig.idleMode(index == 1 ? IdleMode.kCoast : IdleMode.kBrake);

  //     pivotMotor.configure(tempConfig,
  //     ResetMode.kNoResetSafeParameters,
  //     PersistMode.kNoPersistParameters);
  // }

  public void ResetEncoder(){
      pivotMotor.getEncoder().setPosition(0);
  }

}
