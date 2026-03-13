package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {

    SparkMax pivotMotor;

    public enum positions{
      UPPER,
      LOWER,
      NEUTRAL
    }

    positions p = positions.UPPER;

    private final double LOWER_POS = PivotConstants.LOWER_POSITION;
    private final double UPPER_POS = PivotConstants.UPPER_POSITION;

    private double power = 0.0;
    
    
        PIDController upController;
        PIDController downController;
    
        private static final double pivotTolerance = 1.0;
    
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
    
      // public void pivotPower(double pivotPower){
      //   pivotMotor.set(pivotPower);
      // }
    
      public void stopPivot(){
          pivotMotor.stopMotor();
      }
    
      public double getPivotEncoder(){
          return pivotMotor.getEncoder().getPosition();
      }
    
      public void goUpper() {
        if ((getPivotEncoder() - getPositionValue()) < 15) {
          p = positions.UPPER;
        } 
      }
    
      public void goLower() {
        p = positions.LOWER;
      }
    
      public positions getPosition(){
          return p;
      }
    
      public Command intakeDown(){ //ANNIE LOOK AT THIS AND TELL ME IF IT WORKS
        return runOnce(()-> goLower());
      }
    
      public Command intakeUp(){
        return runOnce(()->goUpper());
      }
    
      public double getPositionValue(){
          return p == positions.LOWER ? LOWER_POS : UPPER_POS;
      }

      public positions getDesiredPosition(){
        return p;
      }
    
      public void periodic() {
          SmartDashboard.putNumber("PivotPosition", getPivotEncoder());

          if (p == positions.NEUTRAL) {
            stopPivot();
          }


          else {
            error = getPivotEncoder() - getPositionValue();
            SmartDashboard.putNumber("pivot error", error);
            SmartDashboard.putNumber("pivot current", pivotMotor.getOutputCurrent());

      
            if( Math.abs(error) < pivotTolerance){
              stopPivot();
            }
      
            else{
              if (getPosition() == positions.UPPER) {
      
                pidOutput = (upController.calculate(getPivotEncoder(), UPPER_POS));
                power = MathUtil.clamp(pidOutput, 0., 0.5);
                
                if (pivotMotor.getOutputCurrent() > 10 && getPivotEncoder() > -17) {
                  p = positions.NEUTRAL;
                }

              } 
              else if (getPosition()== positions.LOWER) {
                pidOutput = downController.calculate(getPivotEncoder(), LOWER_POS);
                power =MathUtil.clamp(pidOutput, -0.5, 0);
              }  

              pivotMotor.set(power);
            }
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
