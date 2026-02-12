package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PivotConstants;

import com.revrobotics.spark.SparkBase.ControlType;

public class PivotSubsystem extends SubsystemBase {
    SparkMax pivotMotor;
    private SlewRateLimiter limit;
    private final SparkClosedLoopController pivotController;

    public enum positions{
      UPPER,
      LOWER
    }

    positions p = positions.UPPER;

    private final double LOWER = PivotConstants.LOWER_POSITION - 6;
    private final double UPPER = PivotConstants.UPPER_POSITION - 3;

    public PivotSubsystem(){
      pivotMotor = new SparkMax(DeviceIDs.PIVOT, MotorType.kBrushless);

      pivotMotor.configure(PivotConstants.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      pivotController = pivotMotor.getClosedLoopController();

      pivotMotor.configure(
            PivotConstants.pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

      limit = new SlewRateLimiter(.94);

       pivotMotor.getEncoder().setPosition(UPPER);


  }

  //this might not be needed if i have lower and upperpositions
  public void drivePivot(double speed){
      pivotMotor.set(speed);
      System.out.println("encoder" + getPivotEncoder());
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

  public double getPositionValue(){
      return p == positions.LOWER ? LOWER : UPPER;
  }

  public void periodic() {
        pivotController.setSetpoint(
            getPositionValue(),
            ControlType.kPosition
        );
    }

  public Command setPosition(positions po){
      return this.runOnce(() -> p = po);
  }

  //TODO


  
   public void setBrakeState(int index){
      SparkMaxConfig tempConfig = new SparkMaxConfig();
      tempConfig.idleMode(index == 1 ? IdleMode.kCoast : IdleMode.kBrake);

      pivotMotor.configure(tempConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);
  }

  public void ResetEncoder(){
      pivotMotor.getEncoder().setPosition(UPPER);
  }
}