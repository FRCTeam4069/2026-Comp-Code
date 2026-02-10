package frc.robot.subsystems;




import java.security.AuthProvider;
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
  SparkMax feedMotor, pivotMotor;




 private SlewRateLimiter limit;




  private final double LOWER = IntakeConstants.LOWER_POSITION - 6;
  private final double UPPER = IntakeConstants.UPPER_POSITION - 3;








  public IntakeController(){
      feedMotor = new SparkMax(DeviceIDs.INTAKE_FEED, MotorType.kBrushless);
      pivotMotor = new SparkMax(DeviceIDs.PIVOT, MotorType.kBrushless);




      feedMotor.configure(IntakeConstants.feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      pivotMotor.configure(IntakeConstants.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




      pivotMotor.getEncoder().setPosition(UPPER);




      limit = new SlewRateLimiter(.94);




  }




  public void driveFeedIn(){
      feedMotor.set((0.85));
  }




  public void driveFeedOut(){
      feedMotor.set((-0.85));
  }




  public void stopFeed(){
      feedMotor.stopMotor();
  }




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




  public double getFeedSpeed(){
      return feedMotor.getAppliedOutput();
  }




  public void setIntakeSpeed(double speed) {
      feedMotor.set(speed);
  }




  //hey! its leticia! good luck coding pleas emake it good
    public enum positions{
      UPPER,
      LOWER
  }




  positions p = positions.UPPER;




  public double getPositionValue(){
      return p == positions.LOWER ? LOWER : UPPER;
  }




  public Command setPosition(positions po){
      return this.runOnce(() -> p = po);
  }




  //TODO












  public positions getPosition(){
      return p;
  }
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







