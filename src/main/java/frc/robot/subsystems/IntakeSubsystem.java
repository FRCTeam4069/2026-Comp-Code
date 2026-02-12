package frc.robot.subsystems;

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


public class IntakeSubsystem extends SubsystemBase {
    SparkMax feedMotor;

    public IntakeSubsystem(){
      feedMotor = new SparkMax(DeviceIDs.INTAKE_FEED, MotorType.kBrushless);

      feedMotor.configure(IntakeConstants.feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  public double getFeedSpeed(){
      return feedMotor.getAppliedOutput();
  }

  public void setIntakeSpeed(double speed) {
      feedMotor.set(speed);
  }
    
}







