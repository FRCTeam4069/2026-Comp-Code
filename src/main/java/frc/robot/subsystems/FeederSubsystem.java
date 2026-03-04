package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    SparkMax feederMotor;

    public FeederSubsystem(){
      feederMotor = new SparkMax(DeviceIDs.FEEDER, MotorType.kBrushless);

      feederMotor.configure(FeederConstants.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void driveFeederIn(){
      feederMotor.setVoltage((10.2));
  }

  public void driveFeederOut(){
      feederMotor.setVoltage((-10.2));
  }

  public void stopFeeder(){
      feederMotor.stopMotor();
  }
}







