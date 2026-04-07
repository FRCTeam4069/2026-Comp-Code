package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;

public class FeederSubsystem extends SubsystemBase {
    SparkMax feederMotor;

    public FeederSubsystem(){
      // feederMotor = new TalonFX(DeviceIDs.FEEDER);
      feederMotor = new SparkMax(DeviceIDs.FEEDER, MotorType.kBrushless);

      //feederMotor.getConfigurator().apply(FeederConstants.feederConfig);
      feederMotor.configure(FeederConstants.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void driveFeederIn(){
      feederMotor.setVoltage((12));
  }

  public void driveFeederOut(){
      feederMotor.setVoltage((-8));
  }

  public void stopFeeder(){
      feederMotor.stopMotor();
  }

  public void autoFeederIn(){
    feederMotor.setVoltage(10.5);
  }
}







