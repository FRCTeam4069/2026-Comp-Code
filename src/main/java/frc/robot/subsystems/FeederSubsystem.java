package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    TalonFX feederMotor;

    public FeederSubsystem(){
      feederMotor = new TalonFX(DeviceIDs.FEEDER);

      feederMotor.getConfigurator().apply(FeederConstants.feederConfig);
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







