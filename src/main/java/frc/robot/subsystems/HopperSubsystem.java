package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  TalonFX hopperMotorOne;

  public HopperSubsystem() {
    hopperMotorOne = new TalonFX(DeviceIDs.HOPPER);

    hopperMotorOne.getConfigurator().apply(HopperConstants.hopperOneConfig);

  }

  public void driveHopperIn() {
    hopperMotorOne.setVoltage(12);
  }

  public void driveHopperOut() {
    hopperMotorOne.setVoltage((-8.0));

  }

  // public void driveHopper(double hopperPower){
  // hopperMotor.set((hopperPower));
  // }

  public void stopHopper() {
    hopperMotorOne.stopMotor();
  }

  // public void autoHopperIn(){
  // hopperMotor.setVoltage(0.0);
  // }
}
