package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  TalonFX hopperMotorOne;
  TalonFX hopperMotorTwo;

  private double volts = 0.0;

  public HopperSubsystem() {
    hopperMotorOne = new TalonFX(DeviceIDs.HOPPER_ONE);
    hopperMotorTwo = new TalonFX(DeviceIDs.HOPPER_TWO);

    hopperMotorOne.getConfigurator().apply(HopperConstants.hopperOneConfig);
    hopperMotorTwo.getConfigurator().apply(HopperConstants.hopperTwoConfig);

  }

  public void driveHopperIn() {
    hopperMotorOne.setVoltage((12.0));
    hopperMotorTwo.setVoltage(12.0);
  }

  public void driveHopperOut() {
    hopperMotorOne.setVoltage((-7.2));
    hopperMotorTwo.setVoltage(-7.2);

  }

  // public void driveHopper(double hopperPower){
  // hopperMotor.set((hopperPower));
  // }

  public void stopHopper() {
    hopperMotorOne.stopMotor();
    hopperMotorTwo.stopMotor();
  }

  // public void autoHopperIn(){
  // hopperMotor.setVoltage(0.0);
  // }
}
