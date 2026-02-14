package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
    SparkMax hopperMotor;

    public HopperSubsystem(){
      hopperMotor = new SparkMax(DeviceIDs.HOPPER, MotorType.kBrushless);

      hopperMotor.configure(HopperConstants.hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void driveHopperIn(){
      hopperMotor.set((0.85));
  }

  public void driveHopperOut(){
      hopperMotor.set((-0.85));
  }

  public void stopHopper(){
      hopperMotor.stopMotor();
  }

}







