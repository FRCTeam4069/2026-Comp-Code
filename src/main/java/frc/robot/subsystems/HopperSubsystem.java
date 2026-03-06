package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
    SparkMax hopperMotor;

    private double volts = 0.0;

    private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(10.0);


    public HopperSubsystem(){
      hopperMotor = new SparkMax(DeviceIDs.HOPPER, MotorType.kBrushless);

      hopperMotor.configure(HopperConstants.hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void driveHopperIn(){
      volts = slewRateLimiter.calculate(7.2);
      hopperMotor.setVoltage((volts));
  }

  public void driveHopperOut(){
      volts = slewRateLimiter.calculate(4.8);
      hopperMotor.setVoltage((volts));
  }

//   public void driveHopper(double hopperPower){
//       hopperMotor.set((hopperPower));
//   }

  public void stopHopper(){
      hopperMotor.stopMotor();
  }
}







