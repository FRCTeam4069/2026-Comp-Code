package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    SparkMax intakeMotor;

    public IntakeSubsystem(){
      intakeMotor = new SparkMax(DeviceIDs.INTAKE, MotorType.kBrushless);

      intakeMotor.configure(IntakeConstants.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void driveFeedIn(){
      intakeMotor.set((-0.85));
  }

  public Command intakeOn(){
    return runOnce(() -> driveFeedIn());
  }

  public Command intakeOff(){
    return runOnce(() -> stopFeed());
  }

  public void driveFeedOut(){
      intakeMotor.set((0.85));
  }

  public void stopFeed(){
      intakeMotor.stopMotor();
  }

  public double getFeedSpeed(){
      return intakeMotor.getAppliedOutput();
  }

  public void setIntakeSpeed(double speed) {
      intakeMotor.set(speed);
  }

}







