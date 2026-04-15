package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  SparkFlex intakeMotor;

  private double power = 0.0;

  public IntakeSubsystem() {
    intakeMotor = new SparkFlex(DeviceIDs.INTAKE, MotorType.kBrushless);

    intakeMotor.configure(IntakeConstants.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void driveFeedInTele(double intakePower) {
    power = MathUtil.clamp((intakePower), 0, 0.8);
    intakeMotor.set((Math.pow(intakePower, 2)));
  }

  public void driveFeedIn() {
    intakeMotor.setVoltage((12));
  }

  public Command intakeOn() {
    return runOnce(() -> driveFeedIn());
  }

  public Command intakeOff() {
    return runOnce(() -> stopFeed());
  }

  public void driveFeedOut() {
    intakeMotor.setVoltage((-12));
  }

  public void stopFeed() {
    intakeMotor.set(0);
  }

  public double getFeedSpeed() {
    return intakeMotor.getAppliedOutput();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

}
