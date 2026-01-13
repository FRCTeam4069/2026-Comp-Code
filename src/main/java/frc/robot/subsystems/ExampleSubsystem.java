// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
    private TalonFX motor1;
    private TalonFX motor2;
  
   

  public ExampleSubsystem(){
    motor1 = new TalonFX(Constants.motor1ID);
    motor2 = new TalonFX(Constants.motor2ID);


    motor1.getConfigurator().apply(Constants.motor1config);
    motor2.getConfigurator().apply(Constants.motor2config);
   }

   private void setMotorSpeeds(double speed1, double speed2){
    motor1.set(speed1);
    motor2.set(speed2);
   }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
        setMotorSpeeds(1.0, 1.0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
