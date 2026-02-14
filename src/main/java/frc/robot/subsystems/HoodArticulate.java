package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HoodConstants;

//TODO add slew rate limiter

public class HoodArticulate extends SubsystemBase {
     SparkMax hoodArticulate;

     private final double SHOOT = HoodConstants.SHOOT;
     private final double PASS = HoodConstants.PASS;

     PIDController hoodPID = new PIDController(
      HoodConstants.hoodCoefficients.kP(),
      HoodConstants.hoodCoefficients.kI(),
      HoodConstants.hoodCoefficients.kD());

     public HoodArticulate(){
        hoodArticulate = new SparkMax(DeviceIDs.HOOD, MotorType.kBrushless);

        hoodArticulate.configure(HoodConstants.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     }

     public void stop(){
      hoodArticulate.set(0);
     }

     public void setPosition(double position){


     }




    
}
