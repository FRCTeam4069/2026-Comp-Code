package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode; //TODO figure out which is the right version of this import
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.HoodConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class HoodArticulate extends SubsystemBase {

   SparkMax hoodArticulate;

   private double targetDeg = 0.0;

   private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(HoodConstants.LAMPREY_PORT);// FIXME FO‰ PORT


   //private final double toDegrees = 360/4096; //TODO check the ticks val is right
   private double hoodPos = 0.0;
   private double pidOut = 0.0;

   

   PIDController hoodController = new PIDController(
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

     public double getHoodPos(){
         hoodPos = (hoodEncoder.get() * 360.0) - HoodConstants.LAMPREY_OFFSET; // FIXME I question this
         return hoodPos;

     }

      public void setPosition(double targetDeg){
         this.targetDeg = MathUtil.clamp(targetDeg, HoodConstants.lowerLimit, HoodConstants.upperLimit);

      }

     @Override
     public void periodic(){
         hoodPos = getHoodPos();

         pidOut = hoodController.calculate (hoodPos, targetDeg);
         pidOut = MathUtil.clamp(pidOut, -12.0, 12.0);

         if (hoodPos <= HoodConstants.lowerLimit && pidOut < 0) pidOut = 0;
         if (hoodPos >= HoodConstants.upperLimit && pidOut > 0) pidOut = 0;

         hoodArticulate.setVoltage(pidOut);

        
     }


    
}
