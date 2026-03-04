package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;


public class PivotCommand extends Command{
  private final PivotSubsystem pivot;
  private final BooleanSupplier upSupplier;
  private final BooleanSupplier downSupplier;

 // private final DoubleSupplier pivotPower;
  

   public PivotCommand(
      PivotSubsystem pivot,
      BooleanSupplier up, 
      BooleanSupplier down
      //DoubleSupplier pivotPower
      ){

      this.pivot = pivot;
      this.upSupplier = up;
      this.downSupplier = down;
      //this.pivotPower = pivotPower;

      addRequirements(pivot);
  }


  @Override
  public void execute(){

  if (upSupplier.getAsBoolean()){
    pivot.goUpper();
  } 
  
  else if (downSupplier.getAsBoolean()){
    pivot.goLower();
  }
}

 
    

      // if (upSupplier.getAsBoolean()){
      //     pivot.goUpper();
      // } else if (downSupplier.getAsBoolean()) {
      //     pivot.goLower();
      // } else {
      //     pivot.stopPivot();
      //       //pivot.pivotPower(pivotPower.getAsDouble());

      // }
    

  @Override
    public void end(boolean interupted){
      pivot.stopPivot();
  }  


  }












