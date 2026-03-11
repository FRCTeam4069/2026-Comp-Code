package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;


public class IntakeCommand extends Command{
  private final IntakeSubsystem intake;
  private final PivotSubsystem pivot;


  private final DoubleSupplier inSupplier;
  private final DoubleSupplier outSupplier;


  private final BooleanSupplier up;
  private final BooleanSupplier down;

   public IntakeCommand(
      IntakeSubsystem intake,
      PivotSubsystem pivot,
      DoubleSupplier in, 
      DoubleSupplier out,

      BooleanSupplier up, 
      BooleanSupplier down
      ){

      this.intake = intake;
      this.pivot = pivot;

      this.outSupplier = out;
      this.inSupplier = in;

      this.up = up;
      this.down= down;


      addRequirements(intake);
  }

  @Override
  public void execute(){

      if (inSupplier.getAsDouble() > 0.2){
          intake.driveFeedInTele(inSupplier.getAsDouble());
      } 
      else if (outSupplier.getAsDouble() > 0.2) {
          intake.driveFeedOut();
      } 

      else if (up.getAsBoolean()){
        pivot.goUpper();
        intake.driveFeedIn();
      } 
  
      else if (down.getAsBoolean()){
        pivot.goLower();
      }

      else {
          intake.stopFeed();
      }
  }

  @Override
    public void end(boolean interupted){
      intake.stopFeed();
      pivot.stopPivot();
  }
}
  
























