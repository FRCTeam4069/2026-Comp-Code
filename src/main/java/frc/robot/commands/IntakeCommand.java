package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.positions;


public class IntakeCommand extends Command{
  private final IntakeSubsystem intake;
  private final PivotSubsystem pivot;


  private final DoubleSupplier inSupplier;
  private final DoubleSupplier outSupplier;
   private final DoubleSupplier inSupplier2;
  private final DoubleSupplier outSupplier2;


  private final BooleanSupplier up;
  private final BooleanSupplier down;

   public IntakeCommand(
      IntakeSubsystem intake,
      PivotSubsystem pivot,
      DoubleSupplier in, 
      DoubleSupplier out,
      DoubleSupplier in2,
      DoubleSupplier out2,

      BooleanSupplier up, 
      BooleanSupplier down
      ){

      this.intake = intake;
      this.pivot = pivot;

      this.outSupplier = out;
      this.inSupplier = in;
      this.outSupplier2 = out2;
      this.inSupplier2 = in2;

      this.up = up;
      this.down= down;


      addRequirements(intake);
  }

  @Override
  public void execute(){


      if (up.getAsBoolean() && pivot.getDesiredPosition()!=positions.UPPER){
        pivot.goUpper();
        intake.driveFeedIn();
      } 
  
      else if (down.getAsBoolean() && pivot.getDesiredPosition()!=positions.LOWER){
        pivot.goLower();
      }

      else if (inSupplier.getAsDouble() > 0.2){
          intake.driveFeedInTele(inSupplier.getAsDouble());
      } 
      // else if (outSupplier.getAsDouble() > 0.2) {
      //     intake.driveFeedOut();
      // } 

      else if (inSupplier2.getAsDouble() > 0.2){
        intake.driveFeedIn();
      }

      else if (outSupplier2.getAsDouble() > 0.2){
        intake.driveFeedOut();
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
  
























