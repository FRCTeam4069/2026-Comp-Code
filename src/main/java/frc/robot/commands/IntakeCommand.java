package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends Command{
  private final IntakeSubsystem intake;
  private final BooleanSupplier inSupplier;
  //private final BooleanSupplier outSupplier;

   public IntakeCommand(
      IntakeSubsystem intake,
      BooleanSupplier in 
      //BooleanSupplier out
      ){

      this.intake = intake;
    //   this.outSupplier = out;
      this.inSupplier = in;


      addRequirements(intake);
  }

  @Override
  public void execute(){
      if (inSupplier.getAsBoolean()){
          intake.driveFeedIn();
    //   } else if (outSupplier.getAsBoolean()) {
    //       intake.driveFeedOut();
      } else {
          intake.stopFeed();
      }
  }

  @Override
    public void end(boolean interupted){
      intake.stopFeed();
  }
}













