package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class RunIntakeCommand extends Command{
  private final IntakeSubsystem intake;
  private final BooleanSupplier inSupplier;
  private final BooleanSupplier outSupplier;

   public RunIntakeCommand(
      IntakeSubsystem intake,
      BooleanSupplier in, BooleanSupplier out){

      this.intake = intake;
      this.inSupplier = in;
      this.outSupplier = out;

      addRequirements(intake);
  }


  @Override
  public void execute(){
      if (inSupplier.getAsBoolean()){
          intake.driveFeedIn();
      } else if (outSupplier.getAsBoolean()) {
          intake.driveFeedOut();
      } else {
          intake.stopFeed();
      }
  }

  @Override
    public void end(boolean interupted){
      intake.stopFeed();
  }
}













