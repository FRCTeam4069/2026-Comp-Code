package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;


public class FeederCommand extends Command{
  private final FeederSubsystem feeder;
  private final BooleanSupplier inSupplier;
  private final BooleanSupplier outSupplier;

   public FeederCommand(
      FeederSubsystem feeder,
      BooleanSupplier up, BooleanSupplier down){

      this.feeder = feeder;
      this.inSupplier = up;
      this.outSupplier = down;

      addRequirements(feeder);
  }


  @Override
  public void execute(){
      if (inSupplier.getAsBoolean()){
          feeder.driveFeederIn();
      } else if (outSupplier.getAsBoolean()) {
          feeder.driveFeederOut();
      } else {
          feeder.stopFeeder();
      }
  }

  @Override
    public void end(boolean interupted){
      feeder.stopFeeder();
  }
}













