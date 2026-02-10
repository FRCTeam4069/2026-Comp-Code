package frc.robot.commands;




import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;




import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeController;




public class RunIntakeCommand extends Command{
  private final IntakeController intake;
  private final BooleanSupplier inSupplier;
  private final BooleanSupplier outSupplier;




   public RunIntakeCommand(
      IntakeController intake,
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













