package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class HopperCommand extends Command{
  private final HopperSubsystem hopper;
  private final BooleanSupplier inSupplier;
  private final BooleanSupplier outSupplier;

   public HopperCommand(
      HopperSubsystem hopper,
      BooleanSupplier in, 
      BooleanSupplier out,
      DoubleSupplier hopperPowerSupplier){

      this.hopper = hopper;
      this.inSupplier = in;
      this.outSupplier = out;


      addRequirements(hopper);
  }

  @Override
  public void execute(){

      if (inSupplier.getAsBoolean()){
          hopper.driveHopperIn();
      } else if (outSupplier.getAsBoolean()) {
          hopper.driveHopperOut();
      } else {
          //hopper.stopHopper();
      }
  }

  @Override
    public void end(boolean interupted){
      hopper.stopHopper();
  }
}













