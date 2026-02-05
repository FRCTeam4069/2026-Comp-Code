package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeController;

public class FeedIntakeCommand extends Command{
    private final IntakeController intake;
     private final BooleanSupplier feedSpeed;

    
    public FeedIntakeCommand(
        IntakeController intake, 
        BooleanSupplier feedSpeed){

        this.intake = intake;
        this.feedSpeed = feedSpeed;

        addRequirements(intake);
        //addRequirements(RobotContainer.intake);
    }

    @Override
    public void execute(){
        intake.driveFeedIn();
    }

    @Override

    public void end(boolean interupted){
        intake.stopFeed();
    }

    @Override

    public boolean isFinished(){
        return false;
    }
} 

