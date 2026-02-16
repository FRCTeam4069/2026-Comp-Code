package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem;

public class TestCommand extends Command {

    private final TestSubsystem testSubsystem;

    private final BooleanSupplier go;
    private final BooleanSupplier stop;

    public TestCommand(
        TestSubsystem testSubsystem,
        BooleanSupplier go,
        BooleanSupplier stop

    ){

        this.go = go;
        this.stop = stop;
        this.testSubsystem = testSubsystem;

        addRequirements(testSubsystem);

    }

    @Override
    public void execute(){
        if (go.getAsBoolean()){
            testSubsystem.driveWithCustomSpeed( 0.8, 0.8);

        }

        else if(stop.getAsBoolean()){

        testSubsystem.driveWithCustomSpeed( 0.0, 0.0);

        }

    }

  

    
}
