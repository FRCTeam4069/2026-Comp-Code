package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TestSubsystem;

public class TestCommand extends Command {
    private TestSubsystem shooter;

    public TestCommand(TestSubsystem c){

        shooter = c;
        
        addRequirements(c);
    }

    public void execute(){
        //     shooter.goToAngle();
        // // }
        // // else if(shooter.isClimbing() && !shooter.NotDown()){
        // //     shooter.setCustomAngle(1);
        // // }
        // else{
        //     shooter.setCustomAngle(15);
        // }
        
    }

    public void goTo70(){

        shooter.goToAngle();

    }

    public void end(boolean interrupted){

    }

    public boolean isFinished(){
        return shooter.controller.atSetpoint();
    }
}
