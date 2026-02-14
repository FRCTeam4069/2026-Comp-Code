package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.HoodArticulate;

public class HoodCommand extends Command{
    private final HoodArticulate hood;
    private final BooleanSupplier shootPos;
    private final BooleanSupplier passPos;


    public HoodCommand(
        HoodArticulate hood,
        BooleanSupplier shootPos,
        BooleanSupplier passPos

    ){
      
        this.hood = hood;
        this.shootPos = shootPos;
        this.passPos = passPos;

        addRequirements(hood);
        
    }

    @Override
    public void execute(){

        if(shootPos.getAsBoolean()){

           hood.setPosition(HoodConstants.SHOOT);
        }

        else if (passPos.getAsBoolean()){

            hood.setPosition(HoodConstants.PASS);


        }


    }
    
}

