package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.HoodArticulate;

public class HoodCommand extends Command{
    private final HoodArticulate hood;
    private final BooleanSupplier closeShootPos;
    private final BooleanSupplier farShootPos;
    private final BooleanSupplier passPos;


    public HoodCommand(
        HoodArticulate hood,
        BooleanSupplier farShootPos,
        BooleanSupplier closeShootPos,
        BooleanSupplier passPos

    ){
      
        this.hood = hood;
        this.farShootPos = farShootPos;
        this.closeShootPos = closeShootPos;
        this.passPos = passPos;

        addRequirements(hood);
        
    }

    @Override
    public void execute(){


        if(farShootPos.getAsBoolean()){
           hood.setPosition(HoodConstants.FAR_SHOOT);
        }

        else if (closeShootPos.getAsBoolean()){
            hood.setPosition(HoodConstants.CLOSE_SHOOT);  
        }
        
        else if (passPos.getAsBoolean()){
            hood.setPosition(HoodConstants.PASS);
        }


    }
    
}

