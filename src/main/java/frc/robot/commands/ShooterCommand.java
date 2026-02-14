package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class ShooterCommand extends Command{
    private final ShooterController shooter;

    private final BooleanSupplier shoot;
    private double distance = 0.0;
    private double currentPositionX = 0.0;
    private double currentPositionY = 0.0;



    private double deltaX = 0.0;
    private double deltaY = 0.0;

    private Alliance alliance = Alliance.Blue;


    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);



    public ShooterCommand(

        ShooterController shooter,
        BooleanSupplier shoot

        ){

        this.shooter = shooter;
        this.shoot = shoot;

        addRequirements(shooter);

        }

    @Override
    public void execute(){
        var result = DriverStation.getAlliance();

        currentPositionX= new Pose2d().getX();
        currentPositionY= new Pose2d().getY();



        if (result.isPresent()) {
            alliance = result.get();
        }


        if (shoot.getAsBoolean()){

            if(alliance == Alliance.Blue){

                deltaX = blueHubX - currentPositionX;
                deltaY = blueHubY - currentPositionY; 

            }

            else {

                deltaX = redHubX - currentPositionX;
                deltaY = redHubY - currentPositionY;

            }

            distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            shooter.shoot(distance);


        }

        else{
            shooter.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }


}

    
