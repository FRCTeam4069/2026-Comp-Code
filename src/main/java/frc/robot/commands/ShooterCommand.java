package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.FeederSubsystem;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;


public class ShooterCommand extends Command{
    private final ShooterController shooter;
    private final FeederSubsystem feeder;

    private final BooleanSupplier shoot;
    private double distance = 0.0;
    private double currentPositionX = 0.0;
    private double currentPositionY = 0.0;

    private double deltaX = 0.0;
    private double deltaY = 0.0;

    private Alliance alliance = Alliance.Blue;

    private static final double RPMDiff = 50; //FIXME


    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);



    public ShooterCommand(

        ShooterController shooter,
        FeederSubsystem feeder,
        BooleanSupplier shoot

        ){

        this.shooter = shooter;
        this.feeder = feeder;
        this.shoot = shoot;

        addRequirements(shooter, feeder);

        }

    @Override
    public void execute(){
        var result = DriverStation.getAlliance();

        currentPositionX = shooter.getCurrentRobotPose().getX();
        currentPositionY = shooter.getCurrentRobotPose().getY();


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

            if ((Math.abs(shooter.targetRPMOne -shooter.currentRPMOne) <= RPMDiff)) { 

                feeder.driveFeederIn();
            }

            else{
                feeder.stopFeeder();
            }

        }

        else{
            shooter.stop();
            feeder.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stopFeeder();

    }
}