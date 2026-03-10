package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;


public class ShooterCommand extends Command{
    private final ShooterController shooter;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;


    private final DoubleSupplier shoot;
   private final DoubleSupplier pass;
    private final BooleanSupplier reverse;

    private double distance = 0.0;
    private double currentPositionX = 0.0;
    private double currentPositionY = 0.0;

    private double deltaX = 0.0;
    private double deltaY = 0.0;

    private Alliance alliance = Alliance.Blue;

    private static final double RPMDiff = 50; 


    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);

   // private final DoubleSupplier hopperPowerSupplier;
//    private final BooleanSupplier testShoot;
    private final BooleanSupplier feederManual;

    // private final BooleanSupplier passTest;
    // private final BooleanSupplier closeShootTest;
    // private final BooleanSupplier farShootTest;
    // private final BooleanSupplier away; 

    private Boolean shootReady;
    private final Timer timer = new Timer();

    private final BooleanSupplier trenchShoot;
    private final BooleanSupplier closeShoot;


   
    public ShooterCommand(

        ShooterController shooter,
        FeederSubsystem feeder,
        HopperSubsystem hopper,
        
        DoubleSupplier shoot,
        DoubleSupplier pass,
        BooleanSupplier reverse,
        BooleanSupplier feederManual,

        BooleanSupplier trenchShoot,
        BooleanSupplier closeShoot

        // BooleanSupplier passTest,
        // BooleanSupplier closeShootTest,
        // BooleanSupplier farShootTest,
        // BooleanSupplier away
        // BooleanSupplier testShoot 
         //DoubleSupplier hopperPowerSupplier

        ){

        this.shooter = shooter;
        this.feeder = feeder;        
        this.hopper = hopper;

        this.shoot = shoot;
        this.pass = pass;
        this.reverse = reverse;
        this.feederManual = feederManual;

        this.trenchShoot= trenchShoot;
        this.closeShoot = closeShoot;

        // this.passTest = passTest;
        // this.closeShootTest = closeShootTest;
        // this.farShootTest= farShootTest;
        // this.away = away;
        // this.testShoot = testShoot;
        //this.hopperPowerSupplier = hopperPowerSupplier;



        addRequirements(shooter, feeder, hopper);

        }

    @Override
    public void execute(){
        var result = DriverStation.getAlliance();
       
        currentPositionX = shooter.getCurrentRobotPose().getX();
        currentPositionY = shooter.getCurrentRobotPose().getY();


        if (result.isPresent()) {
            alliance = result.get();
        }


        if (shoot.getAsDouble() > 0.2){

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

            if ((Math.abs(shooter.targetRPMOne -shooter.currentRPMOne) <= RPMDiff)  && shooter.hoodInPosition() ) { 
               shootReady = true;
               timer.start();
            }

            if(shootReady = true && timer.hasElapsed(0.25)){
            hopper.driveHopperIn();            
            }

            if(shootReady = true && timer.hasElapsed(0.3)){
                feeder.driveFeederIn();
            }

        }

        else if (feederManual.getAsBoolean()){

            feeder.driveFeederIn();
            hopper.driveHopperIn();
        }

        else if(closeShoot.getAsBoolean()){
            shooter.manualCloseShoot();

            hopper.driveHopperIn();


             if ((Math.abs(shooter.targetRPMOne -shooter.currentRPMOne) <= RPMDiff)  && shooter.hoodInPosition() ) { 
               shootReady = true;
               timer.start();
            }

            if(shootReady = true && timer.hasElapsed(0.3)){
                feeder.driveFeederIn();
            }
        }

        else if (trenchShoot.getAsBoolean()){
            shooter.trenchShoot();

            hopper.driveHopperIn();


             if ((Math.abs(shooter.targetRPMOne -shooter.currentRPMOne) <= RPMDiff)  && shooter.hoodInPosition() ) { 
               shootReady = true;
               timer.start();
            }

            if(shootReady = true && timer.hasElapsed(0.3)){
                feeder.driveFeederIn();
            }
        }

        else if (pass.getAsDouble() > 0.2){
            shooter.pass();
            hopper.driveHopperIn();


             if ((Math.abs(shooter.targetRPMOne -shooter.currentRPMOne) <= RPMDiff)  && shooter.hoodInPosition() ) { 
               shootReady = true;
               timer.start();
            }

            if(shootReady = true && timer.hasElapsed(0.3)){
                feeder.driveFeederIn();
            }
        }

        else if(reverse.getAsBoolean()){
            hopper.driveHopperOut();
            feeder.driveFeederOut();
        }

        else{
            shooter.stop();
            feeder.stopFeeder();
            hopper.stopHopper();
            shooter.hoodAway();
            shootReady = false;

            timer.stop();
            timer.reset();
        }

       

        SmartDashboard.putNumber("distance",distance);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        // feeder.stopFeeder();
    }
}