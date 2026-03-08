package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;



public class ShootWithTimeout extends Command{
    private final ShooterController shooter;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final PivotSubsystem pivot;
    private final SwerveDrivetrain swerve;


    private double distance = 0.0;
    private double currentPositionX = 0.0;
    private double currentPositionY = 0.0;
    private final double shootTime = 5.0; //FIXME
    private double desiredHeading = 0.0;

    private final Timer timer = new Timer();
    private boolean finished = false;

    private double deltaX = 0.0;
    private double deltaY = 0.0;

    private Alliance alliance = Alliance.Blue;

    private static final double RPMDiff = 50; //FIXME


    private final double redHubX = Units.inchesToMeters(469.1);
    private final double redHubY = Units.inchesToMeters(158.85);
    private final double blueHubX = Units.inchesToMeters(182.1);
    private final double blueHubY = Units.inchesToMeters(158.85);


    public ShootWithTimeout(
        ShooterController shooter,
        FeederSubsystem feeder,
        HopperSubsystem hopper,
        PivotSubsystem pivot,
        SwerveDrivetrain swerve

    ){
        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.pivot = pivot;
        this.swerve = swerve;
    }

    @Override
    public void execute(){
        var result = DriverStation.getAlliance();

        currentPositionX = shooter.getCurrentRobotPose().getX();
        currentPositionY = shooter.getCurrentRobotPose().getY();

        if (result.isPresent()) {
            alliance = result.get();
        }

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

        if ((Math.abs(shooter.targetRPMOne -shooter.currentRPMOne) <= RPMDiff)) { //&& shooter.hoodInPosition() 
            feeder.autoFeederIn();
            hopper.autoHopperIn();

            if (!timer.isRunning()){
                timer.restart();
            }

            if(timer.hasElapsed(2.5)){
                pivot.goUpper();
            }
        }
        

        else{
            feeder.stopFeeder();
            hopper.stopHopper();
            timer.stop();
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stopFeeder();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished(){
        // for second comp do this with the limelite detects no balls thing

        return timer.hasElapsed(shootTime);

    }
}