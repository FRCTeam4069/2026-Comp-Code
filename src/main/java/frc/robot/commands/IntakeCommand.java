package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem.positions;

public class IntakeCommand extends Command{
    IntakeSubsystem intake;
    PivotSubsystem pivot;

    private final double kP = 0.0155, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);
    private double setpoint;
    private double speed;
    private boolean setSpeed = false;

    public IntakeCommand(IntakeSubsystem intake, positions position){
        this.intake = intake;
        if (position == positions.UPPER) {
            setpoint = PivotConstants.UPPER_POSITION - 3;
        } else {
            setpoint = PivotConstants.LOWER_POSITION - 6;
        }
        pivot.setPosition(position);
        // intake.ResetEncoder();
    }

    public IntakeCommand(IntakeSubsystem intake, positions position, double speed){
        this.intake = intake;
        this.speed = speed;
        this.setSpeed = true;
        if (position == positions.UPPER) {
            setpoint = PivotConstants.UPPER_POSITION - 3;
        } else {
            setpoint = PivotConstants.LOWER_POSITION - 6;
        }
        pivot.setPosition(position);
        // intake.ResetEncoder();
    }

    @Override
    public void initialize() {
        if (setSpeed) {
            if (speed == 0) {
                intake.stopFeed();
            } else {
                intake.setIntakeSpeed(speed);

            }
        }
    }

    @Override
    public void execute(){
        pivot.drivePivot(controller.calculate(pivot.getPivotEncoder(), setpoint));
    }

    @Override
    public void end(boolean interrupted){
        pivot.stopPivot();
    }

    @Override
    public boolean isFinished(){
        return MathUtil.isNear(setpoint, pivot.getPivotEncoder(), 5);
    }
    
}