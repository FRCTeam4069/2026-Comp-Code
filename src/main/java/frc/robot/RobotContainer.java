
package frc.robot;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.ThroughTrench;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder; 
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
      public static final SwerveDrivetrain drive = new SwerveDrivetrain();
      public static final IntakeController intake = new IntakeController();


      private final CommandXboxController controller0 = new CommandXboxController(0);
      private final CommandXboxController controller1 = new CommandXboxController(1);

      private final SendableChooser<Command> autoChooser;

        // addSysIdCommands()

  public final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();



  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings


      autoChooser = AutoBuilder.buildAutoChooser();

      //SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // new Trigger(exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(exampleSubsystem));

    // m_driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      //An example command will be run in autonomous
      return autoChooser.getSelected();
    
  }
  
    public Command defaultDriveCommand() {
        return new FieldCentricDrive(
                drive,
                () -> -controller0.getLeftY(),
                () -> -controller0.getLeftX(),
                () -> -controller0.getRightX(),
                () -> controller0.getHID().getAButton(),
                () -> controller0.getHID().getYButton());
    }
    
    public Command defaultIntakeCommand() {
    return new RunIntakeCommand(
        intake,
        () -> controller0.getHID().getAButton() > 0.2,  // IN
        () -> controller0.getLeftTriggerAxis() > 0.2    // OUT
    );
}


    public Command driveCommand(){
      return new ThroughTrench(
        drive,
        () -> controller0.getHID().getXButton()); 
    }
}
