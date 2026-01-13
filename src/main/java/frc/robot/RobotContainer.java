
package frc.robot;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
      public static final SwerveDrivetrain drive = new SwerveDrivetrain();

      private final CommandXboxController controller0 = new CommandXboxController(0);
     // private final CommandXboxController controller1 = new CommandXboxController(1);


  public final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
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
    new Trigger(exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(exampleSubsystem));

    m_driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }
  
    public Command defaultDriveCommand() {
        return new FieldCentricDrive(
                drive,
                () -> -controller0.getLeftY(),
                () -> -controller0.getLeftX(),
                () -> -controller0.getRightX(),
                () -> controller0.getHID().getLeftBumperButton(),
                () -> false,
                // () -> controller0.getHID().getYButton(),
                () -> controller0.getHID().getXButton(),
                () -> controller0.getHID().getBButton(),
                () -> controller0.getHID().getRightBumperButton());
    }

  
}
