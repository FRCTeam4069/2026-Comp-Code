
package frc.robot;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.commands.ShooterCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShootWithTimeout;


public class RobotContainer {

     

    public static final SwerveDrivetrain drive = new SwerveDrivetrain();
    public static final IntakeSubsystem intake = new IntakeSubsystem();
    public static final PivotSubsystem pivot = new PivotSubsystem();
    public static final FeederSubsystem feeder = new FeederSubsystem();
    public static final HopperSubsystem hopper = new HopperSubsystem();

    public static final TestSubsystem testSubsystem = new TestSubsystem();

    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);

    public static final ShooterController shooter = new ShooterController();

    public static final ShootWithTimeout shootWithTimeout = new ShootWithTimeout(shooter, feeder, hopper, pivot);



    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

       // addSysIdCommands()


    private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

 public RobotContainer() {

      registerAutoCommands();
      // Configure the trigger bindings


     //autoChooser = AutoBuilder.buildAutoChooser();

       // addSysIdCommands()

      autoChooser.addOption("Blue Two Cycle Left ", new PathPlannerAuto("Blue Two Cycle Left"));
      autoChooser.addOption ("Red Two Cycle Left ", new PathPlannerAuto("Red Two Cycle Left"));

       //autoChooser.addOption("Blue 2 Cycle Right", new PathPlannerAuto("Blue 2 Cycle Right"));
      autoChooser.addOption ("HP Blue Auto", new PathPlannerAuto("HP Blue Auto"));
      autoChooser.addOption ("HP Red Auto", new PathPlannerAuto("HP Red Auto"));

      //  autoChooser.addOption("Test", new PathPlannerAuto("Test"));
      //  autoChooser.addOption("test first seg", new PathPlannerAuto("test first seg"));


       SmartDashboard.putData("Auto Chooser", autoChooser);

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

   private void registerAutoCommands(){
      // NamedCommands.registerCommand("shooterArticulate", testSubsystem.driveWithEighteyCommand());
      // NamedCommands.registerCommand("driveStop", testSubsystem.driveStop());

      NamedCommands.registerCommand("intakeOn", intake.intakeOn());
      NamedCommands.registerCommand("intakeOff", intake.intakeOff());

      NamedCommands.registerCommand("intakeDown", pivot.intakeDown());
      NamedCommands.registerCommand ("intakeUp", pivot.intakeUp());

      NamedCommands.registerCommand("shoot", shootWithTimeout);
      NamedCommands.registerCommand("autoRamp", shooter.autoRampCommand());
      NamedCommands.registerCommand("stop drivetrain", drive.stopCommand());


      
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
               () -> -controller0.getLeftY(), //drive
               () -> -controller0.getLeftX(), //strafe
               () -> -controller0.getRightX(), //rotation
               () -> controller0.getHID().getAButton(), //autoalign
               () -> controller0.getHID().getYButton(), //reset odometry
               //() -> controller0.getHID().getXButton(), // through trench
               () -> controller0.getHID().getRightBumperButton()); //lock heading
   }

   public Command defaultIntakeCommand() {
      return new IntakeCommand(
              intake,
              pivot,
              () -> controller1.getHID().getRightTriggerAxis(), // In
              () -> controller1.getHID().getLeftTriggerAxis(),  // out //B for everything out
              () -> controller1.getHID().getPOV() == 180,    // down
              () -> controller1.getHID().getPOV() == 0  // up
      );
  }
  
  public Command defaultShooterCommand() { 
      return new ShooterCommand(
              shooter,
              feeder,
              hopper,
              () -> controller1.getHID().getAButton(), // shoot 
              () -> controller1.getHID().getYButton(),//pass
              () -> controller1.getHID().getLeftBumperButton(),// feeder out
              () -> controller1.getHID().getRightBumperButton(), //feeder manual
              () -> controller1.getHID().getXButton(), // manual trench shoot
              () -> controller1.getHID().getBButton()// manual close shoot
      );

   }

}


