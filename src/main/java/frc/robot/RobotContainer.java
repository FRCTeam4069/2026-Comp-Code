
package frc.robot;

import frc.robot.commands.AlignNeg90;
import frc.robot.commands.AutoAlignAutoCommand;
import frc.robot.commands.AutoAlignInfinite;
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
import frc.robot.commands.autos.RedLeftBump;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShootWithTimeout;
import frc.robot.commands.DriveToShootPosition;
import frc.robot.commands.autos.RedLeftTrench;
import frc.robot.commands.autos.RedLeftTrenchV2;
import frc.robot.commands.autos.RedLeftTrenchV3;
import frc.robot.commands.autos.RedMiddle;


public class RobotContainer {

   public final SwerveDrivetrain drive = new SwerveDrivetrain();
   public final IntakeSubsystem intake = new IntakeSubsystem();
   public final PivotSubsystem pivot = new PivotSubsystem();
   public final FeederSubsystem feeder = new FeederSubsystem();
   public final HopperSubsystem hopper = new HopperSubsystem();

   public final TestSubsystem testSubsystem = new TestSubsystem();

   private final CommandXboxController controller0 = new CommandXboxController(0);
   private final CommandXboxController controller1 = new CommandXboxController(1);

   public final ShooterController shooter = new ShooterController();

   public final ShootWithTimeout shootWithTimeout = new ShootWithTimeout(shooter, feeder, hopper, pivot);
   private final AlignNeg90 alignNeg90 = new AlignNeg90(drive);

   private final AutoAlignAutoCommand autoAlignAutoCommand = new AutoAlignAutoCommand(drive);
   public final AutoAlignInfinite autoAlignInfinite = new AutoAlignInfinite(drive);

   private String autoName;

   private final SendableChooser<Command> autoChooser = new SendableChooser<>();

   // addSysIdCommands()

   private final CommandXboxController m_driverController = new CommandXboxController(
         OperatorConstants.kDriverControllerPort);

   public RobotContainer() {

      registerAutoCommands();

      autoName = "";

      // autoChooser = AutoBuilder.buildAutoChooser();

      // addSysIdCommands()

      // autoChooser.addOption("Blue One Cycle Left", new PathPlannerAuto("Blue One
      // Cycle Left"));
      // autoChooser.addOption("Red One Cycle Left", new PathPlannerAuto("Red One
      // Cycle Left"));

      // autoChooser.addOption("Blue 2 Cycle Left", new PathPlannerAuto("Blue 2 Cycle
      // Left"));
      // autoChooser.addOption("Blue 2 Cycle Right", new PathPlannerAuto("Blue 2 Cycle
      // Right"));

      // autoChooser.addOption("Red 2 Cycle Right", new PathPlannerAuto("Red 2 Cycle
      // Right"));
      // autoChooser.addOption("Red 2 Cycle Left", new PathPlannerAuto("Red 2 Cycle
      // Left"));

      // autoChooser.addOption("HP Blue Auto", new PathPlannerAuto("HP Blue Auto"));
      // autoChooser.addOption("HP Red Auto", new PathPlannerAuto("HP Red Auto"));

      // autoChooser.addOption("Red Leave Middle", new PathPlannerAuto("Red Leave
      // Middle"));
      // autoChooser.addOption("Blue Leave Middle", new PathPlannerAuto("Blue Leave
      // Middle"));

      // autoChooser.addOption("Blue Middle Preload Shoot", new PathPlannerAuto("Blue
      // Middle Preload Shoot"));
      // autoChooser.addOption("Red Middle Preload Shoot", new PathPlannerAuto("Red
      // Middle Preload Shoot"));

      // autoChooser.addOption("Red Test Auto", new RedTestAuto(drive, feeder, hopper,
      // intake, shooter, pivot));
      autoChooser.addOption("Red Left Bump", new RedLeftBump(drive, feeder, hopper, intake, shooter, pivot));
      autoChooser.addOption("Red Left Trench", new RedLeftTrench(drive, feeder, hopper, intake, shooter, pivot));
      autoChooser.addOption("Red Left V2", new RedLeftTrenchV2(drive, feeder, hopper, intake, shooter, pivot));
      autoChooser.addOption("Red Left V3", new RedLeftTrenchV3(drive, feeder, hopper, intake, shooter, pivot));
      autoChooser.addOption("Red Middle", new RedMiddle(drive, feeder, hopper, intake, shooter, pivot));


      SmartDashboard.putData("Auto Chooser", autoChooser);

      configureBindings();

   }

   /**
    * Use this method to define your trigger->command mappings. Triggers can be
    * created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
    * an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
    * {@link
    * CommandXboxController
    * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or
    * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
   private void configureBindings() {
      // new Trigger(exampleSubsystem::exampleCondition)
      // .onTrue(new ExampleCommand(exampleSubsystem));

      // m_driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
      controller0.povLeft()
            .onTrue(new DriveToShootPosition(drive, DriveToShootPosition.ClimbTarget.LEFT))
            .onFalse(drive.stopOnceCommand());
      controller0.povUp()
            .onTrue(new DriveToShootPosition(drive, DriveToShootPosition.ClimbTarget.CENTER))
            .onFalse(drive.stopOnceCommand());
      controller0.povRight()
            .onTrue(new DriveToShootPosition(drive, DriveToShootPosition.ClimbTarget.RIGHT))
            .onFalse(drive.stopOnceCommand());
      controller0.povDown()
            .onTrue(new DriveToShootPosition(drive, DriveToShootPosition.ClimbTarget.UP))
            .onFalse(drive.stopOnceCommand());

   }

   private void registerAutoCommands() {

      NamedCommands.registerCommand("intakeOn", intake.intakeOn());
      NamedCommands.registerCommand("intakeOff", intake.intakeOff());

      NamedCommands.registerCommand("intakeDown", pivot.intakeDown());
      NamedCommands.registerCommand("intakeUp", pivot.intakeUp());

      NamedCommands.registerCommand("shoot", shootWithTimeout);
      NamedCommands.registerCommand("stop drivetrain", drive.stopCommand());
      NamedCommands.registerCommand("align negative 90", alignNeg90);

      NamedCommands.registerCommand("auto align", autoAlignAutoCommand);
      NamedCommands.registerCommand("stop shooter", shooter.stopShooterCommand());
      NamedCommands.registerCommand("auto align infinite", autoAlignInfinite);

   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      Command selectedCommand = autoChooser.getSelected();
      if (selectedCommand != null)
         autoName = selectedCommand.getName();
      return selectedCommand;
   }

   public String getAutonomousName() {
      return autoName;
   }

   public Command defaultDriveCommand() {
      return new FieldCentricDrive(
            drive,
            () -> -controller0.getLeftY(), // drive
            () -> -controller0.getLeftX(), // strafe
            () -> -controller0.getRightX(), // rotation
            () -> controller0.getHID().getRightBumperButton(), // autoalign
            () -> controller0.getHID().getStartButton(), // reset odometry
            () -> false, // lock closest
            () -> false, // lock heading
            () -> false, // miss walls
            () -> controller0.getHID().getBackButton(), //disable 
            () -> controller0.getRightY(), // snap modules
            () -> controller0.getHID().getXButton(), // left
            () -> controller0.getHID().getBButton(), // right
            () -> controller0.getHID().getYButton(), // front
            () -> controller0.getHID().getAButton(), // back
            () -> controller0.getHID().getLeftBumperButton());// back

   }

   public Command defaultIntakeCommand() {
      return new IntakeCommand(
            intake,
            pivot,
            () -> controller0.getHID().getRightTriggerAxis(), // In //ON DRIVE 1
            () -> controller0.getHID().getLeftTriggerAxis(), // out //B for everything out //ON DRIVEr 1
            () -> controller1.getHID().getRightTriggerAxis(),
            () -> controller1.getHID().getLeftTriggerAxis(),
            () -> controller1.getHID().getPOV() == 180, // up
            () -> controller1.getHID().getPOV() == 0 // down

      );
   }

   public Command defaultShooterCommand() {
      return new ShooterCommand(
            shooter,
            feeder,
            hopper,
            () -> controller1.getHID().getAButton(), // shoot
            () -> controller1.getHID().getYButton(), // pass
            () -> controller1.getHID().getLeftBumperButton(), // feeder out
            () -> controller1.getHID().getRightBumperButton(), // feeder manual
            () -> controller1.getHID().getXButton(), // manual trench shoot
            () -> controller1.getHID().getBButton()// manual close shoot
      );

   }

}
