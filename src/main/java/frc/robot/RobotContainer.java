package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    // public static boolean autoEnabled =false;
    
    SendableChooser<Command> Chooser=new SendableChooser<>();
    //Subsystems

    private final Swerve swerve = new Swerve();
    public static final Intake intake = new Intake();
    public static final Arm arm = new Arm();
    private final Gripper grip = new Gripper();
    private final ElementSelector elementSelected = new ElementSelector();

    //Controllers

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    //Drive Controls and Buttons

    private final JoystickButton ZeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton RobotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton IntakePosition = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton RetractButtoon = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton OutputPosition = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton DropGamePieces = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton PrecisionButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton BoostButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton Docking = new JoystickButton(driver,XboxController.Button.kStart.value);
    private final JoystickButton Resting = new JoystickButton(driver,XboxController.Button.kBack.value);
    public static double speed = 0.7;

    //Operator Controls

    private final JoystickButton ConeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CubeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton LowHybrid = new JoystickButton(operator, XboxController.Button.kX.value);
    private final POVButton TestingAuto = new POVButton(operator, 0);
    private final JoystickButton HybridPosition = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton HighPosition = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton MediumPosition = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton ManualGripperUp = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton ManualGripperDown = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final POVButton RetractArm = new POVButton(driver , 180);

    public RobotContainer() {
        Chooser.addOption("Cone+CubePick+Dock",new AutoConeCubePickDock(swerve, intake, arm, grip));
        Chooser.addOption("Cube+CubePick+Dock", new AutoCubeCubePickDock(swerve, intake));
        Chooser.addOption("Cone+HybridShoot+CubePick",new AutoConeCubeHybridCubeIntake(swerve, intake, arm, grip));
        Chooser.addOption("Cube+HybridShoot+CubePick",new AutoCubeCubeHybridCUbeIntake(swerve, intake, arm, grip));
        Chooser.addOption("BarrierCube+CubePick+Dock",new AutoBarrierCubeDock(swerve, intake));
        Chooser.addOption("BarrierCube+CubeHybrid+CubePick", new AutoBarrierCubeCubeIntake(swerve, intake));
        Chooser.addOption("PreloadConeDock", new AutoPreloadConeDock(swerve, intake, arm, grip));
        Chooser.addOption("PreloadCubeDock", new AutoPreloadCubeDock(swerve, intake));
        Shuffleboard.getTab("Autonomus").add(Chooser);
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(1)*speed, 
                () -> -driver.getRawAxis(0)*speed, 
                () -> -driver.getRawAxis(4)*speed,
                // () -> RobotCentric.getAsBoolean()
                ()-> false
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //driver
        ZeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        IntakePosition.onTrue(new frc.robot.commands.IntakePosition(intake, elementSelected,arm,grip));
        // RetractButtoon.onTrue(new frc.robot.commands.ReintakeSubstation(arm, intake, grip));
        OutputPosition.onTrue(new frc.robot.commands.OutputPosition(intake, elementSelected,arm,grip));
        DropGamePieces.onTrue(new frc.robot.commands.DropGamePieces(intake, elementSelected,arm,grip));
        PrecisionButton.whileTrue(new PrecisionCommand(0.5));
        BoostButton.whileTrue(new PrecisionCommand(0.95));

        //operator
        ConeButton.toggleOnTrue(new InstantCommand(()->this.elementSelected.mode(false)));
        CubeButton.toggleOnTrue(new InstantCommand(()->this.elementSelected.mode(true)));
        // GroundPosition.onTrue(new frc.robot.commands.GroundPosition(intake, elementSelected,arm));
        // SubstationPosition.onTrue(new frc.robot.commands.SubstationPosition(arm, elementSelected));
        HybridPosition.onTrue(new frc.robot.commands.OutputHybridPosition(intake, elementSelected));
        MediumPosition.onTrue(new frc.robot.commands.OutputMediumPosition(intake, elementSelected,arm));
        HighPosition.onTrue(new frc.robot.commands.OutputHighPosition(intake, elementSelected,arm));
        // TestingAuto.onTrue(getAutonomousCommand());
        Resting.onTrue(new frc.robot.commands.DockBalanceRest(swerve));
        Docking.onTrue(new SequentialCommandGroup(new DockBalance(swerve), new DockBalanceRest(swerve)));
        ManualGripperUp.onTrue(new frc.robot.commands.ManualGripper(arm, 95));
        ManualGripperDown.onTrue(new frc.robot.commands.ManualGripper(arm, 135));
        LowHybrid.onTrue(new frc.robot.commands.OutputHybridClosePosition(intake, elementSelected));
        RetractArm.onTrue(new frc.robot.commands.ReintakeSubstation(arm, intake, grip));
    }
    
    public Command getAutonomousCommand() {
        return Chooser.getSelected();
    }
}