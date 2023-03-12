package frc.robot;

import javax.print.attribute.standard.JobOriginatingUserName;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.PoseEstimator;
    
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick Joy2 = new Joystick(1);
    // public PhotonCamera photonCamera = new PhotonCamera("AprilTag");

    // /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);

    public static double speed = 0.7;
    // private final JoystickButton IntakeUp = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton IntakeDown = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton IntakeInside = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton IntakeOutside = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton SolGripperOutside = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton SolGripperInside = new JoystickButton(driver, XboxController.Button.kBack.value);
    // private final POVButton HOMEPosition = new POVButton(driver, 180);
    // private final POVButton INTAKEPosition  = new POVButton(driver, 90);
    // private final JoystickButton Solenoid_On = new JoystickButton(Joy2, XboxController.Button.kStart.value);
    // private final JoystickButton Solenoid_OFF = new JoystickButton(Joy2, XboxController.Button.kBack.value);
   
    // private final POVButton IntakeJoy2 = new POVButton(Joy2, 270);
    // // private final POVButton MediumPosition = new POVButton(Joy2%, 90);
    // private final POVButton ThrowPosition= new POVButton(Joy2, 0);
    // private final POVButton ThrowPositionHybrid= new POVButton(driver, 0);
    // private final POVButton throwPower  = new POVButton(Joy2, 180);

    
    private final JoystickButton OutputPosition = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton DropGamePieces = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton GroundPosition = new POVButton(Joy2, 180);
    private final JoystickButton HighPosition = new JoystickButton(Joy2, XboxController.Button.kY.value);
    private final JoystickButton MediumPosition = new JoystickButton(Joy2, XboxController.Button.kB.value);
    private final JoystickButton HybridPosition = new JoystickButton(Joy2, XboxController.Button.kA.value);
    private final POVButton SubstationPosition = new POVButton(Joy2, 0);
    private final JoystickButton ConeButton = new JoystickButton(Joy2, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CubeButton = new JoystickButton(Joy2, XboxController.Button.kRightBumper.value);
    private final JoystickButton IntakePosition = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton precisionButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton boostButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final POVButton GripperUp = new POVButton(driver, 90);
    // private final POVButton GripperDown = new POVButton(driver, 270);
    // private final JoystickButton Docking = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton Resting = new JoystickButton(driver, XboxController.Button.kX.value);


    // private final POVButton MediumPosition = new POVButton(driver, 0);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public static Arm arm = new Arm();
    public static Intake intake = new Intake();
    private final InputMode inputMode = new InputMode();
    private final Gripper grip = new Gripper();
    // private final PoseEstimator PoseEstimator = new PoseEstimator(photonCamera, s_Swerve);

 
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis)*speed, 
                () -> -driver.getRawAxis(strafeAxis)*speed, 
                () -> -driver.getRawAxis(rotationAxis)*speed, 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ConeButton.toggleOnTrue(new InstantCommand(()->this.inputMode.mode(false)));
        CubeButton.toggleOnTrue(new InstantCommand(()->this.inputMode.mode(true)));
        GroundPosition.onTrue(new frc.robot.commands.GroundPosition(intake, inputMode,arm));
        HighPosition.onTrue(new frc.robot.commands.HighPosition(intake, inputMode,arm));
        HybridPosition.onTrue(new frc.robot.commands.HybridPosition(intake, inputMode));
        MediumPosition.onTrue(new frc.robot.commands.MediumPosition(intake, inputMode,arm));
        OutputPosition.onTrue(new frc.robot.commands.OutputPosition(intake, inputMode,arm,grip));
        DropGamePieces.onTrue(new frc.robot.commands.DropGamePieces(intake, inputMode,arm,grip));
        IntakePosition.onTrue(new frc.robot.commands.IntakePosition(intake, inputMode,arm,grip));
        SubstationPosition.onTrue(new frc.robot.commands.SubstationPosition(arm, inputMode));
        precisionButton.whileTrue(new PrecisionCommand(0.5));
        boostButton.whileTrue(new PrecisionCommand(0.95));
        // GripperUp.onTrue(new InstantCommand(()-> arm.manualGripperUpOrDown(2)));
        // GripperDown.onTrue(new InstantCommand(()-> arm.manualGripperUpOrDown(-2)));
        // Docking.onTrue(new SequentialCommandGroup(new DockBalance3(s_Swerve),new DockBalanceRest(s_Swerve)));
        // Resting.onTrue(new DockBalanceRest(s_Swerve));

        // SubstationPo sition.onTrue(new frc.robot.commands.GroundPosition(intake, inputMode));
        /* Driver Buttons */
        
        // IntakeJoy2.whileTrue(new InstantCommand(()-> intake.LifterDegrees(117)));
        // ThrowPosition.whileTrue(new InstantCommand(()-> intake.LifterDegrees(75)));
        // throwPower.whileTrue(new InstantCommand(() -> intake.OuttakeCube(-1)));
        // throwPower.whileFalse(new InstantCommand(() -> intake.OuttakeCube(0)));
        // // IntakeDown.whileTrue(new InstantCommand(()-> intake.ThrowPosition(000)));
        // IntakeUp.whileTrue(new frc.robot.commands.ManualIntakeUp(this.intake));
        // IntakeDown.whileTrue(new frc.robot.commands.ManualIntakeDown(this.intake));
        // INTAKEPosition.whileTrue(new InstantCommand(() -> arm.IntakePosition()));
        // HOMEPosition.whileTrue(new InstantCommand(() -> arm.HomePostion()));
        // IntakeInside.whileTrue(new InstantCommand(() -> intake.OuttakeCube(-0.38)));
        // IntakeOutside.whileTrue(new InstantCommand(() -> intake.OuttakeCube(-0.25)));
        // IntakeInside.whileFalse(new InstantCommand(() -> intake.OuttakeCube(0)));
        // IntakeOutside.whileFalse(new InstantCommand(() -> intake.OuttakeCube(0)));

        // // SolGripperOutside.whileTrue(new InstantCommand(()-> intake.ThrowPosition(211000)));
        // SolGripperInside.toggleOnTrue(new frc.robot.commands.IntakeOnBeamBreaked(this.intake));
        // // SolGripperInside.whileFalse(new InstantCommand(()->intake.IntakeCube(0)));
        // // MediumPosition.whileTrue(new InstantCommand(()-> arm.MediumPosition()));
        // Solenoid_On.onTrue(new InstantCommand(()-> arm.Grip(false)));
        // Solenoid_OFF.onTrue(new InstantCommand(()-> arm.Grip(true)));
        // MoveArmUp.whileTrue(new InstantCommand(() -> arm.MoveArm(2)));
        // MoveArmDown.whileTrue(new InstantCommand(() -> arm.MoveArm(-2)));
        // MoveElbowUp.whileTrue(new InstantCommand(() -> arm.MoveElbow(2)));
        // MoveElbowDown.whileTrue(new InstantCommand(() -> arm.MoveElbow(-2)));
        // IntakeCOne.whileTrue((new InstantCommand(()-> arm.IntakeCOne())));
        // MediumPosition.whileTrue(new InstantCommand(()-> arm.MediumPosition()));
        
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    }
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new ConeAuto(s_Swerve,intake,arm,grip);
        // return null;
    }
}
