package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  public final Joystick driver = new Joystick(0);
  private final Joystick panel = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton lockOn = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final POVButton intakeButton = new POVButton(driver, 0);
  private final POVButton outtakeButton = new POVButton(driver, 180);

  private final JoystickButton clawIntake = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton clawOuttake = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton clawRaise = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton clawLower = new JoystickButton(driver, XboxController.Button.kA.value);

  /* Panel Buttons */
  
  private final JoystickButton maxHeight = new JoystickButton(panel, 10);
  private final JoystickButton midHeight = new JoystickButton(panel, 9);
  private final JoystickButton lowHeight = new JoystickButton(panel, 8);
  private final JoystickButton pickupHeight = new JoystickButton(panel, 7);
  private final JoystickButton lowCoral = new JoystickButton(panel, 12);
  private final JoystickButton dropElevator = new JoystickButton(panel, 7);

  // private final POVButton angleForward = new POVButton(driver, 0);
  // private final POVButton angleLeft = new POVButton(driver, 90);
  // private final POVButton angleBackward = new POVButton(driver, 180);
  // private final POVButton angleRight = new POVButton(driver, 270);

  public final JoystickButton elevatorUp = new JoystickButton(panel, 13);
  public final JoystickButton elevatorDown = new JoystickButton(panel, 14);

  /* Subsystems */
  public final Elevator elevator = new Elevator();
  public final Placer placer = new Placer();
  public final Claw claw = new Claw();
  // public final Music music = new Music();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Swerve.getInstance().setDefaultCommand(
      new TeleopSwerve(
        Swerve.getInstance(),
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
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
    /* Driver Buttons */
    // zeroGyro.onTrue(new InstantCommand(() -> Swerve.getInstance().zeroHeading()));

    elevatorUp
      .onTrue(new InstantCommand(() -> elevator.move(3)))
      .onFalse(new InstantCommand(elevator::drop));
    
    elevatorDown
      .onTrue(new InstantCommand(() -> elevator.move(-1.25)))
      .onFalse(new InstantCommand(elevator::drop));
    
    lowHeight
      .onTrue(new InstantCommand(() -> elevator.setGoal(1.0)));

    maxHeight.onTrue(new InstantCommand(() -> elevator.setGoal(4.2414)));
    midHeight.onTrue(new InstantCommand(() -> elevator.setGoal(2.7495)));
    lowHeight.onTrue(new InstantCommand(() -> elevator.setGoal(1.6829)));
    lowCoral.onTrue(new InstantCommand(() -> elevator.setGoal(1.226)));
    pickupHeight.onTrue(new InstantCommand(() -> elevator.setGoal(0.76928)));
    
    intakeButton
      .onTrue(
        new RunCommand(placer::intake, placer)
          .until(placer::hasCoral)
          .andThen(
            new InstantCommand(placer::stop, placer)
          )
      )
      .onFalse(new InstantCommand(placer::stop, placer));

    outtakeButton
      .onTrue(new InstantCommand(placer::outtake))
      .onFalse(new InstantCommand(placer::stop));
    
    clawIntake
      .onTrue(new InstantCommand(claw::intake))
      .onFalse(new InstantCommand(claw::stopIntake));
    
    clawOuttake
      .onTrue(new InstantCommand(claw::outtake))
      .onFalse(new InstantCommand(claw::stopIntake));
    
    clawRaise
      .onTrue(new InstantCommand(claw::raiseClaw))
      .onFalse(new InstantCommand(claw::stopClawPosition));
    
    clawLower
      .onTrue(new InstantCommand(claw::lowerClaw))
      .onFalse(new InstantCommand(claw::stopClawPosition));
    
    lockOn.onTrue(new LockOnTag());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return new exampleAuto(s_Swerve);
  // }
}