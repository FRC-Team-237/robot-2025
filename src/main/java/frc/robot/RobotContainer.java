package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autos;
import frc.robot.autos.TrackAndScore;
import frc.robot.autos.TrackAndScore.Side;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.TrackReefHorizontally;
import frc.robot.commands.autonomous.TrackReefWithOffset;
import frc.robot.subsystems.*;

public class RobotContainer {
  /* Controllers */
  public final CommandXboxController driver = new CommandXboxController(0);
  public final Joystick panel = new Joystick(1);

  public final Joystick interlink = new Joystick(2);

  /* Drive Controls */
  public final int translationAxis = XboxController.Axis.kLeftY.value;
  public final int strafeAxis = XboxController.Axis.kLeftX.value;
  public final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  public final Trigger robotCentric = driver.leftBumper();
  // private final JoystickButton lockOn = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  // private final POVButton outtakeButton = new POVButton(driver, 180);
  private final Trigger lockOn = driver.rightBumper();
  private final Trigger outtakeButton = driver.pov(180);
  // private final Trigger intakeButton = driver.pov(0);
  private final Trigger intakeButton = driver.leftTrigger(0.3);
  private final JoystickButton intakeButton2 = new JoystickButton(interlink, 26);

  // private final JoystickButton clawIntake = new JoystickButton(driver, XboxController.Button.kX.value);
  // private final JoystickButton clawOuttake = new JoystickButton(driver, XboxController.Button.kY.value);
  // private final JoystickButton clawRaise = new JoystickButton(driver, XboxController.Button.kB.value);
  // private final JoystickButton clawLower = new JoystickButton(driver, XboxController.Button.kA.value);
  private final Trigger clawIntake = driver.x();
  private final Trigger clawOuttake = driver.y();
  private final Trigger clawRaise = driver.b();
  private final Trigger clawLower = driver.a();
  
  public final Trigger paradeShoot = driver.back();

  // private final JoystickButton TEST_horizontalTrack = new JoystickButton(driver, XboxController.Button.kStart.value);

  /* Panel Buttons */
  
  // HEIGHTS
  private final JoystickButton maxHeight = new JoystickButton(panel, 10);
  private final JoystickButton midHeight = new JoystickButton(panel, 9);
  private final JoystickButton lowHeight = new JoystickButton(panel, 8);
  private final JoystickButton lowAlgae = new JoystickButton(panel, 12);
  private final JoystickButton highAlgae = new JoystickButton(panel, 11);
  private final JoystickButton dropElevator = new JoystickButton(panel, 7);

  // REEF POSITIONS
  private final JoystickButton farMiddle = new JoystickButton(panel, 1);
  private final JoystickButton farRight = new JoystickButton(panel, 2);
  private final JoystickButton closeRight = new JoystickButton(panel, 3);
  private final JoystickButton closeMiddle = new JoystickButton(panel, 4);
  private final JoystickButton closeLeft = new JoystickButton(panel, 5);
  private final JoystickButton farLeft = new JoystickButton(panel, 6);

  private final JoystickButton angleRight = new JoystickButton(panel, 20);

  private final JoystickButton rightCoralSwitch = new JoystickButton(panel, 15);

  // INTAKE POSITIONS
  private final JoystickButton intakeLeft = new JoystickButton(panel, 18);
  private final JoystickButton intakeRight = new JoystickButton(panel, 19);

  private final JoystickButton algaeArmUp = new JoystickButton(panel, 16);
  private final JoystickButton algaeArmDown = new JoystickButton(panel, 17);

  // MANUAL CONTROL
  public final JoystickButton elevatorUp = new JoystickButton(panel, 13);
  public final JoystickButton elevatorDown = new JoystickButton(panel, 14);

  /* Subsystems */
  public Elevator elevator = Elevator.getInstance();
  public final Placer placer = Placer.getInstance();
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
    // Swerve.getInstance().setDefaultCommand(
    //   new TeleopSwerve(
    //     Swerve.getInstance(),
    //     () -> interlink.getRawAxis(1),
    //     () -> -interlink.getRawAxis(0),
    //     () -> -interlink.getRawAxis(3),
    //     () -> robotCentric.getAsBoolean()
    //   )
    // );

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

    paradeShoot.whileTrue(new RunCommand(() -> {
      placer.paradeShoot();
    }, placer)).onFalse(new InstantCommand(() -> {
      placer.stop();
    }, placer));

    elevatorUp
      .onTrue(new InstantCommand(() -> elevator.move(7)))
      // .onFalse(new InstantCommand(() -> elevator.setGoal(Elevator.getHeight())));
      .onFalse(new InstantCommand(elevator::drop));
    
    elevatorDown
      .onTrue(new InstantCommand(() -> elevator.move(-4)))
      // .onFalse(new InstantCommand(() -> elevator.setGoal(Elevator.getHeight())));
      .onFalse(new InstantCommand(elevator::drop));
    
    maxHeight.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.MAX_HEIGHT)));
    midHeight.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.MID_HEIGHT)));
    lowHeight.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT)));
    lowAlgae.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_ALGAE_HEIGHT)));
    highAlgae.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.HIGH_ALGAE_HEIGHT)));

    dropElevator.onTrue(new InstantCommand(() -> elevator.move(-8.0)));
    
    // @TODO: Test if we can put these in each intake[Side] command to reduce code duplication
    // var intakeUntilHasCoralOrCancel = new RunCommand(placer::intake, placer)
    //   .until(() -> {
    //     // either the elevator should manually move
    //     var r = driver.getRawAxis(rotationAxis);
    //     if(Math.abs(r) > 0.1) {
    //       return true;
    //     }

    //     if(!elevator.isGoingToIntake()) return true;

    //     if(placer.hasCoral()) return true;

    //     return false;
    //   })
    //   // or the placer has the coral
    //   .until(placer::hasCoral)
    //   .andThen(new InstantCommand(() -> LEDs.set(LEDs.WHITE)))
    //   // stop the intake
    //   .andThen(new InstantCommand(placer::stop, placer));

    intakeLeft.onTrue(
      // set the elevator to the intake height
      new InstantCommand(() -> elevator.setGoal(Elevator.INTAKE_HEIGHT))
        .andThen(new InstantCommand(() -> LEDs.set(LEDs.getDefaultColor())))
        // rotate the robot to face the intake
        .andThen(new TurnToAngle(Rotation2d.fromDegrees(-55)))
        // run the intake until...
        .andThen(new RunCommand(placer::intake, placer)
          // either the elevator should manually move
          .until(() -> {
            var r = driver.getRawAxis(rotationAxis);
            if(Math.abs(r) > 0.1) {
              return true;
            }

            if(!elevator.isGoingToIntake()) return true;

            if(placer.hasCoral()) return true;

            return false;
          })
          // or the placer has the coral
          // .until(placer::hasCoral)
          .andThen(new InstantCommand(() -> LEDs.set(LEDs.WHITE)))
          // stop the intake
          .andThen(new InstantCommand(placer::stop, placer))
        )
    );
    
    intakeRight.onTrue(
      // set the elevator to the intake height
      new InstantCommand(() -> elevator.setGoal(Elevator.INTAKE_HEIGHT))
        .andThen(new InstantCommand(() -> LEDs.set(LEDs.getDefaultColor())))
        // rotate the robot to face the intake
        .andThen(new TurnToAngle(Rotation2d.fromDegrees(55)))
        // run the intake until...
        .andThen(new RunCommand(placer::intake, placer)
          // either the elevator should manually move
          .until(() -> {
            var r = driver.getRawAxis(rotationAxis);
            if(Math.abs(r) > 0.1) {
              return true;
            }

            if(!elevator.isGoingToIntake()) return true;

            if(placer.hasCoral()) return true;

            return false;
          })
          // .until(() -> !elevator.isGoingToIntake())
          // // or the placer has the coral
          // .until(placer::hasCoral)
          .andThen(new InstantCommand(() -> LEDs.set(LEDs.WHITE)))
          // stop the intake
          .andThen(new InstantCommand(placer::stop, placer))
        )
    );

    intakeButton.or(intakeButton2)
      .onTrue(new InstantCommand(placer::spitCoral, placer))
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
      .onTrue(new InstantCommand(claw::raiseClaw, claw))
      .onFalse(new InstantCommand(claw::stopClawPosition, claw));
    
    algaeArmUp
      .onTrue(new InstantCommand(claw::raiseClaw, claw))
      .onFalse(new InstantCommand(claw::stopClawPosition, claw));
    
    algaeArmDown
      .onTrue(new InstantCommand(claw::lowerClaw));

    clawLower
      .onTrue(new InstantCommand(claw::lowerClaw));

    // lockOn.whileTrue(
    //   new BigBrainTarget()
    //     .andThen(
    //       new TrackReefHorizontally(() -> {
    //         if(rightCoralSwitch.getAsBoolean()) {
    //           return TrackReefHorizontally.Side.RIGHT;
    //         } else {
    //           return TrackReefHorizontally.Side.LEFT;
    //         }
    //       })
    //     )
    //     .andThen(new RelativeDriveTest(
    //       new Translation2d(
    //         0.7,
    //         0
    //       ),
    //       Rotation2d.kZero,
    //       true
    //     ))
    // );
    // lockOn.whileTrue(
    //   new TrackReefWithOffset(
    //     new Translation2d(
    //       -1,
    //       0
    //     ),
    //     Rotation2d.fromDegrees(0)
    //   ).andThen(
    //     new TrackReefHorizontally(() -> {
    //       if(rightCoralSwitch.getAsBoolean()) {
    //         return TrackReefHorizontally.Side.RIGHT;
    //       } else {
    //         return TrackReefHorizontally.Side.LEFT;
    //       }
    //     })
    //   ).andThen(new RelativeDriveTest(
    //     new Translation2d(
    //       0.52,
    //       0
    //     ),
    //     Rotation2d.kZero,
    //     true
    //   ))
    // );

    farMiddle.onTrue(new TurnToAngle(Rotation2d.fromDegrees(180)));
    farRight.onTrue(new TurnToAngle(Rotation2d.fromDegrees(120)));
    closeRight.onTrue(new TurnToAngle(Rotation2d.fromDegrees(60)));
    closeMiddle.onTrue(new TurnToAngle(Rotation2d.fromDegrees(0)));
    closeLeft.onTrue(new TurnToAngle(Rotation2d.fromDegrees(-60)));
    farLeft.onTrue(new TurnToAngle(Rotation2d.fromDegrees(-120)));
    angleRight.onTrue(new TurnToAngle(Rotation2d.fromDegrees(-90)));

    // SmartDashboard.putData(
    //   "Auto Testing/2",
    //   new TurnToAngle(Rotation2d.k180deg).andThen(
    //     new RelativeDriveTest(
    //       new Translation2d(
    //         Meters.convertFrom(6.75, Feet),
    //         -Meters.convertFrom(6, Inches)
    //       ),
    //       Rotation2d.fromDegrees(-60),
    //       true
    //     )
    //     .andThen(new WaitCommand(1.0))
    //     .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT)))
    //     .andThen(
    //       new TrackReefWithOffset(
    //         new Translation2d(
    //           -0.485,
    //           -Meters.convertFrom(8.0, Inches)
    //         ),
    //         Rotation2d.fromDegrees(0)
    //       )
    //     )
    //     .andThen(new WaitCommand(0.5))
    //     .andThen(new InstantCommand(placer::spitCoral))
    //   )
    // );

    // SmartDashboard.putData("AUTO TEST/1", high1Auto);
    // SmartDashboard.putData("AUTO TEST/2", high2Auto);
    // SmartDashboard.putData("AUTO TEST/3", high6Auto);

    // SmartDashboard.putData(
    //   "AUTO TEST/DRIVE RELATIVE",
    //   new TestDriveToRelativePose(
    //     () -> new Translation2d(2.0, 0.0),
    //     () -> Rotation2d.fromDegrees(180)
    //   )
    // );

    SmartDashboard.putData(
      "AUTO TEST/Reef 2",
      Autos.Reef2(Elevator.MAX_HEIGHT, Side.LEFT)
    );

    // SmartDashboard.putData(
    //   "AUTO TEST/TRACK TARGET",
    //   new TestDriveToTargetWithOffset(
    //     () -> new Translation2d(-1.0, 0.0),
    //     () -> Rotation2d.kZero
    //   )
    // );

    // lockOn.whileTrue(
    //   new TestDriveToTargetWithOffset(
    //     // () -> new Translation2d(-Units.inchesToMeters(28), 0.0),
    //     () -> {
    //       var horizontal = rightCoralSwitch.getAsBoolean() ? Units.inchesToMeters(6) : Units.inchesToMeters(-6);
    //       return new Translation2d(-Units.inchesToMeters(28), horizontal);
    //     },
    //     () -> Rotation2d.kZero
    //   )
    // );

    lockOn.whileTrue(
      new TrackAndScore(
        () -> rightCoralSwitch.getAsBoolean() ? Side.RIGHT : Side.LEFT
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var isOneAuto = panel.getRawButton(22);
    var isTwoAuto = panel.getRawButton(23);
    var isThreeAuto = panel.getRawButton(24);
    var isFourAuto = panel.getRawButton(25);

    if(isThreeAuto) return Autos.Reef2(Elevator.MAX_HEIGHT, Side.LEFT);
    if(isOneAuto) return Autos.Reef6(Elevator.MAX_HEIGHT, Side.RIGHT);
    if(isTwoAuto) return Autos.Reef1(Elevator.MAX_HEIGHT, Side.RIGHT);

    // return Autos.Reef2(Elevator.MAX_HEIGHT, Side.LEFT);
    return new InstantCommand();

    // if(isOneAuto) return high1Auto;
    // if(isTwoAuto) return Autos.Reef2(Elevator.MAX_HEIGHT, Side.LEFT);

    // return new InstantCommand();

    // if(isOneAuto) return high6Auto;
    // if(isTwoAuto) return high1Auto;
    // if(isThreeAuto) return high2Auto;

    // return new RelativeDriveTest(
    //   new Translation2d(
    //     -2.0,
    //     0.0
    //   ),
    //   Rotation2d.kZero,
    //   true
    // );
    
    // return new TurnToAngle(Rotation2d.k180deg).andThen(
    //   new RelativeDriveTest(
    //     new Translation2d(
    //       Meters.convertFrom(6.75, Feet),
    //       -Meters.convertFrom(6, Inches)
    //     ),
    //     Rotation2d.fromDegrees(-60),
    //     true
    //   )
    // )
    // .andThen(new WaitCommand(0.75))
    // .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT)))
    // .andThen(
    //   new TrackReefWithOffset(
    //     new Translation2d(
    //       -0.5,
    //       Meters.convertFrom(10.5, Inches)
    //     ),
    //     Rotation2d.fromDegrees(0)
    //   )
    // )
    // .andThen(new InstantCommand(placer::spitCoral));
  }
}