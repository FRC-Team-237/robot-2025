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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.TrackReefHorizontally;
import frc.robot.commands.autonomous.TrackReefWithOffset;
import frc.robot.subsystems.*;

public class RobotContainer {
  /* Controllers */
  public final Joystick driver = new Joystick(0);
  public final Joystick panel = new Joystick(1);

  public final Joystick interlink = new Joystick(2);

  /* Drive Controls */
  public final int translationAxis = XboxController.Axis.kLeftY.value;
  public final int strafeAxis = XboxController.Axis.kLeftX.value;
  public final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  public final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton lockOn = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final POVButton intakeButton = new POVButton(driver, 0);
  private final POVButton outtakeButton = new POVButton(driver, 180);

  private final JoystickButton clawIntake = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton clawOuttake = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton clawRaise = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton clawLower = new JoystickButton(driver, XboxController.Button.kA.value);

  private final JoystickButton TEST_horizontalTrack = new JoystickButton(driver, XboxController.Button.kStart.value);

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

    elevatorUp
      .onTrue(new InstantCommand(() -> elevator.move(7)))
      .onFalse(new InstantCommand(elevator::drop));
    
    elevatorDown
      .onTrue(new InstantCommand(() -> elevator.move(-4)))
      .onFalse(new InstantCommand(elevator::drop));
    
    maxHeight.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.MAX_HEIGHT)));
    midHeight.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.MID_HEIGHT)));
    lowHeight.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT)));
    lowAlgae.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_ALGAE_HEIGHT)));
    highAlgae.onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.HIGH_ALGAE_HEIGHT)));

    dropElevator.onTrue(new InstantCommand(() -> elevator.move(-4.0)));
    
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

    intakeButton
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
    lockOn.whileTrue(
      new TrackReefWithOffset(
        new Translation2d(
          -1,
          0
        ),
        Rotation2d.fromDegrees(0)
      ).andThen(
        new TrackReefHorizontally(() -> {
          if(rightCoralSwitch.getAsBoolean()) {
            return TrackReefHorizontally.Side.RIGHT;
          } else {
            return TrackReefHorizontally.Side.LEFT;
          }
        })
      ).andThen(new RelativeDriveTest(
        new Translation2d(
          0.52,
          0
        ),
        Rotation2d.kZero,
        true
      ))
    );

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

    SmartDashboard.putData("AUTO TEST/1", high1Auto);
    SmartDashboard.putData("AUTO TEST/2", high2Auto);
    SmartDashboard.putData("AUTO TEST/3", high6Auto);
  }
  
  Command high1Auto = new RelativeDriveTest(
    new Translation2d(
      -Meters.convertFrom(4.5, Feet),
      -0.3
    ),
    Rotation2d.fromDegrees(165),
    false,
    0.15
  ).withTimeout(4).alongWith(
    new WaitCommand(2.0)
      .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.MAX_HEIGHT)))
  )
  .andThen(new WaitCommand(0.5))
  .andThen(
    new TrackReefWithOffset(
      new Translation2d(
        -1,
        0
      ),
      Rotation2d.fromDegrees(0)
    ).andThen(
      new TrackReefHorizontally(() -> {
        if(rightCoralSwitch.getAsBoolean()) {
          return TrackReefHorizontally.Side.RIGHT;
        } else {
          return TrackReefHorizontally.Side.LEFT;
        }
      })
    )
    .andThen(new WaitCommand(0.5))
    .andThen(
      new TrackReefHorizontally(() -> {
        if(rightCoralSwitch.getAsBoolean()) {
          return TrackReefHorizontally.Side.RIGHT;
        } else {
          return TrackReefHorizontally.Side.LEFT;
        }
      })
    )
    // .andThen(new RelativeDriveTest(
    //   new Translation2d(
    //     0.54,
    //     0
    //   ),
    //   Rotation2d.kZero,
    //   true
    // ))
    .andThen(new InstantCommand(() -> {
      Swerve.getInstance().driveUnsafe(
        new Translation2d(
          0.225,
          0
        ),
        0,
        false,
        true
      );
    }))
    .andThen(new WaitCommand(2.75))
    .andThen(new InstantCommand(() -> Swerve.getInstance().drive(Translation2d.kZero, 0, false, false)))
    .andThen(new InstantCommand(placer::spitCoral))
    .andThen(new WaitCommand(1.0))
    .andThen(new InstantCommand(() -> {
      Swerve.getInstance().drive(
        new Translation2d(-0.25, 0),
        0,
        false,
        false
      );
    }))
    .andThen(new WaitCommand(0.5))
    .andThen(new InstantCommand(() -> elevator.move(-2)))
  );

  Command high2Auto = new RelativeDriveTest(
    new Translation2d(
      -Meters.convertFrom(6.75, Feet),
      -Meters.convertFrom(6, Inches)
    ),
    Rotation2d.fromDegrees(120),
    false,
    0.15
  ).withTimeout(4).alongWith(
    new WaitCommand(2.0)
      .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.MAX_HEIGHT)))
  )
  .andThen(new WaitCommand(0.5))
  .andThen(
    new TrackReefWithOffset(
      new Translation2d(
        -1,
        0
      ),
      Rotation2d.fromDegrees(0)
    ).andThen(
      new TrackReefHorizontally(() -> {
        if(rightCoralSwitch.getAsBoolean()) {
          return TrackReefHorizontally.Side.RIGHT;
        } else {
          return TrackReefHorizontally.Side.LEFT;
        }
      })
    )
    .andThen(new WaitCommand(0.5))
    .andThen(
      new TrackReefHorizontally(() -> {
        if(rightCoralSwitch.getAsBoolean()) {
          return TrackReefHorizontally.Side.RIGHT;
        } else {
          return TrackReefHorizontally.Side.LEFT;
        }
      })
    )
    // .andThen(new RelativeDriveTest(
    //   new Translation2d(
    //     0.54,
    //     0
    //   ),
    //   Rotation2d.kZero,
    //   true
    // ))
    .andThen(new InstantCommand(() -> {
      Swerve.getInstance().driveUnsafe(
        new Translation2d(
          0.225,
          0
        ),
        0,
        false,
        true
      );
    }))
    .andThen(new WaitCommand(2.75))
    .andThen(new InstantCommand(() -> Swerve.getInstance().drive(Translation2d.kZero, 0, false, false)))
    .andThen(new InstantCommand(placer::spitCoral))
  );

  Command high6Auto = new RelativeDriveTest(
    new Translation2d(
      -Meters.convertFrom(6.75, Feet),
      Meters.convertFrom(6, Inches)
    ),
    Rotation2d.fromDegrees(-120),
    false,
    0.15
  ).withTimeout(4).alongWith(
    new WaitCommand(2.0)
      .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.MAX_HEIGHT)))
  )
  .andThen(new WaitCommand(0.5))
  .andThen(
    new TrackReefWithOffset(
      new Translation2d(
        -1,
        0
      ),
      Rotation2d.fromDegrees(0)
    ).andThen(
      new TrackReefHorizontally(() -> {
        if(rightCoralSwitch.getAsBoolean()) {
          return TrackReefHorizontally.Side.RIGHT;
        } else {
          return TrackReefHorizontally.Side.LEFT;
        }
      })
    )
    .andThen(
      new TrackReefHorizontally(() -> {
        if(rightCoralSwitch.getAsBoolean()) {
          return TrackReefHorizontally.Side.RIGHT;
        } else {
          return TrackReefHorizontally.Side.LEFT;
        }
      })
    )
    // .andThen(new RelativeDriveTest(
    //   new Translation2d(
    //     0.54,
    //     0
    //   ),
    //   Rotation2d.kZero,
    //   true
    // ))
    .andThen(new InstantCommand(() -> {
      Swerve.getInstance().driveUnsafe(
        new Translation2d(
          0.225,
          0
        ),
        0,
        false,
        true
      );
    }))
    .andThen(new WaitCommand(2.75))
    .andThen(new InstantCommand(() -> Swerve.getInstance().drive(Translation2d.kZero, 0, false, false)))
    .andThen(new InstantCommand(placer::spitCoral))
  );

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

    if(isOneAuto) return high6Auto;
    if(isTwoAuto) return high1Auto;
    if(isThreeAuto) return high2Auto;

    return new RelativeDriveTest(
      new Translation2d(
        -2.0,
        0.0
      ),
      Rotation2d.kZero,
      true
    );
    
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