package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.tests.TestDriveToRelativePose;
import frc.robot.commands.tests.TestDriveToTargetWithOffset;
import frc.robot.subsystems.Placer;
import frc.robot.subsystems.Swerve;

public class TrackAndScore extends SequentialCommandGroup {
  
  public enum Side {
    LEFT,
    RIGHT
  }

  private Rotation2d angleAtReef;
  private Swerve swerve = Swerve.getInstance();
  private Placer placer = Placer.getInstance();

  public TrackAndScore(
    Supplier<Side> side
  ) {

    addRequirements(
      swerve,
      placer
    );

    addCommands(
      new InstantCommand(() -> swerve.clearAngleSetpoint()),
      new TestDriveToTargetWithOffset(
        () -> {
          var horizontal = side.get() == Side.LEFT ? Units.inchesToMeters(6.25) : -Units.inchesToMeters(6.25);
          return new Translation2d(-Units.inchesToMeters(24), horizontal);
        },
        () -> Rotation2d.kZero
      ).andThen(
        new InstantCommand(() -> {
          angleAtReef = swerve.getPose().getRotation();
        })
      ),
      new TestDriveToRelativePose(
        () -> new Translation2d(Units.inchesToMeters(10), 0),
        () -> angleAtReef
      ).withTimeout(1.5)
      // ,
      // new InstantCommand(() -> {
      //   placer.spitCoral();
      // }),
      // new WaitCommand(0.5),
      // new InstantCommand(() -> {
      //   placer.stop();
      // })
    );
  }
}
