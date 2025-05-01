package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.TrackAndScore.Side;
import frc.robot.commands.tests.TestDriveToRelativePose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Placer;

public class Autos {
  
  private static Elevator elevator = Elevator.getInstance();
  private static Placer placer = Placer.getInstance();

  public static Command Reef1(
    double height,
    Side side
  ) {
    return new ParallelCommandGroup(
      new TestDriveToRelativePose(
        () -> new Translation2d(-Units.feetToMeters(4.0), 0),
        () -> Rotation2d.fromDegrees(179),
        true,
        4.0
      ),
      new InstantCommand(() -> elevator.setGoal(height))
    ).andThen(new SequentialCommandGroup(
      new WaitCommand(0.6),
      new TrackAndScore(() -> side),
      new InstantCommand(() -> {
        placer.spitCoral();
      }),
      new WaitCommand(1.0),
      new InstantCommand(() -> {
        placer.stop();
      }),
      new InstantCommand(() -> elevator.drop())
    ));
  }

  public static Command Reef2(
    double height,
    Side side
  ) {
    return new ParallelCommandGroup(
      new TestDriveToRelativePose(
        () -> new Translation2d(-Units.feetToMeters(6.0), Units.inchesToMeters(3.0)),
        () -> Rotation2d.fromDegrees(120.0)
      ),
      new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT))
    ).andThen(
      new SequentialCommandGroup(
        new WaitCommand(0.1)
          .andThen(new InstantCommand(() -> elevator.setGoal(height))),
        new TrackAndScore(() -> side),
        new WaitCommand(0.5),
        new InstantCommand(() -> {
          placer.spitCoral();
        }),
        new WaitCommand(1.0),
        new InstantCommand(() -> {
          placer.stop();
        }),
        new InstantCommand(() -> elevator.setGoal(Elevator.INTAKE_HEIGHT)),
        new TestDriveToRelativePose(
          () -> new Translation2d(-Units.feetToMeters(1.0), 0.0),
          () -> Rotation2d.fromDegrees(120.0)
        ),
        new ParallelCommandGroup(
          new TestDriveToRelativePose(
            () -> new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(14.0)),
            () -> Rotation2d.fromDegrees(55),
            true,
            3.0
          ),
          new SequentialCommandGroup(
            new InstantCommand(() -> LEDs.set(LEDs.getDefaultColor())),
            new RunCommand(placer::intake, placer)
              .until(() -> {
                if(!elevator.isGoingToIntake()) return true;
                if(placer.hasCoral()) return true;
  
                return false;
              }),
            new InstantCommand(placer::stop, placer)
              .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT))),
            new InstantCommand(() -> LEDs.set(LEDs.WHITE))
          )
        ),
        new TestDriveToRelativePose(
          () -> new Translation2d(Units.feetToMeters(9.0), -Units.feetToMeters(4.0)),
          () -> Rotation2d.fromDegrees(55),
          true
        )
      )
    );
  }

  public static Command Reef6(
    double height,
    Side side
  ) {
    return new ParallelCommandGroup(
      new TestDriveToRelativePose(
        () -> new Translation2d(-Units.feetToMeters(6.0), -Units.inchesToMeters(3.0)),
        () -> Rotation2d.fromDegrees(-120.0)
      ),
      new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT))
    ).andThen(
      new SequentialCommandGroup(
        new WaitCommand(0.1)
          .andThen(new InstantCommand(() -> elevator.setGoal(height))),
        new TrackAndScore(() -> side),
        new WaitCommand(0.5),
        new InstantCommand(() -> {
          placer.spitCoral();
        }),
        new WaitCommand(1.0),
        new InstantCommand(() -> {
          placer.stop();
        }),
        new InstantCommand(() -> elevator.setGoal(Elevator.INTAKE_HEIGHT)),
        new TestDriveToRelativePose(
          () -> new Translation2d(-Units.feetToMeters(1.0), 0.0),
          () -> Rotation2d.fromDegrees(-120.0)
        ),
        new ParallelCommandGroup(
          new TestDriveToRelativePose(
            () -> new Translation2d(Units.feetToMeters(2.5), -Units.feetToMeters(14.0)),
            () -> Rotation2d.fromDegrees(-55),
            true,
            3.0
          ),
          new SequentialCommandGroup(
            new InstantCommand(() -> LEDs.set(LEDs.getDefaultColor())),
            new RunCommand(placer::intake, placer)
              .until(() -> {
                if(!elevator.isGoingToIntake()) return true;
                if(placer.hasCoral()) return true;
  
                return false;
              }),
            new InstantCommand(placer::stop, placer)
              .andThen(new InstantCommand(() -> elevator.setGoal(Elevator.LOW_HEIGHT))),
            new InstantCommand(() -> LEDs.set(LEDs.WHITE))
          )
        ),
        new TestDriveToRelativePose(
          () -> new Translation2d(Units.feetToMeters(9.0), Units.feetToMeters(4.0)),
          () -> Rotation2d.fromDegrees(55),
          true
        )
      )
    );
  }
}
