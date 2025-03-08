package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class RelativeDriveTest extends Command {
  private Swerve swerve = Swerve.getInstance();

  private static final TrapezoidProfile.Constraints rotationConstraints =
    new TrapezoidProfile.Constraints(1.5, 1);
  private final ProfiledPIDController rotationController =
    new ProfiledPIDController(4, 0.0, 0, rotationConstraints);
  
  private static final TrapezoidProfile.Constraints strafeConstraints =
    new TrapezoidProfile.Constraints(2.0, 1);
  private final ProfiledPIDController strafeController =
    new ProfiledPIDController(4.0, 0.4, 0, strafeConstraints);
  
  private final boolean relativeAngle;
  private Translation2d translationDelta;
  private Rotation2d rotationDelta;
  
  public RelativeDriveTest(
    Translation2d translationDelta,
    Rotation2d rotationDelta,
    boolean relativeAngle
  ) {
    this(translationDelta, rotationDelta, relativeAngle, 0.1);
  }

  private final double positionDeadband;

  public RelativeDriveTest(
    Translation2d translationDelta,
    Rotation2d rotationDelta,
    boolean relativeAngle,
    double positionDeadband
  ) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    this.relativeAngle = relativeAngle;
    this.translationDelta = translationDelta;
    this.rotationDelta = rotationDelta;
    this.positionDeadband = positionDeadband;

    this.addRequirements(Swerve.getInstance());
  }

  private Translation2d startPose;
  private Rotation2d startRotation;
  
  @Override
  public void initialize() {
    startPose = swerve.getPose().getTranslation();
    startRotation = swerve.getGyroYaw();

    var targetAngle = relativeAngle
      ? startRotation.plus(rotationDelta)
      : rotationDelta;

    rotationController.reset(startRotation.getRadians());
    rotationController.setGoal(targetAngle.getRadians());

    strafeController.setGoal(translationDelta.getNorm());
    strafeController.reset(0);
  }

  @Override
  public void execute() {
    var currentPose = swerve.getPose().getTranslation();
    var currentAngle = swerve.getGyroYaw();
    var targetUnit = translationDelta
      .rotateBy(startRotation)
      .div(translationDelta.getNorm());

    var distanceFromStart = startPose.getDistance(currentPose);

    var strafeOutput = strafeController.calculate(distanceFromStart);
    var rotationOutput = rotationController.calculate(currentAngle.getRadians());

    swerve.drive(
      targetUnit.times(strafeOutput),
      rotationOutput,
      true,
      true
    );
  }

  @Override
  public boolean isFinished() {
    var strafeGoal = strafeController.getGoal().position;
    var rotationGoal = rotationController.getGoal().position;

    var currentPose = swerve.getPose().getTranslation();
    var currentAngle = swerve.getGyroYaw();

    var distanceFromStart = startPose.getDistance(currentPose);

    var strafeDifference = strafeGoal - distanceFromStart;
    var rotationDifference = rotationGoal - currentAngle.getRadians();

    var strafeFinished = Math.abs(strafeDifference) < positionDeadband;
    var rotationFinished = Math.abs(rotationDifference) < Radians.convertFrom(1, Degree);

    return strafeFinished && rotationFinished;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
  }
}
