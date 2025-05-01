package frc.robot.commands.tests;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TestDriveToRelativePose extends Command {
  
  private Swerve swerve = Swerve.getInstance();

  private Pose2d startPoseAbsolute;
  private Supplier<Translation2d> targetTranslationRelativeSupplier;
  private Supplier<Rotation2d> targetRotationAbsoluteSupplier;

  private TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(4.0, 1.75);
  private ProfiledPIDController driveController = new ProfiledPIDController(8.0, 0.02, 0.01, driveConstraints);

  private TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 2.0);
  private ProfiledPIDController angleController = new ProfiledPIDController(4.0, 0.1, 0.08, angleConstraints);
  
  private boolean ignoreHeightMultiplier = false;

  public TestDriveToRelativePose(
    Supplier<Pose2d> targetPoseRelative,
    boolean ignoreHeightMultiplier,
    double maxSpeed
  ) {
    this.targetTranslationRelativeSupplier = () -> targetPoseRelative.get().getTranslation();
    this.targetRotationAbsoluteSupplier = () -> targetPoseRelative.get().getRotation();
    this.ignoreHeightMultiplier = ignoreHeightMultiplier;
    this.driveConstraints = new TrapezoidProfile.Constraints(maxSpeed, 1.75);
    
    driveController.setTolerance(Units.inchesToMeters(1.85));
    angleController.setTolerance(Units.degreesToRadians(3));

    addRequirements(Swerve.getInstance());
  }

  public TestDriveToRelativePose(
    Supplier<Pose2d> targetPoseRelative,
    boolean ignoreHeightMultiplier
  ) {
    this(targetPoseRelative, ignoreHeightMultiplier, 4.0);
  }

  public TestDriveToRelativePose(
    Supplier<Translation2d> targetTranslationRelativeSupplier,
    Supplier<Rotation2d> targetRotationAbsoluteSupplier,
    boolean ignoreHeightMultiplier
  ) {
    this(targetTranslationRelativeSupplier, targetRotationAbsoluteSupplier, ignoreHeightMultiplier, 4.0);
  }

  public TestDriveToRelativePose(
    Supplier<Translation2d> targetTranslationRelativeSupplier,
    Supplier<Rotation2d> targetRotationAbsoluteSupplier,
    boolean ignoreHeightMultiplier,
    double maxSpeed
  ) {
    this(() -> new Pose2d(
      targetTranslationRelativeSupplier.get(),
      targetRotationAbsoluteSupplier.get()
    ), ignoreHeightMultiplier, 4.0);
  }

  public TestDriveToRelativePose(
    Supplier<Translation2d> targetTranslationRelativeSupplier,
    Supplier<Rotation2d> targetRotationAbsoluteSupplier
  ) {
    this(targetTranslationRelativeSupplier, targetRotationAbsoluteSupplier, false);
  }

  @Override
  public void initialize() {
    startPoseAbsolute = swerve.getPose();

    updatePositions();
    updateDistance();
    updateRotations();

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    driveController.reset(
      0,
      swerve.getVelocity().getNorm()
    );

    angleController.reset(
      swerve.getPose().getRotation().getRadians(),
      swerve.getRotationalVelocity().getRadians()
    );
  }

  private Pose2d currentPoseRelative;
  private Translation2d targetTranslationRelative;
  
  private void updatePositions() {
    var currentPose = swerve.getPose();
    currentPoseRelative = currentPose.relativeTo(startPoseAbsolute);
    targetTranslationRelative = this.targetTranslationRelativeSupplier.get();

    driveController.setGoal(
      targetTranslationRelative.getNorm()
    );
  }

  private double rotation;

  private void updateRotations() {
    var currentPose = swerve.getPose();
    rotation = currentPose.getRotation().getRadians();
    
    angleController.setGoal(
      targetRotationAbsoluteSupplier.get().getRadians()
    );
  }
  
  private double distanceFromStart;

  private void updateDistance() {
    var translationFromStart = currentPoseRelative.getTranslation();
    distanceFromStart = translationFromStart.getNorm();
  }

  @Override
  public void execute() {
    updatePositions();
    updateDistance();
    updateRotations();

    var rotationOutput = angleController.calculate(rotation);
    var strafeOutput = driveController.calculate(distanceFromStart);
    
    var unitVector = targetTranslationRelative.minus(currentPoseRelative.getTranslation());
    var dist = unitVector.getNorm();
    if(dist == 0) {
      if(ignoreHeightMultiplier) {
        swerve.driveUnsafe(
          Translation2d.kZero,
          rotationOutput,
          false,
          true
        );
      } else {
        swerve.drive(
          Translation2d.kZero,
          rotationOutput,
          false,
          true
        );
      }
    } else {
      unitVector = unitVector.div(dist);
      if(ignoreHeightMultiplier) {
        swerve.driveUnsafe(
          unitVector
            .times(strafeOutput)
            .rotateBy(currentPoseRelative.getRotation().times(-1)),
          rotationOutput,
          false,
          true
        );
      } else {
        swerve.drive(
          unitVector
            .times(strafeOutput)
            .rotateBy(currentPoseRelative.getRotation().times(-1)),
          rotationOutput,
          false,
          true
        );
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(
      Translation2d.kZero,
      0,
      false,
      true
    );
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && angleController.atGoal();
  }
}
