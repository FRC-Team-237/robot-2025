package frc.robot.commands.tests;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Camera;
import frc.robot.subsystems.Swerve;

public class TestDriveToTargetWithOffset extends Command {
  
  private Swerve swerve = Swerve.getInstance();

  private Pose2d startPoseAbsolute;
  private Supplier<Translation2d> targetOffsetTranslationRelativeSupplier;
  private Supplier<Rotation2d> targetRotationRelativeSupplier;

  private TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(3.0, 2.0);
  private ProfiledPIDController driveController = new ProfiledPIDController(10.0, 0.1, 0.01, driveConstraints);

  private TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 2.0);
  private ProfiledPIDController angleController = new ProfiledPIDController(4.0, 0.1, 0.08, angleConstraints);

  public TestDriveToTargetWithOffset(
    Supplier<Translation2d> targetOffsetTranslationRelativeSupplier,
    Supplier<Rotation2d> targetRotationRelativeSupplier
  ) {
    this.targetOffsetTranslationRelativeSupplier = targetOffsetTranslationRelativeSupplier;
    this.targetRotationRelativeSupplier = targetRotationRelativeSupplier;

    driveController.setTolerance(Units.inchesToMeters(0.75));
    angleController.setTolerance(Units.degreesToRadians(3));

    addRequirements(Swerve.getInstance());
  }

  private int targetId = -1;
  private Optional<PhotonTrackedTarget> target = Optional.empty();

  private void updateTarget() {
    // var target = Camera.getLatestResult();
    // if(target.isPresent()) {
    //   var t = target.get();
    //   if(t.fiducialId == targetId) this.target = target;
    // }
  }

  private Pose2d currentPoseRelative;
  private Translation2d targetTranslationRelative;

  private void updatePositions() {
    var currentPose = swerve.getPose();
    currentPoseRelative = currentPose.relativeTo(startPoseAbsolute);
    targetTranslationRelative = targetOffsetTranslationRelativeSupplier.get()
      .plus(target.get().getBestCameraToTarget().inverse().getTranslation().toTranslation2d());
    
    driveController.setGoal(
      targetTranslationRelative.getNorm()
    );
  }

  private double rotation;

  private void updateRotations() {
    var currentPose = swerve.getPose();
    rotation = currentPose.getRotation().getRadians();

    var targetRotation = startPoseAbsolute.getRotation()
      .plus(targetRotationRelativeSupplier.get())
      .plus(target.get().getBestCameraToTarget().getRotation().toRotation2d().plus(Rotation2d.k180deg));
    
    angleController.setGoal(
      targetRotation.getRadians()
    );
  }

  private double distanceFromStart;

  private void updateDistance() {
    var translationFromStart = currentPoseRelative.getTranslation();
    distanceFromStart = translationFromStart.getNorm();
  }

  @Override
  public void initialize() {
    swerve.clearAngleSetpoint();

    startPoseAbsolute = swerve.getPose();
    var target = Camera.getLatestResult();
    if(target.isPresent()) {
      targetId = target.get().fiducialId;
      this.target = target;}

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    driveController.reset(0, swerve.getVelocity().getNorm());
    angleController.reset(
      startPoseAbsolute.getRotation().getRadians(),
      swerve.getRotationalVelocity().getRadians()
    );
  }

  @Override
  public void execute() {
    if(targetId == -1 || !target.isPresent()) return;
    updateTarget();

    updatePositions();
    updateDistance();
    updateRotations();

    var rotationOutput = angleController.calculate(rotation);
    var strafeOutput = driveController.calculate(distanceFromStart);

    var unitVector = targetTranslationRelative.minus(currentPoseRelative.getTranslation());
    var dist = unitVector.getNorm();
    if(dist == 0) {
      swerve.drive(
        Translation2d.kZero,
        rotationOutput,
        false,
        true
      );
    } else {
      unitVector = unitVector.div(dist);
      swerve.driveUnsafe(
        unitVector
          .times(
            Math.max(strafeOutput, 0.1)
          )
          .rotateBy(currentPoseRelative.getRotation().times(-1)),
        rotationOutput,
        false,
        true
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return
      driveController.atGoal() &&
      angleController.atGoal();
  }
}