package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class BigBrainTarget extends Command {
  private Swerve swerve = Swerve.getInstance();
  private PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  private final Joystick panel = new Joystick(1);
  private final JoystickButton rightSwitch = new JoystickButton(panel, 15);

  private int targetId = -1;
  private Optional<PhotonTrackedTarget> target = Optional.empty();

  private static final TrapezoidProfile.Constraints rotationConstraints =
    new TrapezoidProfile.Constraints(2.0, 1);
  private final ProfiledPIDController rotationController =
    new ProfiledPIDController(4, 0.0, 0, rotationConstraints);
  
  private static final TrapezoidProfile.Constraints strafeConstraints =
    new TrapezoidProfile.Constraints(2.5, 1);
  private final ProfiledPIDController strafeController =
    new ProfiledPIDController(4.0, 0.2, 0, strafeConstraints);
  
  private Translation2d translationDelta;
  private Rotation2d rotationDelta;

  private Timer updateTargetThrottle = new Timer();
  
  private boolean captureLatest(boolean matchId) {
    var maybeTarget = camera
      .getLatestResult()
      .getTargets()
      .stream()
      .sorted((t1, t2) -> t1.getArea() > t2.getArea() ? -1 : 1)
      .filter(t -> {
        var goodPoseAmbiguity = t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1;
        var goodMatchId = t.getFiducialId() == targetId;

        if(matchId) {
          return goodPoseAmbiguity && goodMatchId;
        }
        
        return goodPoseAmbiguity;
      })
      .findFirst();
    
    if(maybeTarget.isPresent()) {
      target = maybeTarget;
      return true;
    }

    return false;
  }

  public BigBrainTarget() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Translation2d startTranslation;
  private Rotation2d startRotation;

  @Override
  public void initialize() {
    captureLatest(false);
    if(target.isEmpty()) return;
    if(target.get().getFiducialId() == -1) {
      target = Optional.empty();
      return;
    }
    targetId = target.get().getFiducialId();

    updatePIDTargets();
    updateTargetThrottle.reset();
  }

  private void updatePIDTargets() {
    if(!updateTargetThrottle.isRunning()) {
      updateTargetThrottle.start();
    } else if(!updateTargetThrottle.hasElapsed(1.0)) {
      return;
    } else {
      updateTargetThrottle.reset();
      updateTargetThrottle.start();
    }

    startTranslation = swerve.getPose().getTranslation();
    startRotation = swerve.getGyroYaw();

    rotationController.reset(
      swerve.getGyroYaw().getRadians(),
      swerve.getRotationalVelocity().getRadians()
    );

    strafeController.reset(
      0,
      swerve.getVelocity().getNorm()
    );

    var targetTransform = target.get().getBestCameraToTarget();

    rotationDelta = targetTransform
      .getRotation()
      .toRotation2d()
      .plus(startRotation)
      .plus(Rotation2d.k180deg);
    
    translationDelta = targetTransform
      .inverse()
      .getTranslation()
      .toTranslation2d();
    
    var sideOffset = rightSwitch.getAsBoolean() ? -0.2 : 0.175;
    
    translationDelta = translationDelta.plus(new Translation2d(-1.0, sideOffset))
      .rotateBy(
        targetTransform.getRotation().toRotation2d()
          .plus(Rotation2d.k180deg)
      );
    
    rotationController.setGoal(rotationDelta.getRadians());
    strafeController.setGoal(translationDelta.getNorm());
  }

  @Override
  public void execute() {
    if(target.isEmpty()) return;

    var capturedNewTarget = captureLatest(true);
    if(capturedNewTarget) {
      // updatePIDTargets();
    }

    var currentPose = swerve.getPose().getTranslation();
    var currentAngle = swerve.getGyroYaw();
    var targetUnit = translationDelta
      .rotateBy(startRotation)
      .div(translationDelta.getNorm());
    
    var distanceFromStart = startTranslation.getDistance(currentPose);
    
    var strafeOutput = strafeController.calculate(distanceFromStart);
    var rotationOutput = rotationController.calculate(currentAngle.getRadians());
    
    swerve.driveUnsafe(
      targetUnit.times(strafeOutput),
      rotationOutput,
      true,
      true
    );
  }

  @Override
  public boolean isFinished() {
    var currentPose = swerve.getPose().getTranslation();
    var distanceFromStart = startTranslation.getDistance(currentPose);
    var strafeDifference = translationDelta.getNorm() - distanceFromStart;
    var rotationDifference = rotationDelta.getRadians() - swerve.getGyroYaw().getRadians();

    var strafeFinished = Math.abs(strafeDifference) < 0.1;
    var rotationFinished = Math.abs(rotationDifference) < Radians.convertFrom(1, Degree);

    return strafeFinished && rotationFinished;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
  }
}
