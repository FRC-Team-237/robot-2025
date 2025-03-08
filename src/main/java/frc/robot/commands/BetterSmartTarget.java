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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class BetterSmartTarget extends Command {
  
  private Swerve swerve = Swerve.getInstance();
  private PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  private final Joystick panel = new Joystick(1);
  private final JoystickButton rightSwitch = new JoystickButton(panel, 15);

  private int targetId = -1;
  private Optional<PhotonTrackedTarget> target;

  private static final TrapezoidProfile.Constraints rotationConstraints =
    new TrapezoidProfile.Constraints(1.0, 1);
  private final ProfiledPIDController rotationController =
    new ProfiledPIDController(4, 0.0, 0, rotationConstraints);
  
  private static final TrapezoidProfile.Constraints strafeConstraints =
    new TrapezoidProfile.Constraints(2.0, 1);
  private final ProfiledPIDController strafeController =
    new ProfiledPIDController(4.0, 0.4, 0, strafeConstraints);
  
  public BetterSmartTarget() {
    rotationController.setTolerance(1.0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    strafeController.setTolerance(0.025);
  }

  private Translation2d startPose;
  private Rotation2d startRotation;
  
  private void captureLatest(boolean matchId) {
    target = camera
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
  }


  @Override public void initialize() {
    captureLatest(false);
    if(target.isEmpty()) return;

    targetId = target.get().getFiducialId();

    if(targetId == -1) {
      target = Optional.empty();
      return;
    }

    startPose = swerve.getPose().getTranslation();
    startRotation = swerve.getGyroYaw();

    rotationController.setGoal(
      target.get()
        .getBestCameraToTarget()
        .getRotation()
        .getZ() + startRotation.getRadians() + Math.PI
    );

    rotationController.reset(
      startRotation.getRadians()
    );

    strafeController.reset(0);
  }

  @Override
  public void execute() {
    if(target.isEmpty()) return;

    var currentPose = swerve.getPose().getTranslation();

    var distanceGoal = 0.7;

    var sideOffset = rightSwitch.getAsBoolean() ? -0.2 : 0.175;

    var targetTransform = target.get().getBestCameraToTarget();
    var inverseTargetTranslation = targetTransform.inverse().getTranslation().toTranslation2d();

    var forwardUnit = inverseTargetTranslation.plus(new Translation2d(-distanceGoal, sideOffset))
      .rotateBy(
        targetTransform.getRotation().toRotation2d()
          .plus(Rotation2d.k180deg)
      );
    
    var goalPose = startPose
      .plus(
        inverseTargetTranslation.plus(new Translation2d(-distanceGoal, sideOffset))
      );
    
    var goalDistance = Math.abs(
      startPose.getDistance(currentPose) - startPose.getDistance(goalPose)
    );

    var strafeOutput = strafeController.calculate(
      startPose.getDistance(currentPose),
      startPose.getDistance(goalPose)
    );

    var currentRotation = swerve.getGyroYaw();
    var rotationDistance = Math.abs(
      rotationController.getGoal().position - currentRotation.getRadians()
    );

    if(goalDistance >= 0.05) {
      swerve.driveUnsafe(
        forwardUnit.times(strafeOutput),
        0,
        false,
        true
      );
      SmartDashboard.putString("TARGET/PHASE", "STRAFING");
    } else if(rotationDistance > Radians.convertFrom(2, Degree)) {
      var rotationOutput = rotationController.calculate(currentRotation.getRadians());

      swerve.driveUnsafe(
        Translation2d.kZero,
        rotationOutput,
        false,
        true
      );
      SmartDashboard.putString("TARGET/PHASE", "ROTATING");
    } else {
      swerve.driveUnsafe(
        new Translation2d(0.075, 0.0),
        0,
        false,
        true
      );
      SmartDashboard.putString("TARGET/PHASE", "FORWARD");
    }
  }

  @Override
  public boolean isFinished() {
    if(target.isEmpty()) return true;

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
  }
}
