package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TrackReefHorizontally extends Command {
  
  public enum Side { LEFT, RIGHT }

  private Swerve swerve = Swerve.getInstance();
  private PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  private Supplier<Side> sideSupplier;

  private int targetId = -1;
  private Optional<PhotonTrackedTarget> target = Optional.empty();

  private static final TrapezoidProfile.Constraints strafeConstraints =
    new TrapezoidProfile.Constraints(2.0, 1);
  private final ProfiledPIDController strafeController =
    new ProfiledPIDController(2.5, 0.0, 0.0, strafeConstraints);

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

  public TrackReefHorizontally(Supplier<Side> sideSupplier) {
    this.sideSupplier = sideSupplier;

    this.addRequirements(Swerve.getInstance());
  }

  Translation2d startTranslation;
  private double translationDelta;

  private void updatePIDTargets() {
    var currentPose = swerve.getPose().getTranslation();
    startTranslation = currentPose;

    var targetTransform = target.get().getBestCameraToTarget();

    var sideOffset = sideSupplier.get() == Side.LEFT
      ? Meters.convertFrom(5.5, Inches)
      : -Meters.convertFrom(8.5, Inches);

    translationDelta = targetTransform
      .inverse()
      .getTranslation()
      .getY() + sideOffset;
    
    SmartDashboard.putNumber("TARGET/Translation Delta", translationDelta);
    
    strafeController.setGoal(translationDelta);
  }

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
  }

  @Override
  public void execute() {
    var capturedNewTarget = captureLatest(true);
    if(capturedNewTarget) {
      updatePIDTargets();
    }
    if(target.isEmpty()) return;

    var currentPose = swerve.getPose().getTranslation();

    var distanceFromStart = startTranslation.getDistance(currentPose);

    var output = strafeController.calculate(distanceFromStart);
    
    SmartDashboard.putNumber("TARGET/Distance From Start", distanceFromStart);
    SmartDashboard.putNumber("TARGET/Strafe Output", output);

    swerve.driveUnsafe(
      new Translation2d(0, MathUtil.clamp(output, -0.225, 0.225)),
      0,
      false,
      true
    );
  }

  @Override
  public boolean isFinished() {
    if(target.isEmpty()) return true;
    var currentPose = swerve.getPose().getTranslation();
    var distanceFromStart = startTranslation.getDistance(currentPose);
    var difference = Math.abs(
      translationDelta - distanceFromStart
    );

    return difference < Meters.convertFrom(0.5, Inches);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
  }
}
