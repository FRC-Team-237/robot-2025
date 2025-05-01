package frc.robot;

import java.util.Optional;
import java.util.function.Function;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Camera {
  private static PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  public static Optional<PhotonTrackedTarget> getBestResult() {
    var results = camera.getAllUnreadResults();

    for(var result : results) {
      if(result.hasTargets()) {
        return Optional.of(result.getBestTarget());
      }
    }

    return Optional.empty();
  }

  // public static void getBestTarget(Consumer<PhotonTrackedTarget> callback) {
  //   var results = camera.getAllUnreadResults();

  //   for(var result : results) {
  //     if(result.hasTargets()) {
  //       callback.accept(result.getBestTarget());
  //     }
  //   }
  // }

  public static Optional<PhotonTrackedTarget> getLatestResult() {
    var target = camera.getLatestResult()
        .getTargets()
        .stream()
        .sorted((t1, t2) -> t1.getArea() > t2.getArea() ? -1 : 1)
        .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
        .findFirst();
    return target;
  }
}
