package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

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
}
