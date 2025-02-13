package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
  private static PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  public static PhotonTrackedTarget getBestResult() {
    var results = camera.getAllUnreadResults();

    for(var result : results) {
      if(result.hasTargets()) {
        return result.getBestTarget();
      }
    }

    return null;
  }
}
