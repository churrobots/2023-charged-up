package frc.robot.helpers.vision;
// https://github.com/Team100/main2023/blob/main/swerve100/src/main/java/team100/localization/Blips.java

import java.util.ArrayList;
import java.util.List;

/**
 * The entire payload from the camera. This is 1:1 copy of the representation in
 * the AprilTag library and our python wrapper.
 */
public class Snapshots {
  /**
   * Elapsed time of the analysis in python.
   * TODO: calibrate this number so we can use it for Kalman filter updates.
   */
  public final double et;

  /**
   * The set of targets seen by the camera.
   */
  public final List<Snapshot> tags;

  /**
   * For the deserializer.
   */
  protected Snapshots() {
    et = 0;
    tags = new ArrayList<Snapshot>();
  }

  @Override
  public String toString() {
    return "Snapshots [et=" + et + ", tags=" + tags + "]";
  }
}
