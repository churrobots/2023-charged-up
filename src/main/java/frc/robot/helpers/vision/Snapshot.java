package frc.robot.helpers.vision;
// https://github.com/Team100/main2023/blob/main/swerve100/src/main/java/team100/localization/Blip.java

import java.util.Arrays;

/**
 * Something the camera sees. This is 1:1 copy of the representation in the
 * AprilTag library.
 */
public class Snapshot {
  /**
   * AprilTag ID
   */
  public final int id;

  /**
   * Rotation in camera (z-forward) frame. This is generally not accurate enough
   * to use for camera pose estimation.
   */
  public final double[][] pose_R;

  /**
   * Translation in camera (z-forward) frame. Even though this is logically a
   * vector it is represented by the AprilTag library as a 3x1 2d array so we use
   * the same representation
   * here.
   */
  public final double[][] pose_t;

  /**
   * Package-private for testing.
   */
  Snapshot(int id, double[][] pose_R, double[][] pose_t) {
    this.id = id;
    this.pose_R = pose_R;
    this.pose_t = pose_t;
  }

  /**
   * For the deserializer.
   */
  protected Snapshot() {
    this.id = 0;
    this.pose_R = new double[3][3];
    this.pose_t = new double[3][1];
  }

  @Override
  public String toString() {
    return "Snapshot [id=" + id + ", pose_R=" + Arrays.deepToString(pose_R) + ", pose_t=" + Arrays.deepToString(pose_t)
        + "]";
  }
}
