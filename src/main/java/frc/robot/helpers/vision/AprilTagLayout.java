package frc.robot.helpers.vision;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The WPILib JSON tag file, and the wrapper, AprilTagFieldLayout, define tag
 * rotation with respect to the *outward* normal, which is the opposite of the
 * Apriltags convention to use the *inward* normal.
 * 
 * This wrapper "fixes" the orientations so they match the Apriltag convention,
 * and thus match the result of camera pose estimates. Without this fix, we
 * would have to sprinkle inversions here and there, which would result in bugs.
 * 
 * @see https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#coordinate-system
 */
public class AprilTagLayout {
  // Inverts yaw
  private static final Transform3d kFix = new Transform3d(
      new Translation3d(),
      new Rotation3d(0, 0, Math.PI));
  private final AprilTagFieldLayout layout;

  // this is private because i don't want the red/blue enum in our code.
  private AprilTagLayout(OriginPosition origin) throws IOException {
    Path path = Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
    layout = new AprilTagFieldLayout(path);
    layout.setOrigin(origin);
  }

  /**
   * @return Layout from the red perspective.
   */
  public static AprilTagLayout redLayout() throws IOException {
    return new AprilTagLayout(OriginPosition.kRedAllianceWallRightSide);
  }

  /**
   * @return Layout from the blue perspective.
   */
  public static AprilTagLayout blueLayout() throws IOException {
    return new AprilTagLayout(OriginPosition.kBlueAllianceWallRightSide);
  }

  /**
   * TODO: get rid of Optional here.
   * 
   * @return Tag pose with correct yaw (inverted compared to json file)
   */
  public Optional<Pose3d> getTagPose(int id) {
    Optional<Pose3d> pose = layout.getTagPose(id);
    if (!pose.isPresent()) {
      return pose;
    }
    return Optional.of(pose.get().transformBy(kFix));
  }
}