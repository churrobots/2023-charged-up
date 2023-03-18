package frc.robot.helpers.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Static methods used to interpret camera input.
 * 
 * TODO: make these not static
 */
public class PoseEstimator {

  /**
   * Calculate robot pose.
   * 
   * First calculates the distance to the tag. If it's closer than the threshold,
   * use the camera-derived tag rotation. If it's far, use the gyro.
   */
  public static Pose3d getRobotPoseInFieldCoords(
      Transform3d cameraInRobotCoords,
      Pose3d tagInFieldCoords,
      Snapshot snapshot,
      Rotation3d robotRotationInFieldCoordsFromGyro,
      double thresholdMeters) {
    Translation3d tagTranslationInCameraCoords = snapshotToTranslation(snapshot);
    if (tagTranslationInCameraCoords.getNorm() < thresholdMeters) {
      return getRobotPoseInFieldCoords(
          cameraInRobotCoords,
          tagInFieldCoords,
          snapshot);
    }
    return getRobotPoseInFieldCoords(
        cameraInRobotCoords,
        tagInFieldCoords,
        snapshot,
        robotRotationInFieldCoordsFromGyro);

  }

  /**
   * Calculate robot pose.
   * 
   * Given the snapshot and its corresponding field location, and the camera
   * offset,
   * return the robot pose in field coordinates.
   * 
   * This method trusts the tag rotation calculated by the camera.
   */
  public static Pose3d getRobotPoseInFieldCoords(
      Transform3d cameraInRobotCoords,
      Pose3d tagInFieldCoords,
      Snapshot snapshot) {
    Transform3d tagInCameraCoords = snapshotToTransform(snapshot);
    Pose3d cameraInFieldCoords = toFieldCoordinates(tagInCameraCoords, tagInFieldCoords);
    return applyCameraOffset(cameraInFieldCoords, cameraInRobotCoords);
  }

  /**
   * Calculate robot pose.
   * 
   * Given the snapshot, the heading, the camera offset, and the absolute tag
   * pose,
   * return the absolute robot pose in field coordinates.
   * 
   * This method does not trust the tag rotation from the camera, it uses the gyro
   * signal instead.
   * 
   * @param cameraInRobotCoords                camera offset expressed as a
   *                                           transform from robot-frame to
   *                                           camera-frame, e.g.camera 0.5m in
   *                                           front of the robot center and 0.5m
   *                                           from the floor would have a
   *                                           translation (0.5, 0, 0.5)
   * @param tagInFieldCoords                   tag location expressed as a pose in
   *                                           field-frame. this should come from
   *                                           the json
   * @param snapshot                           direct from the camera
   * @param robotRotationInFieldCoordsFromGyro direct from the gyro. note that
   *                                           drive.getPose() isn't exactly the
   *                                           gyro reading; it might be better to
   *                                           use the real gyro than the getPose
   *                                           method.
   */
  public static Pose3d getRobotPoseInFieldCoords(
      Transform3d cameraInRobotCoords,
      Pose3d tagInFieldCoords,
      Snapshot snapshot,
      Rotation3d robotRotationInFieldCoordsFromGyro) {
    Rotation3d cameraRotationInFieldCoords = cameraRotationInFieldCoords(
        cameraInRobotCoords,
        robotRotationInFieldCoordsFromGyro);
    Translation3d tagTranslationInCameraCoords = snapshotToTranslation(snapshot);
    Rotation3d tagRotationInCameraCoords = tagRotationInRobotCoordsFromGyro(
        tagInFieldCoords.getRotation(),
        cameraRotationInFieldCoords);
    Transform3d tagInCameraCoords = new Transform3d(
        tagTranslationInCameraCoords,
        tagRotationInCameraCoords);
    Pose3d cameraInFieldCoords = toFieldCoordinates(
        tagInCameraCoords,
        tagInFieldCoords);
    return applyCameraOffset(
        cameraInFieldCoords,
        cameraInRobotCoords);
  }

  //////////////////////////////
  //
  // package private below, don't use these.

  /**
   * given the gyro rotation and the camera offset, return the camera absolute
   * rotation. Package-private for testing.
   */
  static Rotation3d cameraRotationInFieldCoords(
      Transform3d cameraInRobotCoords,
      Rotation3d robotRotationInFieldCoordsFromGyro) {
    return cameraInRobotCoords.getRotation()
        .rotateBy(robotRotationInFieldCoordsFromGyro);
  }

  /**
   * Extract translation and rotation from z-forward snapshot and return the same
   * translation and rotation as an NWU x-forward transform. Package-private for
   * testing.
   */
  static Transform3d snapshotToTransform(Snapshot b) {
    return new Transform3d(snapshotToTranslation(b), snapshotToRotation(b));
  }

  /**
   * Extract the translation from a "z-forward" snapshot and return the same
   * translation expressed in our usual "x-forward" NWU translation.
   * It would be possible to also consume the snapshot rotation matrix, if it were
   * renormalized, but it's not very accurate, so we don't consume it.
   * Package-private for testing.
   */
  static Translation3d snapshotToTranslation(Snapshot b) {
    return new Translation3d(b.pose_t[2][0], -1.0 * b.pose_t[0][0], -1.0 * b.pose_t[1][0]);
  }

  /**
   * Extract the rotation from the "z forward" snapshot and return the same
   * rotation
   * expressed in our usual "x forward" NWU coordinates. Package-private for
   * testing.
   */
  static Rotation3d snapshotToRotation(Snapshot b) {
    Mat rmat = new Mat(3, 3, CvType.CV_64F);
    rmat.put(0, 0, b.pose_R[2][2]);
    rmat.put(0, 1, -b.pose_R[2][0]);
    rmat.put(0, 2, -b.pose_R[2][1]);

    rmat.put(1, 0, -b.pose_R[0][2]);
    rmat.put(1, 1, b.pose_R[0][0]);
    rmat.put(1, 2, b.pose_R[0][1]);

    rmat.put(2, 0, -b.pose_R[1][2]);
    rmat.put(2, 1, b.pose_R[1][0]);
    rmat.put(2, 2, b.pose_R[1][1]);

    // convert it to axis-angle
    Mat rvec = new Mat(3, 1, CvType.CV_64F);
    Calib3d.Rodrigues(rmat, rvec);

    // convert back to rotation matrix -- this should be orthogonal
    Mat rmat2 = new Mat();
    Calib3d.Rodrigues(rvec, rmat2);

    Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        matrix.set(row, col, rmat2.get(row, col)[0]);
      }
    }

    return new Rotation3d(matrix);
  }

  /**
   * Because the camera's estimate of tag rotation isn't very accurate, this
   * synthesizes an estimate using the tag rotation in field frame (from json) and
   * the camera rotation in field frame (from gyro). Package-private for testing.
   */
  static Rotation3d tagRotationInRobotCoordsFromGyro(
      Rotation3d tagRotationInFieldCoords,
      Rotation3d cameraRotationInFieldCoords) {
    return tagRotationInFieldCoords.rotateBy(cameraRotationInFieldCoords.unaryMinus());
  }

  /**
   * Given the tag in camera frame and tag in field frame, return the camera in
   * field frame. Package-private for testing.
   */
  static Pose3d toFieldCoordinates(Transform3d tagInCameraCords, Pose3d tagInFieldCords) {
    // First invert the camera-to-tag transform, obtaining tag-to-camera.
    Transform3d cameraInTagCords = tagInCameraCords.inverse();
    // Then compose field-to-tag with tag-to-camera to get field-to-camera.
    return tagInFieldCords.transformBy(cameraInTagCords);
  }

  /**
   * Given the camera in field frame and camera in robot frame, return the robot
   * in field frame. Package-private for testing.
   */
  static Pose3d applyCameraOffset(Pose3d cameraInFieldCoords, Transform3d cameraInRobotCoords) {
    Transform3d robotInCameraCoords = cameraInRobotCoords.inverse();
    return cameraInFieldCoords.transformBy(robotInCameraCoords);
  }

}
