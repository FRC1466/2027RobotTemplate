package first.robot.auto;

import bline.path.Path;
import bline.path.Path.PathConstraints;
import bline.path.Path.TranslationTarget;
import bline.path.Path.Waypoint;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * Slows paths down for sharp corners. BLine slows only for a path's final stop, not for corners
 * between its points, so a sharp corner taken at full speed overshoots. On a 2023 robot built from
 * this template, a 49 degree corner at 3 m/s overshot by about 0.5 m.
 *
 * <p>{@link #limit} caps the max velocity of each segment that ends in a corner sharper than a
 * threshold. BLine applies a point's limit while the robot drives toward that point, so the robot
 * arrives at the corner slowly and speeds up again on the next segment. In simulation, a 90 degree
 * corner on a 3 m/s path overshot by 1.09 m uncapped and 0.32 m capped at 1.5 m/s. Lower the corner
 * speed for very sharp corners or tight clearances, and check the corner in simulation.
 *
 * <pre>{@code
 * public static Path aroundTheStation() {
 *   return PathCorners.limit(new Path(fast(), start, new TranslationTarget(...), end));
 * }
 * }</pre>
 */
public final class PathCorners {
  private PathCorners() {}

  /** Corners that turn the direction of travel more than this are capped. */
  public static final double DEFAULT_SHARP_CORNER_DEGREES = 35;

  /** Estimate from the 2023 robot. Check the corner in simulation and on carpet. */
  public static final double DEFAULT_CORNER_SPEED_METERS_PER_SEC = 1.5;

  /** {@link #limit(Path, double, double)} with the default threshold and corner speed. */
  public static Path limit(Path path) {
    return limit(path, DEFAULT_SHARP_CORNER_DEGREES, DEFAULT_CORNER_SPEED_METERS_PER_SEC);
  }

  /**
   * Caps the speed into every corner of {@code path} sharper than {@code sharpCornerDegrees}. The
   * path's own constraints still apply everywhere else.
   *
   * @return the same path, with its constraints replaced
   */
  public static Path limit(Path path, double sharpCornerDegrees, double cornerSpeedMetersPerSec) {
    List<Translation2d> points = translationPoints(path);
    // Translation ordinals count the start waypoint as 0, then each later point in order. Corner
    // limits come first so they win over the path's own velocity where both apply.
    var constraints = new ArrayList<PathConstraints>();
    for (int i : sharpCorners(points, sharpCornerDegrees)) {
      constraints.add(
          new PathConstraints().setMaxVelocityMetersPerSec(cornerSpeedMetersPerSec).inRange(i, i));
    }
    if (constraints.isEmpty()) {
      return path;
    }
    constraints.add(path.getPathConstraints());
    path.setPathConstraints(PathConstraints.combine(constraints.toArray(PathConstraints[]::new)));
    return path;
  }

  /** Ordinals of the points where the direction of travel turns more than the threshold. */
  static List<Integer> sharpCorners(List<Translation2d> points, double sharpCornerDegrees) {
    var corners = new ArrayList<Integer>();
    for (int i = 1; i + 1 < points.size(); i++) {
      var in = points.get(i).minus(points.get(i - 1));
      var out = points.get(i + 1).minus(points.get(i));
      if (in.getNorm() < 1e-9 || out.getNorm() < 1e-9) {
        continue;
      }
      double turn =
          Math.abs(
              Math.IEEEremainder(
                  Math.atan2(out.getY(), out.getX()) - Math.atan2(in.getY(), in.getX()),
                  2 * Math.PI));
      if (Math.toDegrees(turn) > sharpCornerDegrees) {
        corners.add(i);
      }
    }
    return corners;
  }

  /** The path's translation points in order: waypoints and translation targets. */
  static List<Translation2d> translationPoints(Path path) {
    var points = new ArrayList<Translation2d>();
    for (var element : path.getPathElements()) {
      if (element instanceof Waypoint waypoint) {
        points.add(waypoint.translationTarget().translation());
      } else if (element instanceof TranslationTarget target) {
        points.add(target.translation());
      }
    }
    return points;
  }
}
