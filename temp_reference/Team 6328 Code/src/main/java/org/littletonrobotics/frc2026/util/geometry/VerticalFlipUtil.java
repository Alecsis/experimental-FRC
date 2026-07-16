// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.geometry;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.*;
import java.util.stream.DoubleStream;
import org.littletonrobotics.frc2026.FieldConstants;

public class VerticalFlipUtil {
  public static double applyY(double y) {
    return FieldConstants.fieldWidth - y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(translation.getX(), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return rotation.times(-1.0);
  }

  public static Pose2d apply(Pose2d pose) {
    return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static Bounds apply(Bounds bounds) {
    return new Bounds(bounds.minX(), bounds.maxX(), applyY(bounds.maxY()), applyY(bounds.minY()));
  }

  public static SwerveSample apply(SwerveSample sample) {
    return new SwerveSample(
        sample.t,
        sample.x,
        applyY(sample.y),
        apply(Rotation2d.fromRadians(sample.heading)).getRadians(),
        sample.vx,
        -sample.vy,
        -sample.omega,
        sample.ax,
        -sample.ay,
        -sample.alpha,
        sample.moduleForcesX(),
        DoubleStream.of(sample.moduleForcesY()).map(y -> -y).toArray());
  }
}
