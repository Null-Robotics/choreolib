// Copyright (c) Choreo contributors

package dev.nullrobotics.choreolib.common.trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * The generic interface for a sample in a trajectory.
 *
 * @param <Self> Derived sample type.
 */
public interface TrajectorySample<Self extends TrajectorySample<Self>> {
  /**
   * Returns the timestamp of this sample.
   *
   * @return the timestamp of this sample.
   */
  double getTimestamp();

  /**
   * Returns the pose at this sample.
   *
   * @return the pose at this sample.
   */
  Pose2D getPose();

  /**
   * Returns this sample, offset by the given timestamp.
   *
   * @param timestampOffset the offset to apply to the timestamp.
   * @return this sample, offset by the given timestamp.
   */
  Self offsetBy(double timestampOffset);

  /**
   * Returns the sample at a certain timestamp, linearly interpolated from this timestamp and one ahead.
   * @param aheadState the state ahead of this one.
   * @param timestamp the timestamp to interpolate to.
   * @return the sample at the given timestamp, interpolated from the timestamps of this and the ahead state.
   */
  Self interpolate(Self aheadState, double timestamp);
}
