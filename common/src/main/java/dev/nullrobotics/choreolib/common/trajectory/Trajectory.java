// Copyright (c) Choreo contributors

package dev.nullrobotics.choreolib.common.trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.joml.Vector2d;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * A trajectory loaded from Choreo.
 *
 * @param <SampleType> DifferentialSample or SwerveSample.
 */
public class Trajectory<SampleType extends TrajectorySample<SampleType>> {
    private final String name;
    private final List<SampleType> samples;
    private final List<Integer> splits;
    private final List<EventMarker> events;

    /**
     * Constructs a Trajectory with the specified parameters.
     *
     * @param name    The name of the trajectory.
     * @param samples The samples of the trajectory.
     * @param splits  The indices of the splits in the trajectory.
     * @param events  The events in the trajectory.
     */
    public Trajectory(
            String name, List<SampleType> samples, List<Integer> splits, List<EventMarker> events) {
        this.name = name;
        this.samples = samples;
        this.splits = splits;
        this.events = events;
    }

    /**
     * Returns the name of the trajectory.
     *
     * @return the name of the trajectory.
     */
    public String name() {
        return name;
    }

    /**
     * Returns the samples of the trajectory.
     *
     * @return the samples of the trajectory.
     */
    public List<SampleType> samples() {
        return samples;
    }

    /**
     * Returns the indices of the splits in the trajectory.
     *
     * @return the indices of the splits in the trajectory.
     */
    public List<Integer> splits() {
        return splits;
    }

    /**
     * Returns the events in the trajectory.
     *
     * @return the events in the trajectory.
     */
    public List<EventMarker> events() {
        return events;
    }

    /**
     * Returns the first {@link SampleType} in the trajectory.
     *
     * <p><b>NULL SAFETY:</b> This function will return null if the trajectory is empty.
     *
     * @return The first {@link SampleType} in the trajectory.
     */
    public SampleType getInitialSample() {
        if (samples.isEmpty()) {
            return null;
        }
        return samples.get(0);
    }

    /**
     * Returns the last {@link SampleType} in the trajectory.
     *
     * <p><b>NULL SAFETY:</b> This function will return null if the trajectory is empty.
     *
     * @return The last {@link SampleType} in the trajectory.
     */
    public SampleType getFinalSample() {
        if (samples.isEmpty()) {
            return null;
        }
        return samples.get(samples.size() - 1);
    }

    private SampleType sampleInternal(double timestamp) {
        if (timestamp < samples.get(0).getTimestamp()) {
            // timestamp oob, return the initial state
            return getInitialSample();
        }
        if (timestamp >= getTotalTime()) {
            // timestamp oob, return the final state
            return getFinalSample();
        }

        // binary search to find the sample before and ahead of the timestamp
        int low = 0;
        int high = samples.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (samples.get(mid).getTimestamp() < timestamp) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        if (low == 0) {
            return samples.get(low);
        }

        SampleType behindState = samples.get(low - 1);
        SampleType aheadState = samples.get(low);

        if ((aheadState.getTimestamp() - behindState.getTimestamp()) < 1e-6) {
            return aheadState;
        }

        return behindState.interpolate(aheadState, timestamp);
    }

    /**
     * Return an interpolated sample of the trajectory at the given timestamp.
     *
     * <p><b>NULL SAFETY:</b> This function will return null if the trajectory is empty.
     *
     * @param timestamp The timestamp of this sample relative to the beginning of the trajectory.
     * @return The SampleType at the given time.
     */
    public SampleType sampleAt(double timestamp) {
        SampleType state;
        if (samples.isEmpty()) {
            return null;
        } else if (samples.size() == 1) {
            state = samples.get(0);
        } else {
            state = sampleInternal(timestamp);
        }
        return state;
    }

    /**
     * Returns the initial pose of the trajectory.
     *
     * <p><b>NULL SAFETY:</b> This function will return null if the trajectory is empty.
     *
     * @return the initial pose of the trajectory.
     */
    public Pose2D getInitialPose() {
        if (samples.isEmpty()) {
            return null;
        }
        return getInitialSample().getPose();
    }

    /**
     * Returns the final pose of the trajectory.
     *
     * <p><b>NULL SAFETY:</b> This function will return null if the trajectory is empty.
     *
     * @return the final pose of the trajectory.
     */
    public Pose2D getFinalPose() {
        if (samples.isEmpty()) {
            return null;
        }
        return getFinalSample().getPose();
    }

    /**
     * Returns the total time of the trajectory (the timestamp of the last sample)
     *
     * @return the total time of the trajectory (the timestamp of the last sample)
     */
    public double getTotalTime() {
        if (samples.isEmpty()) {
            return 0;
        }
        return getFinalSample().getTimestamp();
    }

    /**
     * Returns the array of poses corresponding to the trajectory.
     *
     * @return the array of poses corresponding to the trajectory.
     */
    public Pose2D[] getPoses() {
        return samples.stream().map(SampleType::getPose).toArray(Pose2D[]::new);
    }

    /**
     * Returns a list of all events with the given name in the trajectory.
     *
     * @param eventName The name of the event.
     * @return A list of all events with the given name in the trajectory, if no events are found, an
     * empty list is returned.
     */
    public List<EventMarker> getEvents(String eventName) {
        return events.stream().filter(event -> event.event.equals(eventName)).collect(Collectors.toList());
    }

    /**
     * Returns a choreo trajectory that represents the split of the trajectory at the given index.
     *
     * @param splitIndex the index of the split trajectory to return.
     * @return a choreo trajectory that represents the split of the trajectory at the given index.
     */
    public Optional<Trajectory<SampleType>> getSplit(int splitIndex) {
        if (splitIndex < 0 || splitIndex >= splits.size()) {
            return Optional.empty();
        }
        int start = splits.get(splitIndex);
        int end = splitIndex + 1 < splits.size() ? splits.get(splitIndex + 1) + 1 : samples.size();
        List<SampleType> sublist = samples.subList(start, end);
        double startTime = sublist.get(0).getTimestamp();
        double endTime = sublist.get(sublist.size() - 1).getTimestamp();
        return Optional.of(
                new Trajectory<SampleType>(
                        this.name + "[" + splitIndex + "]",
                        sublist.stream().map(s -> s.offsetBy(-startTime)).collect(Collectors.toList()),
                        List.of(),
                        events.stream()
                                .filter(e -> e.timestamp >= startTime && e.timestamp <= endTime)
                                .map(e -> e.offsetBy(-startTime))
                                .collect(Collectors.toList())));
    }

    /**
     * Retrieves the closest sample along the path to the given point
     *
     * @param pose The point to find the closest sample to
     * @return The closest sample to the given point
     */
    public SampleType getClosestSample(final Pose2D pose) {
        final double x = pose.getX(DistanceUnit.INCH);
        final double y = pose.getY(DistanceUnit.INCH);

        double minDistanceSquared = Double.MAX_VALUE;
        SampleType minDistanceSample = null;

        for (int i = 0; i < samples.size() - 1; i++) {
            final Pose2D start = samples.get(i).getPose();
            final Pose2D end = samples.get(i + 1).getPose();

            final Vector2d robotPos = new Vector2d(x, y);
            final Vector2d pathStart = new Vector2d(
                    start.getX(DistanceUnit.INCH),
                    start.getY(DistanceUnit.INCH)
            );

            final Vector2d pathEnd = new Vector2d(
                    end.getX(DistanceUnit.INCH),
                    end.getY(DistanceUnit.INCH)
            );

            final double length = pathEnd.distance(pathStart);
            final Vector2d diff = robotPos.sub(pathStart, new Vector2d());
            final Vector2d pathDirection = pathEnd.sub(pathStart, new Vector2d()).normalize();

            double distAlongSegment = diff.dot(pathDirection);

            if (distAlongSegment < 0) distAlongSegment = 0.0;
            if (distAlongSegment > length) distAlongSegment = length;

            Vector2d point = pathStart.fma(distAlongSegment, pathDirection, new Vector2d());

            final double distanceSquared = point.distanceSquared(robotPos);
            if (distanceSquared < minDistanceSquared) {
                minDistanceSquared = distanceSquared;
                SampleType startSample = this.samples.get(i);
                SampleType endSample = this.samples.get(i + 1);

                final double timestamp = startSample.getTimestamp() + (endSample.getTimestamp() - startSample.getTimestamp()) * distAlongSegment / length;
                minDistanceSample = startSample.interpolate(endSample, timestamp);
            }
        }

        return minDistanceSample;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Trajectory<?>)) {
            return false;
        }

        Trajectory<?> other = (Trajectory<?>) obj;
        return this.name.equals(other.name)
                && this.samples.equals(other.samples)
                && this.splits.equals(other.splits)
                && this.events.equals(other.events);
    }

    public List<SampleType> getSamples() {
        return this.samples;
    }
}
