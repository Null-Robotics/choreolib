package dev.nullrobotics.choreolib.common.sample;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nullrobotics.choreolib.common.trajectory.TrajectorySample;

/**
 * A single swerve robot sample in a Trajectory.
 */
public class SwerveSample implements TrajectorySample<SwerveSample> {
    private static final double[] EMPTY_MODULE_FORCES = new double[]{0, 0, 0, 0};

    /**
     * The timestamp of this sample, relative to the beginning of the trajectory.
     */
    public final double t;

    /**
     * The X position of the sample relative to the blue alliance wall origin in meters.
     */
    public final double x;

    /**
     * The Y position of the sample relative to the blue alliance wall origin in meters.
     */
    public final double y;

    /**
     * The heading of the sample in radians, with 0 being in the +X direction.
     */
    public final double heading;

    /**
     * The velocity of the sample in the X direction in m/s.
     */
    public final double vx;

    /**
     * The velocity of the sample in the Y direction in m/s.
     */
    public final double vy;

    /**
     * The angular velocity of the sample in rad/s.
     */
    public final double omega;

    /**
     * The acceleration of the in the X direction in m/s².
     */
    public final double ax;

    /**
     * The acceleration of the in the Y direction in m/s².
     */
    public final double ay;

    /**
     * The angular acceleration of the sample in rad/s².
     */
    public final double alpha;

    /**
     * The force on each swerve module in the X direction in Newtons. Module forces appear in the
     * following order: [FL, FR, BL, BR].
     */
    private final double[] fx;

    /**
     * The force on each swerve module in the Y direction in Newtons Module forces appear in the
     * following order: [FL, FR, BL, BR].
     */
    private final double[] fy;

    /**
     * Constructs a SwerveSample with the specified parameters.
     *
     * @param t             The timestamp of this sample, relative to the beginning of the trajectory.
     * @param x             The X position of the sample in meters.
     * @param y             The Y position of the sample in meters.
     * @param heading       The heading of the sample in radians, with 0 being in the +X direction.
     * @param vx            The velocity of the sample in the X direction in m/s.
     * @param vy            The velocity of the sample in the Y direction in m/s.
     * @param omega         The angular velocity of the sample in rad/s.
     * @param ax            The acceleration of the sample in the X direction in m/s².
     * @param ay            The acceleration of the sample in the Y direction in m/s².
     * @param alpha         The angular acceleration of the sample in rad/s².
     * @param moduleForcesX The force on each swerve module in the X direction in Newtons. Module
     *                      forces appear in the following order: [FL, FR, BL, BR].
     * @param moduleForcesY The force on each swerve module in the Y direction in Newtons. Module
     *                      forces appear in the following order: [FL, FR, BL, BR].
     */
    public SwerveSample(
            double t,
            double x,
            double y,
            double heading,
            double vx,
            double vy,
            double omega,
            double ax,
            double ay,
            double alpha,
            double[] moduleForcesX,
            double[] moduleForcesY) {
        this.t = t;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
        this.fx = moduleForcesX;
        this.fy = moduleForcesY;
    }

    /**
     * A null safe getter for the module forces in the X direction.
     *
     * @return The module forces in the X direction.
     */
    public double[] moduleForcesX() {
        if (fx == null || fx.length != 4) {
            return EMPTY_MODULE_FORCES;
        }
        return fx;
    }

    /**
     * A null safe getter for the module forces in the Y direction.
     *
     * @return The module forces in the Y direction.
     */
    public double[] moduleForcesY() {
        if (fy == null || fy.length != 4) {
            return EMPTY_MODULE_FORCES;
        }
        return fy;
    }

    @Override
    public double getTimestamp() {
        return t;
    }

    @Override
    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, heading);
    }

    @Override
    public SwerveSample interpolate(SwerveSample endValue, double timestamp) {
        double scale = (timestamp - this.t) / (endValue.t - this.t);

        return new SwerveSample(
                timestamp,
                this.x + (endValue.x - this.x) * scale,
                this.y + (endValue.y - this.y) * scale,
                this.heading + (endValue.heading - this.heading) * scale,
                this.vx + (endValue.vx - this.vx) * scale,
                this.vy + (endValue.vy - this.vy) * scale,
                this.omega + (endValue.omega - this.omega) * scale,
                this.ax + (endValue.ax - this.ax) * scale,
                this.ay + (endValue.ay - this.ay) * scale,
                this.alpha + (endValue.alpha - this.alpha) * scale,
                new double[]{
                        this.fx[0] + (endValue.fx[0] - this.fx[0]) * scale,
                        this.fx[1] + (endValue.fx[1] - this.fx[1]) * scale,
                        this.fx[2] + (endValue.fx[2] - this.fx[2]) * scale,
                        this.fx[3] + (endValue.fx[3] - this.fx[3]) * scale
                },
                new double[]{
                        this.fy[0] + (endValue.fy[0] - this.fy[0]) * scale,
                        this.fy[1] + (endValue.fy[1] - this.fy[1]) * scale,
                        this.fy[2] + (endValue.fy[2] - this.fy[2]) * scale,
                        this.fy[3] + (endValue.fy[3] - this.fy[3]) * scale
                });
    }

    @Override
    public SwerveSample offsetBy(double timestampOffset) {
        return new SwerveSample(
                this.t + timestampOffset,
                this.x,
                this.y,
                this.heading,
                this.vx,
                this.vy,
                this.omega,
                this.ax,
                this.ay,
                this.alpha,
                this.moduleForcesX(),
                this.moduleForcesY());
    }

}