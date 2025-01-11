package dev.nullrobotics.choreolib.common.drivebases;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.joml.Matrix2d;
import org.joml.Matrix2dc;
import org.joml.Matrix4f;
import org.joml.Vector2d;
import org.joml.Vector3f;

/**
 * Represents an FTC Drivetrain, provides default methods for driving field relative and robot relative.
 */
public interface IDriveable {
    void set(Vector3f movement, double turnSpeed);

    /**
     * Provides a method for setting drive powers using JOML's Vector3f in a field relative environment.
     * @param movement The pose we are trying to move to, relative to the robot's current position.
     * @param turnSpeed The powers we are attempting to set for turning while performing this movement
     * @param angle The current angle of the robot, in radians.
     */
    default void setField(Vector3f movement, double turnSpeed, float angle) {
        Matrix4f identity = new Matrix4f().identity();
        identity.rotate(angle, 1, 1, 0);

        Vector3f movementVector = identity.transformPosition(movement);

        set(movementVector, turnSpeed);
    }

    /**
     * Provides a method for driving with the FTC SDK's Pose2D class.
     * @param movement The pose we are trying to move to, relative to the robot's current position.
     * @param turnSpeed The powers we are attempting to set for turning while performing this movement
     */
    default void drive(Pose2D movement, double turnSpeed) {
        Vector3f movementVector = new Vector3f((float) movement.getX(DistanceUnit.INCH), (float) movement.getY(DistanceUnit.INCH), 0);
        set(movementVector, turnSpeed);
    }

    /**
     * Provides a method for driving with the FTC SDK's Pose2D class in a field relative environment.
     * @param movement The pose we are trying to move to, relative to the robot's current position.
     * @param turnSpeed The powers we are attempting to set for turning while performing this movement
     * @param angle The current angle of the robot, in radians.
     */
    default void driveField(Pose2D movement, double turnSpeed, float angle) {
        Vector3f movementVector = new Vector3f((float) movement.getX(DistanceUnit.INCH), (float) movement.getY(DistanceUnit.INCH), 0);
        setField(movementVector, turnSpeed, angle);
    }
}
