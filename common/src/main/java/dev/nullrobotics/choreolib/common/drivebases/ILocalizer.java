package dev.nullrobotics.choreolib.common.drivebases;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Represents a Localizer.
 */
public interface ILocalizer {
    Pose2D getPose();
    float getAngle();
}
