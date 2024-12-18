// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve.util;

/** Add your docs here. */
public class Limiter 
{   
    /**
     * Clamps value [-1..1]
     * @param value Value to clamp
     * @return Clamped value
     */
    public static double clamp(double value)
    {
        return clamp(value, -1, 1);
    }

    /**
     * Clamps values to parameters
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    public static double clamp(double value, double min, double max)
    {
        return Math.min(Math.max(value, Math.min(min,max)), Math.max(min,max));
    }
}
