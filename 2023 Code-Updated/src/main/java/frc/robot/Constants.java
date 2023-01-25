// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

//Arm Motors
    public static final byte SHOULDER_MOTOR = 0;
    public static final byte ELBOW_MOTOR = 0;
    public static final byte HAND_MOTOR = 0;
    public static final byte RETRACTABLE_ARM_MOTOR = 0;

    //joysticks
    public static final byte LEFT_JOYSTICK = 0;
    public static final byte MID_JOYSTICK = 1;
    public static final byte RIGHT_JOYSTICK = 2;

    //apriltag variables
    public static final double WANTED_DISTANCE = 0;
    public static final double WANTED_YAW = 0;
    public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0;
    public static final double WANTED_SKEW = 0;
}