package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {
  public final class Controller_Constants {
    public static final int Controller_Port = 3;
  }

  public static final class Drive_Constants {
    public static final int Front_Left_Drive_ID = 8;
    public static final int Front_Left_Rotation_ID = 12;
    public static final int Front_Left_Rotation_Encoder_ID = 4;
    public static final double Front_Left_Angle_Offset = 4.987 + Math.PI/2;
    public static final boolean Front_Left_Drive_Motor_Invert = false;
    public static final boolean Front_Left_Rotation_Motor_Invert = false;
    public static final boolean Front_Left_Rotation_Encoder_Invert = true;

    public static final int Front_Right_Drive_ID = 7;
    public static final int Front_Right_Rotation_ID = 11;
    public static final int Front_Right_Rotation_Encoder_ID = 3;
    public static final double Front_Right_Angle_Offset = 2.014 + Math.PI/2;
    public static final boolean Front_Right_Drive_Motor_Invert = false;
    public static final boolean Front_Right_Rotation_Motor_Invert = false;
    public static final boolean Front_Right_Rotation_Encoder_Invert = true;

    public static final int Rear_Right_Drive_ID = 6;
    public static final int Rear_Right_Rotation_ID = 10;
    public static final int Rear_Right_Rotation_Encoder_ID = 2;
    public static final double Rear_Right_Angle_Offset = 3.034 + Math.PI/2; // In radians
    public static final boolean Rear_Right_Drive_Motor_Invert = false;
    public static final boolean Rear_Right_Rotation_Motor_Invert = false;
    public static final boolean Rear_Right_Rotation_Encoder_Invert = true;

    public static final int Rear_Left_Drive_ID = 9;
    public static final int Rear_Left_Rotation_ID = 13;
    public static final int Rear_Left_Rotation_Encoder_ID = 5;
    public static final double Rear_Left_Angle_Offset = 5.556 + Math.PI/2;
    public static final boolean Rear_Left_Drive_Motor_Invert = true;
    public static final boolean Rear_Left_Rotation_Motor_Invert = false;
    public static final boolean Rear_Left_Rotation_Encoder_Invert = true;

    // Declare location of Swerve Modules relative to robot center 
    // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
    public static final Translation2d Front_Left_Location = new Translation2d(0.31, 0.31);
    public static final Translation2d Front_Right_Location = new Translation2d(0.31, -0.31);
    public static final Translation2d Rear_Right_Location = new Translation2d(-0.31, -0.31);
    public static final Translation2d Rear_Left_Location = new Translation2d(-0.31, 0.31);

    public static final HolonomicPathFollowerConfig PathFolowConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      new PIDConstants(Drive_Constants.Path_Translation_P, Drive_Constants.Path_Translation_I, Drive_Constants.Path_Translation_D), // Translation PID constants
      new PIDConstants(Drive_Constants.Path_Rotation_P, Drive_Constants.Path_Rotation_I, Drive_Constants.Path_Rotation_D), // Rotation PID constants
      Drive_Constants.Max_Drive_Speed, // Max module speed, in m/s
      Drive_Constants.Swerve_Radius, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig()); // Default path replanning config. See the API for the options here

    public static final int Pigeon_ID = 14;

    public static final double Nominal_Voltage = 12.0;

    public static final double Swerve_Radius = 0.44; // m

    public static final double Wheel_Diameter = 0.102;
    public static final double Wheel_Circumference = Wheel_Diameter * Math.PI;

    public static final double Ticks_Per_Revolution_Falcon = 2048;
    public static final double Drive_Gear_Ratio = 1/6.75;

    public static final double Drive_Motor_Position_To_Meters = (1/Ticks_Per_Revolution_Falcon) * Drive_Gear_Ratio * Wheel_Circumference;
    public static final double Drive_Motor_Velcoity_to_Meters_Second = 10 * (1/Ticks_Per_Revolution_Falcon) * Drive_Gear_Ratio * Wheel_Circumference;

    public static final double Max_Drive_Speed = 4.97; // Max possible m/s
    public static final double Max_Drive_Set_Acceleration = 10; // Max choosen m/s^2

    public static final double Max_Possible_Rotation_Speed = Max_Drive_Speed / Swerve_Radius;

    public static final double Rotation_Speed_Scale_Factor = 0.75;

    public static final double Max_Set_Rotation_Speed = Max_Possible_Rotation_Speed * Rotation_Speed_Scale_Factor;

    public static final double Motor_Rotation_P = 0.1;
    public static final double Motor_Rotation_I = 0.1; 
    public static final double Motor_Rotation_D = 0.0;

    public static final double Rotation_Position_Control_P = 1.0; // Needs tuned
    public static final double Rotation_Position_Control_I = 5.0; 
    public static final double Rotation_Position_Control_D = 0.0;    

    public static final double Path_Translation_P = 5.0;
    public static final double Path_Translation_I = 0.0; 
    public static final double Path_Translation_D = 0.0;

    public static final double Path_Rotation_P = 5.0;
    public static final double Path_Rotation_I = 0.0; 
    public static final double Path_Rotation_D = 0.0;

    public static final Pose2d Zero_Pose = new Pose2d();
  }

  public final class Pneumatics_Constants {
    public static final int LaBimba_Port = 0;
  }
}
