package frc.robot;

public final class Constants {
  public final class Drive_Constants {
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

    public static final double Max_Rotation_Speed = Max_Drive_Speed / Swerve_Radius;

    public static final double Rotation_Speed_Scale_Factor = 0.75;

    public static final double Rotation_P = 0.1;
    public static final double Rotation_I = 0.0; 
    public static final double Rotation_D = 0.0;
  }
}
