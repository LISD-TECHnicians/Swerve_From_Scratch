package frc.robot;

public final class Constants {
  public final class Drive_Constants {
    public static final int Swerve_1_Drive_ID = 6;
    public static final int Swerve_1_Rotation_ID = 10;
    public static final int Swerve_1_Rotation_Encoder_ID = 2;
    public static final double Swerve_1_Angle_Offset = 102; // In radians
    public static final boolean Swerve_1_Drive_Motor_Invert = false;
    public static final boolean Swerve_1_Rotation_Motor_Invert = false;
    public static final boolean Swerve_1_Rotation_Encoder_Invert = false;

    public static final int Swerve_2_Drive_ID = 7;
    public static final int Swerve_2_Rotation_ID = 11;
    public static final int Swerve_2_Rotation_Encoder_ID = 3;
    public static final double Swerve_2_Angle_Offset = 32;
    public static final boolean Swerve_2_Drive_Motor_Invert = false;
    public static final boolean Swerve_2_Rotation_Motor_Invert = false;
    public static final boolean Swerve_2_Rotation_Encoder_Invert = false;
    
    public static final int Swerve_3_Drive_ID = 8;
    public static final int Swerve_3_Rotation_ID = 12;
    public static final int Swerve_3_Rotation_Encoder_ID = 4;
    public static final double Swerve_3_Angle_Offset = 32;
    public static final boolean Swerve_3_Drive_Motor_Invert = false;
    public static final boolean Swerve_3_Rotation_Motor_Invert = false;
    public static final boolean Swerve_3_Rotation_Encoder_Invert = false;

    public static final int Swerve_4_Drive_ID = 9;
    public static final int Swerve_4_Rotation_ID = 13;
    public static final int Swerve_4_Rotation_Encoder_ID = 5;
    public static final double Swerve_4_Angle_Offset = 32;
    public static final boolean Swerve_4_Drive_Motor_Invert = false;
    public static final boolean Swerve_4_Rotation_Motor_Invert = false;
    public static final boolean Swerve_4_Rotation_Encoder_Invert = false;

    public static final int Pigeon_ID = 14;

    public static final double Swerve_Radius = 0.5; // m

    public static final double Max_Drive_Speed = 5; // Max possible m/s
    public static final double Max_Drive_Set_Acceleration = 5; // Max choosen m/s^2

    public static final double Max_Rotation_Speed = Max_Drive_Speed / Swerve_Radius;

    public static final double Drive_Gear_Ratio = 1/5;

    public static final double Rotation_Speed_Scale_Factor = 0.25;

    public static final double Rotation_P = 0.05;
    public static final double Rotation_I = 0.0; 
    public static final double Rotation_D = 0.0;
  }
}
