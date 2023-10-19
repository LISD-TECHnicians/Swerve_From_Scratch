package frc.robot;

public final class Constants {
  public final class Drive_Constants {
    public static final int Swerve_1_Drive_ID = 1;
    public static final int Swerve_1_Rotation_ID = 2;
    public static final int Swerve_2_Drive_ID = 3;
    public static final int Swerve_2_Rotation_ID = 4;
    public static final int Swerve_3_Drive_ID = 5;
    public static final int Swerve_3_Rotation_ID = 6;
    public static final int Swerve_4_Drive_ID = 7;
    public static final int Swerve_4_Rotation_ID = 8;

    public static final int Swerve_1_Rotation_Encoder_ID = 9;
    public static final int Swerve_2_Rotation_Encoder_ID = 10;
    public static final int Swerve_3_Rotation_Encoder_ID = 11;
    public static final int Swerve_4_Rotation_Encoder_ID = 12;

    public static final int Pigeon_ID = 0;

    public static final double Operating_Voltage = 12; // For more reliable and precise motors speeds

    public static final double Max_Drive_Speed = 20; // Max possible
    public static final double Max_Drive_Set_Acceleration = 5; // Max choosen

    public static final double Max_Rotation_Speed = 0.25 * Math.PI; // Max possible
    public static final double Max_Rotation_Motor_Set_Speed = 0.25; // Max choosen

    public static final double Drive_Gear_Ratio = 0.01;
    public static final double Rotation_Gear_Ratio = 0.01;

    public static final double Rotation_Speed_Scale_Factor = 0.25;

    public static final double Rotation_P = 3.5;
    public static final double Rotation_I = 0.0; 
    public static final double Rotation_D = 0.0;
  }
}
