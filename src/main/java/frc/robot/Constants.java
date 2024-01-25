package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class ControllerConstants {
    public static final int CONTROLLER_PORT = 0;

    public static final double DEADBAND = 0.2; 
  }

  public static class DriveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 8;
    public static final int FRONT_LEFT_ROTATION_ID = 12;
    public static final int FRONT_LEFT_ROTATION_ENCODER_ID = 4;
    public static final double FRONT_LEFT_ANGLE_OFFSET = 1.836;
    public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERT = false;
    public static final boolean FRONT_LEFT_ROTATION_MOTOR_INVERT = true;
    public static final boolean FRONT_LEFT_ROTATION_ENCODER_INVERT = false;

    public static final int FRONT_RIGHT_DRIVE_ID = 7;
    public static final int FRONT_RIGHT_ROTATION_ID = 11;
    public static final int FRONT_RIGHT_ROTATION_ENCODER_ID = 3;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.029;
    public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERT = true;
    public static final boolean FRONT_RIGHT_ROTATION_MOTOR_INVERT = true;
    public static final boolean FRONT_RIGHT_ROTATION_ENCODER_INVERT = false;

    public static final int REAR_RIGHT_DRIVE_ID = 6;
    public static final int REAR_RIGHT_ROTATION_ID = 10;
    public static final int REAR_RIGHT_ROTATION_ENCODER_ID = 2;
    public static final double REAR_RIGHT_ANGLE_OFFSET = 3.015; // In radians
    public static final boolean REAR_RIGHT_DRIVE_MOTOR_INVERT = true;
    public static final boolean REAR_RIGHT_ROTATION_MOTOR_INVERT = true;
    public static final boolean REAR_RIGHT_ROTATION_ENCODER_INVERT = false;

    public static final int REAR_LEFT_DRIVE_ID = 9;
    public static final int REAR_LEFT_ROTATION_ID = 13;
    public static final int REAR_LEFT_ROTATION_ENCODER_ID = 5;
    public static final double REAR_LEFT_ANGLE_OFFSET = -0.721;
    public static final boolean REAR_LEFT_DRIVE_MOTOR_INVERT = false;
    public static final boolean REAR_LEFT_ROTATION_MOTOR_INVERT = true;
    public static final boolean REAR_LEFT_ROTATION_ENCODER_INVERT = false;

    // Declare location of Swerve Modules relative to robot center 
    // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.31, 0.31);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.31, -0.31);
    public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-0.31, -0.31);
    public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-0.31, 0.31);

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      // Translation constants
      new PIDConstants(DriveConstants.PATH_TRANSLATION_P, DriveConstants.PATH_TRANSLATION_I, DriveConstants.PATH_TRANSLATION_D),  
      // Rotation constants
      new PIDConstants(DriveConstants.PATH_ROTATION_P, DriveConstants.PATH_ROTATION_I, DriveConstants.PATH_ROTATION_D),  
      DriveConstants.MAX_DRIVE_SPEED, 
      DriveConstants.SWERVE_RADIUS, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );

    public static final int PIGEON_ID = 14;

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double SWERVE_RADIUS = 0.44; // m

    public static final double WHEEL_DIAMETER = 0.102;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 1/6.75;

    public static final double DRIVE_MOTOR_POSITION_TO_METERS = DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_MOTOR_VELOCITY_TO_METERS_SECOND = DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;

    public static final double MAX_DRIVE_SPEED = 4.97; // Max possible m/s
    public static final double MAX_DRIVE_SET_ACCELERATION = 10; // Max choosen m/s^2

    public static final double MAX_POSSIBLE_ROTATION_SPEED = MAX_DRIVE_SPEED / SWERVE_RADIUS;

    public static final double ROTATION_SPEED_SCALE_FACTOR = 0.75;

    public static final double MAX_SET_ROTATION_SPEED = MAX_POSSIBLE_ROTATION_SPEED * ROTATION_SPEED_SCALE_FACTOR;

    public static final double MOTOR_ROTATION_P = 0.1;
    public static final double MOTOR_ROTATION_I = 0.1; 
    public static final double MOTOR_ROTATION_D = 0.0;

    public static final double ROTATION_POSITION_CONTROL_P = 1.0; // Needs tuned
    public static final double ROTATION_POSITION_CONTROL_I = 5.0; 
    public static final double ROTATION_POSITION_CONTROL_D = 0.0;    

    public static final double PATH_TRANSLATION_P = 5.0;
    public static final double PATH_TRANSLATION_I = 0.0; 
    public static final double PATH_TRANSLATION_D = 0.0;

    public static final double PATH_ROTATION_P = 5.0;
    public static final double PATH_ROTATION_I = 0.0; 
    public static final double PATH_ROTATION_D = 0.0;

    public static final Pose2d ZERO_POSE = new Pose2d();
  }

  public static class LimelightConstants {
    public static final String LL_ONE = "limelight-llone";
    public static final String LL_TWO = "limelight-lltwo";

    public static final double VALID_TA_THRESHOLD = 0.0;
    
    public static final int POSE_ESTIMATOR_PIPELINE = 0;
    public static final int AIM_PIPELINE = 1;
    public static final int NOTE_DETECTION_PIPELINE = 2;
  }

  public static class PneumaticsConstants {
    public static final int LA_BIMBA_PORT = 0;
  }
}
