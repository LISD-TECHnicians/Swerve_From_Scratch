package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {
  public final class ControllerConstants {
    public static final int controllerPort = 3;
  }

  public static final class DriveConstants {
    public static final int frontLeftDriveID = 8;
    public static final int frontLeftRotationID = 12;
    public static final int frontLeftRotationEncoderID = 4;
    public static final double frontLeftAngleOffset = 4.987 + Math.PI/2;
    public static final boolean frontLeftDriveMotorInvert = false;
    public static final boolean frontLeftRotationMotorInvert = false;
    public static final boolean frontLeftRotationEncoderInvert = true;

    public static final int frontRightDriveID = 7;
    public static final int frontRightRotationID = 11;
    public static final int frontRightRotationEncoderID = 3;
    public static final double frontRightAngleOffset = 2.014 + Math.PI/2;
    public static final boolean frontRightDriveMotorInvert = false;
    public static final boolean frontRightRotationMotorInvert = false;
    public static final boolean frontRightRotationEncoderInvert = true;

    public static final int rearRightDriveID = 6;
    public static final int rearRightRotationID = 10;
    public static final int rearRightRotationEncoderID = 2;
    public static final double rearRightAngleOffset = 3.034 + Math.PI/2; // In radians
    public static final boolean rearRightDriveMotorInvert = false;
    public static final boolean rearRightRotationMotorInvert = false;
    public static final boolean rearRightRotationEncoderInvert = true;

    public static final int rearLeftDriveID = 9;
    public static final int rearLeftRotationID = 13;
    public static final int rearLeftRotationEncoderID = 5;
    public static final double rearLeftAngleOffset = 5.556 + Math.PI/2;
    public static final boolean rearLeftDriveMotorInvert = true;
    public static final boolean rearLeftRotationMotorInvert = false;
    public static final boolean rearLeftRotationEncoderInvert = true;

    // Declare location of Swerve Modules relative to robot center 
    // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
    public static final Translation2d frontLeftLocation = new Translation2d(0.31, 0.31);
    public static final Translation2d frontRightLocation = new Translation2d(0.31, -0.31);
    public static final Translation2d rearRightLocation = new Translation2d(-0.31, -0.31);
    public static final Translation2d rearLeftLocation = new Translation2d(-0.31, 0.31);

    public static final HolonomicPathFollowerConfig PathFolowConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      new PIDConstants(DriveConstants.pathTranslationP, DriveConstants.pathTranslationI, DriveConstants.pathTranslationD), // Translation PID constants
      new PIDConstants(DriveConstants.pathRotationP, DriveConstants.pathRotationI, DriveConstants.pathRotationD), // Rotation PID constants
      DriveConstants.maxDriveSpeed, // Max module speed, in m/s
      DriveConstants.swerveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig()); // Default path replanning config. See the API for the options here

    public static final int pigeonID = 14;

    public static final double nominalVoltage = 12.0;

    public static final double swerveRadius = 0.44; // m

    public static final double wheelDiameter = 0.102;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double ticksPerRevolutionFalcon = 2048;
    public static final double driveGearRatio = 1/6.75;

    public static final double driveMotorPositionToMeters = (1/ticksPerRevolutionFalcon) * driveGearRatio * wheelCircumference;
    public static final double driveMotorVelcoityToMetersSecond = 10 * (1/ticksPerRevolutionFalcon) * driveGearRatio * wheelCircumference;

    public static final double maxDriveSpeed = 4.97; // Max possible m/s
    public static final double maxDriveSetAcceleration = 10; // Max choosen m/s^2

    public static final double maxPossibleRotationSpeed = maxDriveSpeed / swerveRadius;

    public static final double rotationSpeedScaleFactor = 0.75;

    public static final double maxSetRotationSpeed = maxPossibleRotationSpeed * rotationSpeedScaleFactor;

    public static final double motorRotationP = 0.1;
    public static final double motorRotationI = 0.1; 
    public static final double motorRotationD = 0.0;

    public static final double rotationPositionControlP = 1.0; // Needs tuned
    public static final double rotationPositionControlI = 5.0; 
    public static final double rotationPositionControlD = 0.0;    

    public static final double pathTranslationP = 5.0;
    public static final double pathTranslationI = 0.0; 
    public static final double pathTranslationD = 0.0;

    public static final double pathRotationP = 5.0;
    public static final double pathRotationI = 0.0; 
    public static final double pathRotationD = 0.0;

    public static final Pose2d zeroPose = new Pose2d();
  }

  public final class PneumaticsConstants {
    public static final int laBimbaPort = 0;
  }
}
