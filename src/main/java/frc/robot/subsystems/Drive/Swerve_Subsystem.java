package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Drive_Constants;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Swerve_Subsystem extends SubsystemBase {
  // Declare all Swerve Modules
  private final Swerve_Module Front_Left_Swerve = new Swerve_Module(    
    Drive_Constants.Front_Left_Drive_ID, 
    Drive_Constants.Front_Left_Rotation_ID, 
    Drive_Constants.Front_Left_Rotation_Encoder_ID, 
    Drive_Constants.Front_Left_Angle_Offset,
    Drive_Constants.Front_Left_Drive_Motor_Invert,
    Drive_Constants.Front_Left_Rotation_Motor_Invert,
    Drive_Constants.Front_Left_Rotation_Encoder_Invert);
  private final Swerve_Module Front_Right_Swerve = new Swerve_Module(
    Drive_Constants.Front_Right_Drive_ID, 
    Drive_Constants.Front_Right_Rotation_ID, 
    Drive_Constants.Front_Right_Rotation_Encoder_ID, 
    Drive_Constants.Front_Right_Angle_Offset,
    Drive_Constants.Front_Right_Drive_Motor_Invert,
    Drive_Constants.Front_Right_Rotation_Motor_Invert,
    Drive_Constants.Front_Right_Rotation_Encoder_Invert);
  private final Swerve_Module Rear_Right_Swerve = new Swerve_Module(
    Drive_Constants.Rear_Right_Drive_ID, 
    Drive_Constants.Rear_Right_Rotation_ID, 
    Drive_Constants.Rear_Right_Rotation_Encoder_ID, 
    Drive_Constants.Rear_Right_Angle_Offset,
    Drive_Constants.Rear_Right_Drive_Motor_Invert,
    Drive_Constants.Rear_Right_Rotation_Motor_Invert,
    Drive_Constants.Rear_Right_Rotation_Encoder_Invert);
  private final Swerve_Module Rear_Left_Swerve = new Swerve_Module(    
    Drive_Constants.Rear_Left_Drive_ID, 
    Drive_Constants.Rear_Left_Rotation_ID, 
    Drive_Constants.Rear_Left_Rotation_Encoder_ID, 
    Drive_Constants.Rear_Left_Angle_Offset,
    Drive_Constants.Rear_Left_Drive_Motor_Invert,
    Drive_Constants.Rear_Left_Rotation_Motor_Invert,
    Drive_Constants.Rear_Left_Rotation_Encoder_Invert);

  // Declare location of Swerve Modules relative to robot center 
  private final Translation2d Front_Left_Location = new Translation2d(0.31, 0.31);
  private final Translation2d Front_Right_Location = new Translation2d(0.31, -0.31);
  private final Translation2d Rear_Right_Location = new Translation2d(-0.31, -0.31);
  private final Translation2d Rear_Left_Location = new Translation2d(-0.31, 0.31);
  
  // Declare Swerve Kinematics using Swerve Module locations
  // MUST PASS MODULES CLOCKWISE FROM FRONT LEFT
  // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
  private final SwerveDriveKinematics Swerve = new SwerveDriveKinematics( 
    Front_Left_Location, 
    Front_Right_Location, 
    Rear_Right_Location, 
    Rear_Left_Location);

  private ChassisSpeeds Swerve_Speeds = new ChassisSpeeds(); // Declare Chassis Speed for use in methods

  private final Pigeon2 Pigeon = new Pigeon2(Drive_Constants.Pigeon_ID, "canivore"); // Declare IMU

  //  Declare Swerve Module POsitions for SWerve Odometry
  /*private SwerveModulePosition Rear_Right_Position = new SwerveModulePosition(Rear_Right.Get_Drive_Position(), Rear_Right.Get_Swerve_State().angle);
  private SwerveModulePosition Front_Right_Position = new SwerveModulePosition(Front_Right.Get_Drive_Position(), Front_Right.Get_Swerve_State().angle);
  private SwerveModulePosition Front_Left_Position = new SwerveModulePosition(Front_Left.Get_Drive_Position(), Front_Left.Get_Swerve_State().angle);
  private SwerveModulePosition Rear_Left_Position = new SwerveModulePosition(Rear_Left.Get_Drive_Position(), Rear_Left.Get_Swerve_State().angle);

  private SwerveModulePosition[] Swerve_Positions = {Rear_Right_Position, Front_Right_Position, Front_Left_Position, Rear_Left_Position};

  // Declare Swerve Odometry
  private final SwerveDriveOdometry Swerve_Odometry = new SwerveDriveOdometry(Swerve, Rotation2d.fromRadians(Get_Yaw()), Swerve_Positions);*/

  public Swerve_Subsystem() {
    Pigeon.configFactoryDefault();

    Pigeon.configMountPose(0, 0, 0);
  }

  public void Run_Swerve(double X_Speed, double Y_Speed, double Rotation_Speed/*, boolean Field_Oriented*/) {
    // Use given speeds to get Chassis Speed
    if (false /*Field_Oriented*/) {
      Swerve_Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(X_Speed, Y_Speed, Rotation_Speed, Rotation2d.fromRadians(Get_Yaw()));
    }
    else {
      Swerve_Speeds.vxMetersPerSecond = X_Speed;
      Swerve_Speeds.vyMetersPerSecond = Y_Speed;
      Swerve_Speeds.omegaRadiansPerSecond = Rotation_Speed;
    }

    // List of Swerve States from desired Swerve Speeds
    SwerveModuleState[] Swerve_Module_States = Swerve.toSwerveModuleStates(Swerve_Speeds);  

    SwerveDriveKinematics.desaturateWheelSpeeds(Swerve_Module_States, Drive_Constants.Max_Drive_Speed); // Keeps motor speeds in limits

    // Set each Swerve State
    Rear_Right_Swerve.Set_Swerve_State(Swerve_Module_States[0]);
    Front_Right_Swerve.Set_Swerve_State(Swerve_Module_States[1]);
    Front_Left_Swerve.Set_Swerve_State(Swerve_Module_States[2]);
    Rear_Left_Swerve.Set_Swerve_State(Swerve_Module_States[3]);
  }

  public double Get_Yaw() {
    return Units.degreesToRadians(Pigeon.getYaw());
  }

  public double Get_Pitch() {
    return Units.degreesToRadians(Pigeon.getPitch());
  }

  public double Get_Roll() {
    return Units.degreesToRadians(Pigeon.getRoll());
  }
  
  // public Pose2d Get_Pose() {
  //   return Swerve_Odometry.getPoseMeters();
  // }

  @Override
  public void periodic() {
    /*Rear_Right_Position.distanceMeters = Rear_Right.Get_Drive_Position();
    Rear_Right_Position.angle = Rear_Right.Get_Swerve_State().angle;
    Swerve_Positions[0] = Rear_Right_Position;
    
    Front_Right_Position.distanceMeters = Front_Right.Get_Drive_Position();
    Front_Right_Position.angle = Front_Right.Get_Swerve_State().angle;
    Swerve_Positions[1] = Front_Right_Position;
    
    Front_Left_Position.distanceMeters = Front_Left.Get_Drive_Position();
    Front_Left_Position.angle = Front_Left.Get_Swerve_State().angle;
    Swerve_Positions[2] = Front_Left_Position;
    
    Rear_Left_Position.distanceMeters = Rear_Left.Get_Drive_Position();
    Rear_Left_Position.angle = Rear_Left.Get_Swerve_State().angle;
    Swerve_Positions[3] = Rear_Left_Position;

    Swerve_Odometry.update(Rotation2d.fromRadians(Get_Yaw()), Swerve_Positions);*/

    // System.out.println("Cancoder 1; " + Rear_Right.Get_Rotation_Position());
    // System.out.println("Cancoder 2; " + Front_Right.Get_Rotation_Position());
    // System.out.println("Cancoder 3; " + Front_Left.Get_Rotation_Position());
    // System.out.println("Cancoder 4; " + Rear_Left.Get_Rotation_Position());
  }

  @Override
  public void simulationPeriodic() {}
}
