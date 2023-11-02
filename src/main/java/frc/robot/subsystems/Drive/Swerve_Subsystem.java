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
  private final Swerve_Module Swerve_1 = new Swerve_Module(
    Drive_Constants.Swerve_1_Drive_ID, 
    Drive_Constants.Swerve_1_Rotation_ID, 
    Drive_Constants.Swerve_1_Rotation_Encoder_ID, 
    Drive_Constants.Swerve_1_Angle_Offset/*,
    Drive_Constants.Swerve_1_Drive_Motor_Invert,
    Drive_Constants.SWerve_1_Rotation_Motor_Invert,
    Drive_Constants.Swerve_1_Rotation_Encoder_Invert*/);
  private final Swerve_Module Swerve_2 = new Swerve_Module(
    Drive_Constants.Swerve_2_Drive_ID, 
    Drive_Constants.Swerve_2_Rotation_ID, 
    Drive_Constants.Swerve_2_Rotation_Encoder_ID, 
    Drive_Constants.Swerve_2_Angle_Offset/*,
    Drive_Constants.Swerve_2_Drive_Motor_Invert,
    Drive_Constants.Swerve_2_Rotation_Motor_Invert,
    Drive_Constants.Swerve_2_Rotation_Encoder_Invert*/);
  private final Swerve_Module Swerve_3 = new Swerve_Module(    
    Drive_Constants.Swerve_3_Drive_ID, 
    Drive_Constants.Swerve_3_Rotation_ID, 
    Drive_Constants.Swerve_3_Rotation_Encoder_ID, 
    Drive_Constants.Swerve_3_Angle_Offset/*,
    Drive_Constants.Swerve_3_Drive_Motor_Invert,
    Drive_Constants.Swerve_3_Rotation_Motor_Invert,
    Drive_Constants.Swerve_3_Rotation_Encoder_Invert*/);
  private final Swerve_Module Swerve_4 = new Swerve_Module(    
    Drive_Constants.Swerve_4_Drive_ID, 
    Drive_Constants.Swerve_4_Rotation_ID, 
    Drive_Constants.Swerve_4_Rotation_Encoder_ID, 
    Drive_Constants.Swerve_4_Angle_Offset/*,
    Drive_Constants.Swerve_4_Drive_Motor_Invert,
    Drive_Constants.Swerve_4_Rotation_Motor_Invert,
    Drive_Constants.Swerve_4_Rotation_Encoder_Invert*/);

  // Declare location of Swerve Modules relative to robot center
  private final Translation2d Swerve_1_Location = new Translation2d(-0.31, 0.31); 
  private final Translation2d Swerve_2_Location = new Translation2d(0.31, 0.31);
  private final Translation2d Swerve_3_Location = new Translation2d(-0.31, -0.31);
  private final Translation2d Swerve_4_Location = new Translation2d(0.31, -0.31);
  
  // Declare Swerve Kinematics using Swerve Module locations
  private final SwerveDriveKinematics Swerve = new SwerveDriveKinematics(
    Swerve_1_Location, 
    Swerve_2_Location, 
    Swerve_3_Location, 
    Swerve_4_Location);

  private ChassisSpeeds Swerve_Speeds = new ChassisSpeeds(); // Declare Chassis Speed for use in methods

  private final Pigeon2 Pigeon = new Pigeon2(Drive_Constants.Pigeon_ID, "canivore"); // Declare IMU

  //  Declare Swerve Module POsitions for SWerve Odometry
  /*private SwerveModulePosition Swerve_1_Position = new SwerveModulePosition(Swerve_1.Get_Drive_Position(), Swerve_1.Get_Swerve_State().angle);
  private SwerveModulePosition Swerve_2_Position = new SwerveModulePosition(Swerve_2.Get_Drive_Position(), Swerve_2.Get_Swerve_State().angle);
  private SwerveModulePosition Swerve_3_Position = new SwerveModulePosition(Swerve_3.Get_Drive_Position(), Swerve_3.Get_Swerve_State().angle);
  private SwerveModulePosition Swerve_4_Position = new SwerveModulePosition(Swerve_4.Get_Drive_Position(), Swerve_4.Get_Swerve_State().angle);

  private SwerveModulePosition[] Swerve_Positions = {Swerve_1_Position, Swerve_2_Position, Swerve_3_Position, Swerve_4_Position};

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
    Swerve_1.Set_Swerve_State(Swerve_Module_States[0]);
    // Swerve_2.Set_Swerve_State(Swerve_Module_States[1]);
    // Swerve_3.Set_Swerve_State(Swerve_Module_States[2]);
    // Swerve_4.Set_Swerve_State(Swerve_Module_States[3]);
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
    /*Swerve_1_Position.distanceMeters = Swerve_1.Get_Drive_Position();
    Swerve_1_Position.angle = Swerve_1.Get_Swerve_State().angle;
    Swerve_Positions[0] = Swerve_1_Position;
    
    Swerve_2_Position.distanceMeters = Swerve_2.Get_Drive_Position();
    Swerve_2_Position.angle = Swerve_2.Get_Swerve_State().angle;
    Swerve_Positions[1] = Swerve_2_Position;
    
    Swerve_3_Position.distanceMeters = Swerve_3.Get_Drive_Position();
    Swerve_3_Position.angle = Swerve_3.Get_Swerve_State().angle;
    Swerve_Positions[2] = Swerve_3_Position;
    
    Swerve_4_Position.distanceMeters = Swerve_4.Get_Drive_Position();
    Swerve_4_Position.angle = Swerve_4.Get_Swerve_State().angle;
    Swerve_Positions[3] = Swerve_4_Position;

    Swerve_Odometry.update(Rotation2d.fromRadians(Get_Yaw()), Swerve_Positions);*/
  }

  @Override
  public void simulationPeriodic() {}
}
