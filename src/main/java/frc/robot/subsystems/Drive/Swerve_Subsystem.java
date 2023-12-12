package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.Drive_Constants;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;

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
  
  // Declare Swerve Kinematics using Swerve Module locations
  private final SwerveDriveKinematics Swerve = new SwerveDriveKinematics( 
    Drive_Constants.Front_Left_Location, 
    Drive_Constants.Front_Right_Location, 
    Drive_Constants.Rear_Right_Location, 
    Drive_Constants.Rear_Left_Location);

  private ChassisSpeeds Swerve_Speeds = new ChassisSpeeds(); // Declare Chassis Speed for use in methods
  
  private final Pose2d Initial_Pose = new Pose2d(1.0, 1.0, Rotation2d.fromRadians(0.0));

  //  Declare Swerve Module Positions for SWerve Odometry
  private SwerveModulePosition Front_Left_Position = new SwerveModulePosition();
  private SwerveModulePosition Front_Right_Position = new SwerveModulePosition();
  private SwerveModulePosition Rear_Right_Position = new SwerveModulePosition();
  private SwerveModulePosition Rear_Left_Position = new SwerveModulePosition();

  private SwerveModulePosition[] Swerve_Positions = {Front_Left_Position, Front_Right_Position, Rear_Right_Position, Rear_Left_Position};

  // Declare Swerve Odometry
  private final SwerveDriveOdometry Swerve_Odometry = new SwerveDriveOdometry(Swerve, Rotation2d.fromRadians(0.0), Swerve_Positions, Initial_Pose);

  private final Pigeon2 Pigeon = new Pigeon2(Drive_Constants.Pigeon_ID, "canivore"); // Declare IMU

  private final GenericEntry Yaw_Entry = RobotContainer.Robot_Status.add("Heading", 0).getEntry(); //  Elastic Test
  private final GenericEntry Speed_Entry = RobotContainer.Robot_Status.add("Speed", 0).getEntry();

  private final GenericEntry Slider = RobotContainer.Robot_Status.add("Slider", 0).getEntry();

  private final Field2d Field_Layout = new Field2d();

  public Swerve_Subsystem() {
    Pigeon.configFactoryDefault();

    Pigeon.configMountPose(0, 0, 0);
    
    // Allows PathPlanner to construct trajectories on its own
    AutoBuilder.configureHolonomic(
      this::Get_Pose, // Robot pose supplier
      this::Reset_Pose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::Get_Current_ChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::Run_Swerve, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      Drive_Constants.PathFolowConfig,
      this // Reference to this subsystem to set requirements
    );
  }

  public void Run_Swerve(ChassisSpeeds Chassis_Speeds) {
    // List of Swerve States from desired Swerve Speeds
    Swerve_Speeds = Chassis_Speeds;

    SwerveModuleState[] Swerve_Module_States = Swerve.toSwerveModuleStates(Swerve_Speeds);  

    SwerveDriveKinematics.desaturateWheelSpeeds(Swerve_Module_States, Drive_Constants.Max_Drive_Speed); // Keeps motor speeds in limits

    // Set each Swerve State, Inverse of the way locations were passed into Swerve Kinematic Object, Not sure why inverse order
    Front_Left_Swerve.Set_Swerve_State(Swerve_Module_States[0]);
    Rear_Left_Swerve.Set_Swerve_State(Swerve_Module_States[1]);
    Rear_Right_Swerve.Set_Swerve_State(Swerve_Module_States[2]);
    Front_Right_Swerve.Set_Swerve_State(Swerve_Module_States[3]);
  }

  public ChassisSpeeds Get_Current_ChassisSpeeds() {
    return Swerve_Speeds;
  }
  
  public Pose2d Get_Pose() {
    return Swerve_Odometry.getPoseMeters();
  }

  public void Reset_Pose(Pose2d Empty_Pose) {
    Swerve_Odometry.resetPosition(Rotation2d.fromRadians(0.0), Swerve_Positions, Empty_Pose);
  }

  public double Get_Yaw() {
    return -Units.degreesToRadians(Pigeon.getYaw()); // Negative makes clockwise positive
  }

  public double Get_Pitch() {
    return Units.degreesToRadians(Pigeon.getPitch());
  }

  public double Get_Roll() {
    return Units.degreesToRadians(Pigeon.getRoll());
  }

  @Override
  public void periodic() {
    Front_Left_Position.distanceMeters = Front_Left_Swerve.Get_Drive_Position();
    Front_Left_Position.angle = Front_Left_Swerve.Get_Swerve_State().angle;

    Front_Right_Position.distanceMeters = Front_Right_Swerve.Get_Drive_Position();
    Front_Right_Position.angle = Front_Right_Swerve.Get_Swerve_State().angle;

    Rear_Right_Position.distanceMeters = Rear_Right_Swerve.Get_Drive_Position();
    Rear_Right_Position.angle = Rear_Right_Swerve.Get_Swerve_State().angle;
    
    Rear_Left_Position.distanceMeters = Rear_Left_Swerve.Get_Drive_Position();
    Rear_Left_Position.angle = Rear_Left_Swerve.Get_Swerve_State().angle;

    Swerve_Odometry.update(Rotation2d.fromRadians(Get_Yaw()), Swerve_Positions);

    Yaw_Entry.setDouble(Get_Yaw()); 
    Speed_Entry.setDouble(Math.sqrt(Math.pow(Get_Current_ChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(Get_Current_ChassisSpeeds().vyMetersPerSecond, 2)));

    System.out.println(Slider.getDouble(0)); // Doesn't return Slider value

    Field_Layout.setRobotPose(Get_Pose()); // Test
    SmartDashboard.putData("Field Layout", Field_Layout);

    // System.out.println(Front_Left_Swerve.Get_Drive_Position());
    // System.out.println(Get_Pose());
  
    // System.out.println("Cancoder FL; " + Front_Left_Swerve.Get_Rotation_Position());
    // System.out.println("Cancoder FR; " + Front_Right_Swerve.Get_Rotation_Position());
    // System.out.println("Cancoder RR; " + Rear_Right_Swerve.Get_Rotation_Position());
    // System.out.println("Cancoder RL; " + Rear_Left_Swerve.Get_Rotation_Position());
  }

  @Override
  public void simulationPeriodic() {}
}
