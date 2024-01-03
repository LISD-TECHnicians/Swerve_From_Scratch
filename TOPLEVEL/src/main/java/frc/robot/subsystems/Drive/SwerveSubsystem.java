package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

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

public class SwerveSubsystem extends SubsystemBase {
  // Declare all Swerve Modules
  private final SwerveModule frontLeftSwerve = new SwerveModule(    
    DriveConstants.FRONT_LEFT_DRIVE_ID, 
    DriveConstants.FRONT_LEFT_ROTATION_ID, 
    DriveConstants.FRONT_LEFT_ROTATION_ENCODER_ID, 
    DriveConstants.FRONT_LEFT_ANGLE_OFFSET,
    DriveConstants.FRONT_LEFT_DRIVE_MOTOR_INVERT,
    DriveConstants.FRONT_LEFT_ROTATION_MOTOR_INVERT,
    DriveConstants.FRONT_LEFT_ROTATION_ENCODER_INVERT);
  private final SwerveModule frontRightSwerve = new SwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVE_ID, 
    DriveConstants.FRONT_RIGHT_ROTATION_ID, 
    DriveConstants.FRONT_RIGHT_ROTATION_ENCODER_ID, 
    DriveConstants.FRONT_RIGHT_ANGLE_OFFSET,
    DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_INVERT,
    DriveConstants.FRONT_RIGHT_ROTATION_MOTOR_INVERT,
    DriveConstants.FRONT_RIGHT_ROTATION_ENCODER_INVERT);
  private final SwerveModule rearRightSwerve = new SwerveModule(
    DriveConstants.REAR_RIGHT_DRIVE_ID, 
    DriveConstants.REAR_RIGHT_ROTATION_ID, 
    DriveConstants.REAR_RIGHT_ROTATION_ENCODER_ID, 
    DriveConstants.REAR_RIGHT_ANGLE_OFFSET,
    DriveConstants.REAR_RIGHT_DRIVE_MOTOR_INVERT,
    DriveConstants.REAR_RIGHT_ROTATION_MOTOR_INVERT,
    DriveConstants.REAR_RIGHT_ROTATION_ENCODER_INVERT);
  private final SwerveModule rearLeftSwerve = new SwerveModule(    
    DriveConstants.REAR_LEFT_DRIVE_ID, 
    DriveConstants.REAR_LEFT_ROTATION_ID, 
    DriveConstants.REAR_LEFT_ROTATION_ENCODER_ID, 
    DriveConstants.REAR_LEFT_ANGLE_OFFSET,
    DriveConstants.REAR_LEFT_DRIVE_MOTOR_INVERT,
    DriveConstants.REAR_LEFT_ROTATION_MOTOR_INVERT,
    DriveConstants.REAR_LEFT_ROTATION_ENCODER_INVERT);

  // Declare Swerve Kinematics using Swerve Module locations
  private final SwerveDriveKinematics swerve = new SwerveDriveKinematics( 
    DriveConstants.FRONT_LEFT_LOCATION, 
    DriveConstants.FRONT_RIGHT_LOCATION, 
    DriveConstants.REAR_RIGHT_LOCATION, 
    DriveConstants.REAR_LEFT_LOCATION);

  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds(); // Declare Chassis Speed for use in methods
  
  // private final Pose2d initialPose = new Pose2d(1.0, 1.0, Rotation2d.fromRadians(0.0));

  //  Declare Swerve Module Positions for SWerve Odometry
  private SwerveModulePosition frontLeftPosition = new SwerveModulePosition();
  private SwerveModulePosition frontRightPosition = new SwerveModulePosition();
  private SwerveModulePosition rearRightPosition = new SwerveModulePosition();
  private SwerveModulePosition rearLeftPosition = new SwerveModulePosition();

  // Inverting order might give ododmetry correct axis orientation. Needs tested
  private SwerveModulePosition[] swervePositions = {frontLeftPosition, rearLeftPosition, rearRightPosition, frontRightPosition};

  // Declare Swerve Odometry
  private final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(swerve, Rotation2d.fromRadians(0.0), swervePositions/*, initialPose*/);

  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.PIGEON_ID, "canivore"); // Declare IMU

  private final GenericEntry yawEntry = RobotContainer.robotStatus.add("Heading", 0).getEntry(); //  Elastic Test
  private final GenericEntry speedEntry = RobotContainer.robotStatus.add("Speed", 0).getEntry();

  private final GenericEntry slider = RobotContainer.robotStatus
    .add("Slider", 0)
    .withWidget("Number Slider")
    .withPosition(1, 1)
    .withSize(2, 1)
    .getEntry();

  private final Field2d fieldLayout = new Field2d();

  public SwerveSubsystem() {
    pigeon.configFactoryDefault();

    pigeon.configMountPose(0, 0, 0);
    
    // Allows PathPlanner to construct trajectories on its own
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      DriveConstants.PATH_FOLLOW_CONFIG,
      this); // Reference to this subsystem to set requirements
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // List of Swerve States from desired Swerve Speeds
    SwerveModuleState[] swerveModuleStates = swerve.toSwerveModuleStates(chassisSpeeds);  

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED); // Keeps motor speeds in limits

    swerveSpeeds = swerve.toChassisSpeeds(swerveModuleStates);

    // Set each Swerve State, Inverse of the way locations were passed into Swerve Kinematic Object, Not sure why inverse order    
    frontLeftSwerve.setSwerveState(swerveModuleStates[0]); 
    rearLeftSwerve.setSwerveState(swerveModuleStates[1]);
    rearRightSwerve.setSwerveState(swerveModuleStates[2]);
    frontRightSwerve.setSwerveState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerveSpeeds;
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(Rotation2d.fromRadians(getYaw()), swervePositions, pose);
  }
  
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public double getYaw() {
    return -Units.degreesToRadians(pigeon.getYaw()); // Negative makes clockwise positive
  }

  public double getPitch() {
    return Units.degreesToRadians(pigeon.getPitch());
  }

  public double getRoll() {
    return Units.degreesToRadians(pigeon.getRoll());
  }

  public void setDriveBrake() {
    frontLeftSwerve.setDriveBrake(); frontRightSwerve.setDriveBrake(); rearRightSwerve.setDriveBrake(); rearLeftSwerve.setDriveBrake();
  }

  public void setDriveCoast() {
    frontLeftSwerve.setDriveCoast(); frontRightSwerve.setDriveCoast(); rearRightSwerve.setDriveCoast(); rearLeftSwerve.setDriveCoast();
  }

  @Override
  public void periodic() {
    frontLeftPosition.distanceMeters = frontLeftSwerve.getDrivePosition();
    frontLeftPosition.angle = frontLeftSwerve.getSwerveState().angle;

    frontRightPosition.distanceMeters = frontRightSwerve.getDrivePosition();
    frontRightPosition.angle = frontRightSwerve.getSwerveState().angle;

    rearRightPosition.distanceMeters = rearRightSwerve.getDrivePosition();
    rearRightPosition.angle = rearRightSwerve.getSwerveState().angle;
    
    rearLeftPosition.distanceMeters = rearLeftSwerve.getDrivePosition();
    rearLeftPosition.angle = rearLeftSwerve.getSwerveState().angle;

    swerveOdometry.update(Rotation2d.fromRadians(getYaw()), swervePositions);

    yawEntry.setDouble(Math.round(Units.radiansToDegrees(getYaw()))); 
    speedEntry.setDouble(Math.round(Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2))));

    System.out.println(slider.getDouble(0)); // Doesn't return Slider value, test again

    fieldLayout.setRobotPose(getPose()); // Test
    SmartDashboard.putData("Field Layout", fieldLayout);

    // System.out.println("Cancoder FL; " + frontLeftSwerve.getRotationPosition());
    // System.out.println("Cancoder FR; " + frontRightSwerve.getRotationPosition());
    // System.out.println("Cancoder RR; " + rearRightSwerve.getRotationPosition());
    // System.out.println("Cancoder RL; " + rearLeftSwerve.getRotationPosition());
  }

  @Override
  public void simulationPeriodic() {}
}
