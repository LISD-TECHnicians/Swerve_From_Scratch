package frc.robot.subsystems.Drive;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final CANSparkMax rotationMotor;

  private final CANcoder rotationEncoder;
  private final double angleOffset;

  private final boolean rotationEncoderInvert;

  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.MAX_DRIVE_SET_ACCELERATION);

  private final PIDController rotationPID = new PIDController(DriveConstants.MOTOR_ROTATION_P, DriveConstants.MOTOR_ROTATION_I, 
      DriveConstants.MOTOR_ROTATION_D);

  private SwerveModuleState currentSwerveModuleState = new SwerveModuleState();

  public SwerveModule(int driveMotorID, int rotationMotorID, int rotationEncoderID, double angleOffset, 
      boolean driveMotorInvert, boolean rotationMotorInvert, boolean rotationEncoderInvert) {
    // Declare swerve module componenets
    driveMotor = new TalonFX(driveMotorID, "rio");
    rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);

    rotationEncoder = new CANcoder(rotationEncoderID, "rio");

    // Clear any left over settings from previous uses
    // driveMotor.configFactoryDefault();
    // rotationMotor.restoreFactoryDefaults();

    // rotationEncoder.configFactoryDefault();

    // Set max voltage to motors so 100% is the same regardless ofexact battery charge
    // driveMotor.configVoltageCompSaturation(DriveConstants.NOMINAL_VOLTAGE);
    // rotationMotor.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    // rotationMotor.setIdleMode(IdleMode.kBrake);

    this.rotationEncoderInvert = rotationEncoderInvert;

    driveMotor.setInverted(driveMotorInvert);
    rotationMotor.setInverted(rotationMotorInvert);

    this.angleOffset = angleOffset; // Offsets built in error from Absolute Encoder

    // Pevents rotation motor from rotating more than 90 deg
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition() { // Returns meters
    return driveMotor.getPosition().getValueAsDouble() * DriveConstants.DRIVE_MOTOR_POSITION_TO_METERS;
  }

  public double getDriveVelocity() { // Returns meters per second
    return driveMotor.getVelocity().getValueAsDouble() * DriveConstants.DRIVE_MOTOR_VELOCITY_TO_METERS_SECOND; 
  }

  public double getRotationPosition() { // Returns radians
    return (rotationEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset) * (rotationEncoderInvert ? -1 : 1); 
  }

  public double getRotationVelocity() { // Returns radians per second
    return rotationEncoder.getVelocity().getValueAsDouble() * (rotationEncoderInvert ? -1 : 1);
  }

  public SwerveModuleState getSwerveState() {
    currentSwerveModuleState.speedMetersPerSecond = getDriveVelocity();
    currentSwerveModuleState.angle = Rotation2d.fromRadians(getRotationPosition());

    return currentSwerveModuleState;
  }

  public void setSwerveState(SwerveModuleState swerveModuleState) {
    swerveModuleState = SwerveModuleState.optimize(swerveModuleState, getSwerveState().angle);

    double driveSpeed = driveLimiter.calculate(swerveModuleState.speedMetersPerSecond);
    driveSpeed = driveSpeed / DriveConstants.MAX_DRIVE_SPEED;

    double rotationSpeed = rotationPID.calculate(getRotationPosition(), swerveModuleState.angle.getRadians());
    rotationSpeed = MathUtil.clamp(rotationSpeed, -DriveConstants.ROTATION_SPEED_SCALE_FACTOR, 
        DriveConstants.ROTATION_SPEED_SCALE_FACTOR);

    driveMotor.set(driveSpeed);
    rotationMotor.set(rotationSpeed);
  }

  public void setDriveBrake() {
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setDriveCoast() {
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
