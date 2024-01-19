package frc.robot.subsystems.Drive;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
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
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    rotationMotor.restoreFactoryDefaults();

    rotationEncoder.getConfigurator().apply(new CANcoderConfiguration());

    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    rotationMotor.setIdleMode(IdleMode.kBrake);

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
    driveSpeed = DriveConstants.NOMINAL_VOLTAGE * driveSpeed / DriveConstants.MAX_DRIVE_SPEED;

    double rotationSpeed = rotationPID.calculate(getRotationPosition(), swerveModuleState.angle.getRadians());
    rotationSpeed = DriveConstants.NOMINAL_VOLTAGE * MathUtil.clamp(rotationSpeed, -DriveConstants.ROTATION_SPEED_SCALE_FACTOR, 
        DriveConstants.ROTATION_SPEED_SCALE_FACTOR);

    driveMotor.setVoltage(driveSpeed);
    rotationMotor.setVoltage(rotationSpeed);
  }

  public void setDriveBrake() {
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setDriveCoast() {
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
