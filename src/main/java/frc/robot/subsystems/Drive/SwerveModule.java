package frc.robot.subsystems.Drive;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final WPI_TalonFX driveMotor;
  private final CANSparkMax rotationMotor;

  private final CANCoder rotationEncoder;
  private final double angleOffset;

  private final boolean rotationEncoderInvert;

  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.MAX_DRIVE_SET_ACCELERATION);

  private final PIDController rotationPID = new PIDController(DriveConstants.MOTOR_ROTATION_P, DriveConstants.MOTOR_ROTATION_I, DriveConstants.MOTOR_ROTATION_D);

  private SwerveModuleState currentSwerveModuleState = new SwerveModuleState();

  public SwerveModule(int driveMotorID, int rotationMotorID, int rotationEncoderID, double angleOffset, boolean driveMotorInvert, boolean rotationMotorInvert, boolean rotationEncoderInvert) {
    // Declare swerve module componenets
    driveMotor = new WPI_TalonFX(driveMotorID, "rio");
    rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);

    rotationEncoder = new CANCoder(rotationEncoderID, "rio");

    // Clear any left over settings from previous uses
    driveMotor.configFactoryDefault();
    rotationMotor.restoreFactoryDefaults();

    rotationEncoder.configFactoryDefault();

    // Set max voltage to motors so 100% is the same regardless ofexact battery charge
    driveMotor.configVoltageCompSaturation(DriveConstants.NOMINAL_VOLTAGE);
    rotationMotor.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    driveMotor.setNeutralMode(NeutralMode.Coast);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    this.rotationEncoderInvert = rotationEncoderInvert;

    driveMotor.setInverted(driveMotorInvert);
    rotationMotor.setInverted(rotationMotorInvert);

    this.angleOffset = angleOffset; // Offsets built in error from Absolute Encoder

    // Pevents rotation motor from rotating more than 90 deg
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition() * DriveConstants.DRIVE_MOTOR_POSITION_TO_METERS; // Returns meters
  }

  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() * DriveConstants.DRIVE_MOTOR_VELOCITY_TO_METERS_SECOND; // Returns meters per second
  }

  public double getRotationPosition() {
    return (Units.degreesToRadians(rotationEncoder.getAbsolutePosition()) - angleOffset) * (rotationEncoderInvert ? -1 : 1); // Returns radians
  }

  public double getRotationVelocity() {
    return Units.degreesToRadians(rotationEncoder.getVelocity()) * (rotationEncoderInvert ? -1 : 1); // Returns radians per second
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
    rotationSpeed = MathUtil.clamp(rotationSpeed, -DriveConstants.ROTATION_SPEED_SCALE_FACTOR, DriveConstants.ROTATION_SPEED_SCALE_FACTOR);

    driveMotor.set(driveSpeed);
    rotationMotor.set(rotationSpeed);
  }
}
