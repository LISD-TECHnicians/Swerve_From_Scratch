package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.SwerveSubsystem;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;

  private final DoubleSupplier xController; // m/s
  private final DoubleSupplier yController; // m/s
  private final DoubleSupplier rotationController; // rad/s

  private final BooleanSupplier robotOriented;
  private final BooleanSupplier rotationPositionControl;

  private final PIDController rotationPositionPID = new PIDController(DriveConstants.ROTATION_POSITION_CONTROL_P, 
      DriveConstants.ROTATION_POSITION_CONTROL_I, DriveConstants.ROTATION_POSITION_CONTROL_D);

  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds(); 

  public SwerveCmd(SwerveSubsystem swerveSubsystem, DoubleSupplier xController, DoubleSupplier yController, 
      DoubleSupplier rotationController, BooleanSupplier robotOriented, BooleanSupplier rotationPositionControl) {
    this.swerveSubsystem = swerveSubsystem;
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;

    this.robotOriented = robotOriented;
    this.rotationPositionControl = rotationPositionControl;

    rotationPositionPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = Math.abs(xController.getAsDouble()) > ControllerConstants.DEADBAND ? xController.getAsDouble() * 
        DriveConstants.MAX_DRIVE_SPEED : 0;
    double ySpeed = Math.abs(yController.getAsDouble()) > ControllerConstants.DEADBAND ? yController.getAsDouble() * 
        DriveConstants.MAX_DRIVE_SPEED : 0;

    double rotationSpeed = Math.abs(rotationController.getAsDouble()) > ControllerConstants.DEADBAND ? 
        rotationController.getAsDouble() * DriveConstants.MAX_SET_ROTATION_SPEED : 0;
    double rotationPosition = Math.abs(rotationController.getAsDouble()) > ControllerConstants.DEADBAND ? 
        rotationController.getAsDouble() * Math.PI : 0;

    if (robotOriented.getAsBoolean()) {
      swerveSpeeds.vxMetersPerSecond = xSpeed;
      swerveSpeeds.vyMetersPerSecond = ySpeed;
      swerveSpeeds.omegaRadiansPerSecond = rotationSpeed;
    }
    else {
      if (rotationPositionControl.getAsBoolean()) {
        swerveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, 
          ySpeed, 
          MathUtil.clamp(rotationPositionPID.calculate(swerveSubsystem.getYaw(), rotationPosition), 
              -DriveConstants.MAX_SET_ROTATION_SPEED, DriveConstants.MAX_SET_ROTATION_SPEED), 
          Rotation2d.fromRadians(swerveSubsystem.getYaw()));
      }
      else {
        swerveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, 
          ySpeed, 
          rotationSpeed, 
          Rotation2d.fromRadians(swerveSubsystem.getYaw()));
      }
    }

    swerveSubsystem.setChassisSpeeds(swerveSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
