package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive.Swerve_Subsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Swerve_Velocity_Cmd extends CommandBase {
  private final Swerve_Subsystem SwerveSubsystem;

  private final DoubleSupplier X_Speed; // m/s
  private final DoubleSupplier Y_Speed; // m/s
  private final DoubleSupplier Rotation_Speed; // rad/s

  private final BooleanSupplier Robot_Oriented;

  private ChassisSpeeds Swerve_Speeds = new ChassisSpeeds(); 

  public Swerve_Velocity_Cmd(Swerve_Subsystem SwerveSubsystem, DoubleSupplier X_Speed, DoubleSupplier Y_Speed, DoubleSupplier Rotation_Speed, BooleanSupplier Robot_Oriented) {
    this.SwerveSubsystem = SwerveSubsystem;
    this.X_Speed = X_Speed;
    this.Y_Speed = Y_Speed;
    this.Rotation_Speed = Rotation_Speed;

    this.Robot_Oriented = Robot_Oriented;

    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Robot_Oriented.getAsBoolean()) {
      Swerve_Speeds.vxMetersPerSecond = X_Speed.getAsDouble();
      Swerve_Speeds.vyMetersPerSecond = Y_Speed.getAsDouble();
      Swerve_Speeds.omegaRadiansPerSecond = Rotation_Speed.getAsDouble();
    }
    else {
      Swerve_Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        X_Speed.getAsDouble(), 
        Y_Speed.getAsDouble(), 
        Rotation_Speed.getAsDouble(), 
        Rotation2d.fromRadians(SwerveSubsystem.Get_Yaw()));
    }

    SwerveSubsystem.Run_Swerve(Swerve_Speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
