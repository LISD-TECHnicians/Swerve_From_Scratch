package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drive.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

public class SetPoseCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;

  private final Pose2d pose;

  public SetPoseCmd(SwerveSubsystem swerveSubsystem, Pose2d pose) {
    this.swerveSubsystem = swerveSubsystem;

    this.pose = pose;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setPose(pose);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
