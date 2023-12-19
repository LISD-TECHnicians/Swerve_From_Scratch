package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

public class SetPoseCmd extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;

  private Pose2d pose = new Pose2d();

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
