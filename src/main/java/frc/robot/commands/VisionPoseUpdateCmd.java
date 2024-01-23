package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.LimelightConstants;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class VisionPoseUpdateCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public VisionPoseUpdateCmd(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    // addRequirements(limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveSubsystem.visionPoseUpdate(limelightSubsystem.getPose(LimelightConstants.LL_TWO), 
        limelightSubsystem.getTimeStamp(LimelightConstants.LL_TWO));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
