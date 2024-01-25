package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.Constants.LimelightConstants;

public class SetPipelineCmd extends Command {
  private final LimelightSubsystem limelightSubsystem;

  private final String limelightName;
  private final int pipelineIndex;

  public SetPipelineCmd(LimelightSubsystem limelightSubsystem, String limelightName, int pipelineIndex) {
    this.limelightSubsystem = limelightSubsystem;

    this.limelightName = limelightName;
    this.pipelineIndex = pipelineIndex;

    // addRequirements(limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.setPipeline(limelightName, pipelineIndex);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.setPipeline(limelightName, LimelightConstants.POSE_ESTIMATOR_PIPELINE);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
