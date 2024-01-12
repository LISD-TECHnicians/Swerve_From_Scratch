package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;

public class LimelightSubsystem extends SubsystemBase {
  private final DriverStation.Alliance alliance;

  public LimelightSubsystem() {
    this.alliance = DriverStation.getAlliance();
  }

  public double getTX(String limelightName) {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTY(String limelightName) {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getTA(String limelightName) {
    return LimelightHelpers.getTA(limelightName);
  }

  public Pose2d getPose(String limelightName) {
    if (alliance == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }
    else {
      return LimelightHelpers.getBotPose2d_wpiRed(limelightName);
    }   
  }

  public double getFiducialID(String limelightName) {
    return LimelightHelpers.getFiducialID(limelightName);
  }

  public void setPipeline(String limelightName, int pipeline) {
    LimelightHelpers.setPipelineIndex(limelightName, pipeline);
  }

  public double getPipeline(String limelightName) {
    return LimelightHelpers.getCurrentPipelineIndex(limelightName);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
