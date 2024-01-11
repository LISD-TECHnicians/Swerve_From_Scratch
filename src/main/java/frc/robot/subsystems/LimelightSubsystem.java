package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;

public class LimelightSubsystem extends SubsystemBase {
  private final String limelightName;

  private final DriverStation.Alliance alliance;

  public LimelightSubsystem(String limelightName) {
    this.alliance = DriverStation.getAlliance();

    this.limelightName = limelightName;
  }

  public double getTX() {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTY() {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getTA() {
    return LimelightHelpers.getTA(limelightName);
  }

  public Pose2d getPose() { // Needs Testing
    if (alliance == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }
    else {
      return LimelightHelpers.getBotPose2d_wpiRed(limelightName);
    }   
  }

  public double getFiducialID() {
    return LimelightHelpers.getFiducialID(limelightName);
  }

  public double getPipeline() {
    return LimelightHelpers.getCurrentPipelineIndex(limelightName);
  }

  @Override
  public void periodic() {
    System.out.println("Pose; " + getPose() + " | " + "ID; " + getFiducialID() + " | " + "Pipe; " + getPipeline());
  }

  @Override
  public void simulationPeriodic() {}
}
