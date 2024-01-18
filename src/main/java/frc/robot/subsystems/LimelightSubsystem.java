package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.LimelightConstants;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;

public class LimelightSubsystem extends SubsystemBase {
  private final DriverStation.Alliance alliance;

  public LimelightSubsystem() {
    this.alliance = DriverStation.getAlliance().get();
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

  public double getNeuralID(String limelightName) {
    return LimelightHelpers.getNeuralClassID(limelightName);
  }
  
  public boolean getValidTag(String limelightName) {
    return LimelightHelpers.getTV(limelightName);
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

  public void setPipeline(String limelightName, int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
  }

  public double getPipeline(String limelightName) {
    return LimelightHelpers.getCurrentPipelineIndex(limelightName);
  }

  @Override
  public void periodic() {
    /* System.out.println("Pose; " + getPose(LimelightConstants.LL_ONE) + " | " + 
        "ID; " + getFiducialID(LimelightConstants.LL_ONE) + " | " + 
        "Pipe; " + getPipeline(LimelightConstants.LL_ONE)); */
  }

  @Override
  public void simulationPeriodic() {}
}
