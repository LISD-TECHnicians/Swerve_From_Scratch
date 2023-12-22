package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;

public class LimelightSubsystem extends SubsystemBase {
  private final String LL1 = "LL1";

  private final GenericEntry txEntryLL1 = RobotContainer.robotStatus.add("TX", 0.0).getEntry();
  private final GenericEntry tyEntryLL1 = RobotContainer.robotStatus.add("TY", 0.0).getEntry();
  private final GenericEntry taEntryLL1 = RobotContainer.robotStatus.add("TA", 0.0).getEntry();

  public LimelightSubsystem() {}

  public double getTx(String limelightName) {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTy(String limelightName) {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getTa(String limelightName) {
    return LimelightHelpers.getTA(limelightName);
  }

  public Pose2d getPose(String limelightName) {
    return LimelightHelpers.getBotPose2d(limelightName);
  }

  @Override
  public void periodic() {
    txEntryLL1.setDouble(getTx(LL1));
    tyEntryLL1.setDouble(getTy(LL1));
    taEntryLL1.setDouble(getTa(LL1));

    System.out.println(getPose(LL1));
  }

  @Override
  public void simulationPeriodic() {}
}
