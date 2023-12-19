package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private final GenericEntry txEntry = RobotContainer.robotStatus.add("TX", 0.0).getEntry();
  private final GenericEntry tyEntry = RobotContainer.robotStatus.add("TY", 0.0).getEntry();
  private final GenericEntry taEntry = RobotContainer.robotStatus.add("TA", 0.0).getEntry();

  public LimelightSubsystem() {}

  public double getTx() {
    return LimelightHelpers.getTX("LL1");
  }

  public double getTy() {
    return LimelightHelpers.getTY("LL1");
  }

  public double getTa() {
    return LimelightHelpers.getTA("LL1");
  }

  @Override
  public void periodic() {
    txEntry.setDouble(getTx());
    tyEntry.setDouble(getTy());
    taEntry.setDouble(getTa());
  }

  @Override
  public void simulationPeriodic() {}
}
