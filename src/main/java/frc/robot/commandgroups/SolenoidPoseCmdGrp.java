package frc.robot.commandgroups;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;

import frc.robot.commands.SetPoseCmd;
import frc.robot.commands.ToggleSolenoidCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SolenoidPoseCmdGrp extends SequentialCommandGroup /* ParallelCommandGroup */ {
  private final Pose2d randPose2d = new Pose2d(12, 14, new Rotation2d(3));

  public SolenoidPoseCmdGrp(SwerveSubsystem swerveSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
    addCommands(new ToggleSolenoidCmd(pneumaticsSubsystem), new SetPoseCmd(swerveSubsystem, randPose2d));
  }

}