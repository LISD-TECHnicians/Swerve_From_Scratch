package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.Drive_Constants;

import frc.robot.subsystems.Pneumatics_Subsystem;
import frc.robot.subsystems.Drive.Swerve_Subsystem;

import frc.robot.commands.Swerve_Velocity_Cmd;
import frc.robot.commands.Toggle_Solenoid_Cmd;
import frc.robot.commands.Reset_Pose_Cmd;

public class RobotContainer {
  private final CommandPS4Controller Controller = new CommandPS4Controller(3);

  private final Swerve_Subsystem SwerveSubsystem = new Swerve_Subsystem();
  private final Pneumatics_Subsystem PneumaticSubsystem = new Pneumatics_Subsystem();

  private final Toggle_Solenoid_Cmd ToggleSolenoid = new Toggle_Solenoid_Cmd(PneumaticSubsystem);
  private final Swerve_Velocity_Cmd JoystickSwerve = new Swerve_Velocity_Cmd(
    SwerveSubsystem, 
    () -> Controller.getLeftX() * Drive_Constants.Max_Drive_Speed, 
    () -> Controller.getLeftY() * Drive_Constants.Max_Drive_Speed, 
    () -> Controller.getRightX() * Drive_Constants.Max_Rotation_Speed * Drive_Constants.Rotation_Speed_Scale_Factor,
    Controller.L1());
  private final Reset_Pose_Cmd ResetPose = new Reset_Pose_Cmd(SwerveSubsystem);

  public RobotContainer() {
    configureBindings();

    SwerveSubsystem.setDefaultCommand(JoystickSwerve);
  }

  private void configureBindings() {
    Controller.L2().onTrue(ToggleSolenoid);

    Controller.R1().onTrue(ResetPose);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
