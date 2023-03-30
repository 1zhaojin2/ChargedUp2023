// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArm extends CommandBase {

  double speed;
  double setpoint;

  /** Creates a new AutoArm. */
  public AutoArm(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.ArmSubsystem);
    this.speed = speed;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot.ArmSubsystem.runArm(speed);
    // SmartDashboard.putBoolean("auto enabled", true);
    SmartDashboard.putNumber("executing", 1);
    Robot.ArmSubsystem.moveToSetpoint(30);
    // wait 2 seconds
    try {
      TimeUnit.SECONDS.sleep(5);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    //wait 2 seconds and then move back to position 90
    Robot.ArmSubsystem.moveToSetpoint(30);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ArmSubsystem.armReset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.ArmSubsystem.encoderLimitReached(setpoint);
  }
}
