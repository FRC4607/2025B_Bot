// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator_Lock;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Elevator_Lock_SetAngle_SD extends InstantCommand {
  private Elevator_Lock m_subsystem;
  public Elevator_Lock_SetAngle_SD(Elevator_Lock subsystem) {

    m_subsystem = subsystem;

    addRequirements(m_subsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setAngle(SmartDashboard.getNumber("Angle", 45));
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
