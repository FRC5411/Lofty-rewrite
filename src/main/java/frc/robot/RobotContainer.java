// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Structures.Superstructure;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.LEDS;
import frc.robot.Subsystems.PinchersofPower;

public class RobotContainer {
  private Arm m_arm;
  private PinchersofPower m_pop;
  private LEDS m_leds;
  private Superstructure m_superstructure;
  private boolean m_state;
  private CommandXboxController m_Controller;

  public RobotContainer() {
    m_state = true;
    m_arm = new Arm();
    m_pop = new PinchersofPower();
    m_leds = new LEDS();
    m_superstructure = new Superstructure(m_arm, m_pop, m_leds, () -> m_state);
    m_superstructure.setTimeOuts(0.5, 0.5, 0.5);
    m_Controller = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {}

  public void click(Trigger button, Command command, Command command2) {
    button.onTrue(command);
    button.onFalse(command2);
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
