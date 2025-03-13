// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.trobot5013lib.led.BlinkingPattern;
import frc.robot.lib.trobot5013lib.led.SolidColorPattern;
import frc.robot.lib.trobot5013lib.led.TrobotAddressableLED;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDs extends SubsystemBase {
  int ledLength = 90;
  int ledPWMPort = 9;
  
  TrobotAddressableLED m_ledStrip = new TrobotAddressableLED(ledPWMPort, ledLength);
  
  Color white = new Color("fdf0d5");
  Color red = new Color("c1121f");
  Color green = new Color("a7c957");
  Color black = new Color(1,1,1); // Not sure if this will work

  BlinkingPattern blinkingWhite = new BlinkingPattern(white, 0.2);
  BlinkingPattern blinkingRed = new BlinkingPattern(red, 0.2);
  BlinkingPattern blinkingGreen = new BlinkingPattern(green, 0.2);
  BlinkingPattern blinkingBlack = new BlinkingPattern(black, 0.2);

  SolidColorPattern solidWhite = new SolidColorPattern(white);
  SolidColorPattern solidRed = new SolidColorPattern(red);
  SolidColorPattern solidGreen = new SolidColorPattern(green);
  SolidColorPattern solidBlack = new SolidColorPattern(black);




  public Command blinkWhiteThenStayWhite() {
    return runEnd(
      () -> m_ledStrip.setPattern(blinkingWhite),
      () -> m_ledStrip.setPattern(solidWhite)
    ); 
 
  }

  /** Creates a new LEDs. */
  public LEDs() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
