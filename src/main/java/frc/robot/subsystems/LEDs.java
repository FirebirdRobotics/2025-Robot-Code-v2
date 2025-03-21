// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  int ledLength = 51;
  int ledPWMPort = 9;
  
  TrobotAddressableLED m_ledStrip = new TrobotAddressableLED(ledPWMPort, ledLength);
  
  
  Color white = new Color(253, 240, 213);
  Color red = new Color(139, 0, 0);
  Color green = new Color(167, 201, 87);
  Color black = new Color(1,1,1); // Not sure if this will work
  //should be 1 for r value to get roughly black

  BlinkingPattern blinkingWhite = new BlinkingPattern(white, 0.2);
  BlinkingPattern blinkingRed = new BlinkingPattern(red, 0.2);
  BlinkingPattern blinkingGreen = new BlinkingPattern(green, 0.2);
  BlinkingPattern blinkingBlack = new BlinkingPattern(black, 0.2);

  SolidColorPattern solidWhite = new SolidColorPattern(white);
  SolidColorPattern solidRed = new SolidColorPattern(red);
  SolidColorPattern solidGreen = new SolidColorPattern(green);
  SolidColorPattern solidBlack = new SolidColorPattern(black);

  EndEffector m_EndEffector;


  public Command blinkWhiteThenStayWhite() {
    return runEnd(
      () -> m_ledStrip.setPattern(blinkingWhite),
      () -> m_ledStrip.setPattern(solidWhite)
    ); 
 
  }

  public Command blinkRedThenStayRed() {
    return runEnd(
      () -> m_ledStrip.setPattern(blinkingRed),
      () -> m_ledStrip.setPattern(solidRed)
    ); 
  }

  public Command blinkGreenThenStayGreen() {
    return runEnd(
      () -> m_ledStrip.setPattern(blinkingGreen),
      () -> m_ledStrip.setPattern(solidGreen)
    ); 
  }

  String switchString = "black";


  public Command blinkBlackThenStayBlack(double blinkingTime) {
    return runEnd(
      () -> switchString = "black blinking",
      () -> switchString = "black blinking"
    ).withTimeout(blinkingTime); 
  }


  /** Creates a new LEDs. */
  public LEDs() {

  }

  enum colorSwitchCase {
    WHITE,
    RED,
    BLACK
  }

  colorSwitchCase mySwitchCase = colorSwitchCase.WHITE;

  public void setWhite() {
    mySwitchCase = colorSwitchCase.WHITE;
  }

  public void setRed() {
    mySwitchCase = colorSwitchCase.RED;
  }

  public void setBlack() {
    mySwitchCase = colorSwitchCase.BLACK;
  }


  @Override
  public void periodic() {
    
    switch (mySwitchCase) {
      
      case WHITE:
        m_ledStrip.setPattern(blinkingWhite);
        break;

      case RED:
        m_ledStrip.setPattern(blinkingRed);
        break;

      case BLACK:
        m_ledStrip.setPattern(blinkingBlack);
        break;
    
      default:
        break;
    }
    // m_ledStrip.setPattern(blinkingRed);
    // // This method will be called once per scheduler run
    // switch (switchString) {
    //   case "white":
        
    //     break;

    //   case "red":
        
    //     break;

    //   case "green":
        
    //     break;

    //   case "black":
    //     m_ledStrip.setPattern(solidBlack);
    //     break;

    
    //   case "black blinking":
    //     m_ledStrip.setPattern(blinkingBlack);
    //     break;


    //   default:

    //     break;
    // }
    
  }
}
