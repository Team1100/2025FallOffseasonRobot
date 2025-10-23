// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.lang.reflect.Field;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

/*
  JSON RobotMap config format:
  [
    {
      "VarName": "NAME_OF_VARIABLE",
      "Value": NEW_VALUE_OF_CORRESPONDING_TYPE
    }
  ]
*/

class RobotMapConfigValue {
  String varName = null;
  Object value = null;

  public void setVarName(String varName) {
      this.varName = varName;
  }

  public void setValue(Object value) {
      this.value = value;
  }
}

/** Add your docs here. */
public class RobotMap {

  public static void init()
  {
    try {
      String home = java.lang.System.getenv("HOME");
      if (home == null || home.isEmpty()) {
        home = "/home/lvuser";
      }
      else {
        System.out.println("$HOME is " + home);
      }
      File f = new File(home + "/1100_Config");
      if (f.exists())
      {
        ObjectMapper mapper = new ObjectMapper();
        List<RobotMapConfigValue> changedValues = mapper.readValue(f, new TypeReference<List<RobotMapConfigValue>>(){});

        for (int i=0; i < changedValues.size(); i++) {
          RobotMapConfigValue value = changedValues.get(i);
          try {
            Field field = RobotMap.class.getField(value.varName);
            field.set(null, value.value);
            System.out.println("Configuring " + value.varName + " to " + value.value);
          }
          catch (java.lang.NoSuchFieldException x) {
            System.out.println("No RobotMap field named " + value.varName);
          }
        }
      }  
      else
      {
        System.out.println("No configuration present, using defaults");
      }          
    } catch (Exception e) {
      System.err.println("exception from RobotMap.init():" + e.toString());
      // can't read the config. Carry on.
    }
  }
  // [R]obot geometry
  public static double R_TRACK_WIDTH_INCHES = 28.5;
  public static double R_WHEEL_BASE_INCHES  = 28.5;
  public static double R_BASE_RADIUS_INCHES = 14.25 * Math.sqrt(2);

  // [D]rive module CAN IDs
  public static int D_FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = RoboRioMap.CAN_2;
  public static int D_FRONT_RIGHT_TURN_MOTOR_CAN_ID  = RoboRioMap.CAN_3;
  public static int D_FRONT_LEFT_DRIVE_MOTOR_CAN_ID = RoboRioMap.CAN_4;
  public static int D_FRONT_LEFT_TURN_MOTOR_CAN_ID  = RoboRioMap.CAN_5;
  public static int D_BACK_RIGHT_DRIVE_MOTOR_CAN_ID = RoboRioMap.CAN_8;
  public static int D_BACK_RIGHT_TURN_MOTOR_CAN_ID  = RoboRioMap.CAN_9;
  public static int D_BACK_LEFT_DRIVE_MOTOR_CAN_ID = RoboRioMap.CAN_6;
  public static int D_BACK_LEFT_TURN_MOTOR_CAN_ID  = RoboRioMap.CAN_7;

  // [I]ntake Subsystem
  public static boolean I_ENABLED = true;
  public static int I_MOTOR_CAN_ID = RoboRioMap.CAN_11;


  // [P]ivot Subsystem
  public static boolean P_ENABLED = true;
  public static int P_MOTOR_CAN_ID = RoboRioMap.CAN_10;


  // [V]ision
  public static boolean V_ENABLED;
}