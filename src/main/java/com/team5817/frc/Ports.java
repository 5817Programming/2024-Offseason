package com.team5817.frc;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
	/*
	 * LIST OF CHANNEL AND CAN IDS
	 *
	 * Swerve Modules go:
	 * 0 1
	 * 2 3
	 *
	 * spotless:off
	 */

	/* DRIVETRAIN CAN DEVICE IDS */
	public static final CanDeviceId FL_DRIVE = new CanDeviceId(0, "canivore");
	public static final CanDeviceId FL_ROTATION = new CanDeviceId(1, "canivore");
	public static final CanDeviceId FL_CANCODER = new CanDeviceId(0, "canivore");

	public static final CanDeviceId FR_DRIVE = new CanDeviceId(2, "canivore");
	public static final CanDeviceId FR_ROTATION = new CanDeviceId(3, "canivore");
	public static final CanDeviceId FR_CANCODER = new CanDeviceId(1, "canivore");

	public static final CanDeviceId BL_DRIVE = new CanDeviceId(4, "canivore");
	public static final CanDeviceId BL_ROTATION = new CanDeviceId(5, "canivore");
	public static final CanDeviceId BL_CANCODER = new CanDeviceId(2, "canivore");

	public static final CanDeviceId BR_DRIVE = new CanDeviceId(6, "canivore");
	public static final CanDeviceId BR_ROTATION = new CanDeviceId(7, "canivore");
	public static final CanDeviceId BR_CANCODER = new CanDeviceId(3, "canivore");

	/* SUBSYSTEM CAN DEVICE IDS */
	public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(8, "canivore");
	public static final CanDeviceId INTAKE_ROLLER = new CanDeviceId(9, "rio");

	public static final CanDeviceId INDEXER = new CanDeviceId(10, "canivore");
	public static final CanDeviceId FEEDER = new CanDeviceId(11, "canivore");

	public static final CanDeviceId AMP_ROLLER = new CanDeviceId(12, "rio");

	public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(13, "canivore");
	public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(14, "canivore");

	public static final CanDeviceId SHOOTER_2 = new CanDeviceId(15, "canivore");
	public static final CanDeviceId SHOOTER_1 = new CanDeviceId(16, "canivore");

	public static final CanDeviceId Pivot = new CanDeviceId(17, "canivore");
	public static final CanDeviceId Pivot_CANCODER = new CanDeviceId(5, "canivore");

	public static final CanDeviceId CLIMBER_MAIN = new CanDeviceId(18, "canivore");
	public static final CanDeviceId CLIMBER_FOLLOWER = new CanDeviceId(19, "canivore");

	public static final int PIGEON = 20;
	
	public static final CanDeviceId LEDS = new CanDeviceId(21, "rio");

	/* BEAM BREAK DIO CHANNELS*/
	public static final int SERIALIZER_BREAK = 7;
	public static final int FEEDER_BREAK = 8;
	public static final int AMP_BREAK = 9; 

	/* LINEAR SERVO PWM CHANNELS */
	public static final int CLIMBER_LINEAR_ACTUATOR = 9;
	public static final int ELEVATOR_LINEAR_ACTUATOR = 0;

	// spotless:on

        public static final int XBOX_1 = 1;
        public static final int XBOX_2 = 0;

}
