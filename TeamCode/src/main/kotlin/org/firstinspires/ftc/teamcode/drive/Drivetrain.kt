package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.drive

import dev.nextftc.hardware.actuators.*
import dev.nextftc.robot.*


class Drivetrain : Mechanism {
	val frontLeftMotor = NextMotor("frontLeftMotor")
	val backLeftMotor = NextMotor("backLeftMotor")
	val frontRightMotor = NextMotor("frontRightMotor")
	val backRightMotor = NextMotor("backRightMotor")
	
	init {
		this.frontLeftMotor.direction = NextMotor.Direction.REVERSE
		this.backLeftMotor.direction = NextMotor.Direction.REVERSE
	}
}