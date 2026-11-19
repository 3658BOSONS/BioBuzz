package org.firstinspires.ftc.teamcode.drive

import com.pedropathing.math.Pose
import dev.nextftc.hardware.RobotController
import dev.nextftc.hardware.actuators.*
import dev.nextftc.robot.*
import org.firstinspires.ftc.teamcode.pedro.Constants

// TODO: ...
import com.pedropathing.drivetrain.Drivetrain as DrivetrainInterface


class Drivetrain : Mechanism {
	val frontLeftMotor = NextMotor("frontLeftMotor")
	val backLeftMotor = NextMotor("backLeftMotor")
	val frontRightMotor = NextMotor("frontRightMotor")
	val backRightMotor = NextMotor("backRightMotor")
	
	// TODO: ...
//	val follower = Constants.create(RobotController.hardwareMap)
	
	init {
		this.frontLeftMotor.direction = NextMotor.Direction.REVERSE
		this.backLeftMotor.direction = NextMotor.Direction.REVERSE
		
//		this.frontRightMotor.direction = NextMotor.Direction.REVERSE
//		this.backRightMotor.direction = NextMotor.Direction.REVERSE

	
	}
}