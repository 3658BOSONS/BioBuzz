package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.drive.Drivetrain

import com.qualcomm.robotcore.hardware.Gamepad

import dev.nextftc.robot.*
import dev.nextftc.robot.drive.*


class BuckoBee : NextRobot {
	val driveTrain: Drivetrain = Drivetrain()
	
	fun startDrive(gamepad1: Gamepad): Unit {
		mecanumDrive(
			this.driveTrain.frontLeftMotor,
			this.driveTrain.frontRightMotor,
			this.driveTrain.backLeftMotor,
			this.driveTrain.backRightMotor,
			gamepad1
		).schedule()
	}
	
	override val mechanisms = setOf(this.driveTrain);
}