package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.drive.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.util.*

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.hardware.lynx.LynxModule


import dev.nextftc.robot.NextRobot
import dev.nextftc.hardware.RobotController

import dev.nextftc.robot.drive.mecanumDrive
import dev.nextftc.robot.triggers.CommandGamepad

class BuckoBee : NextRobot {
	val driveTrain: Drivetrain = Drivetrain()
	
	val outtake = Outtake()
	
	val status: StatusLight = StatusLight()
	
	val rc = RobotController
	
	fun startDrive(gamepad1: Gamepad): Unit {
		mecanumDrive(
			this.driveTrain.frontLeftMotor,
			this.driveTrain.frontRightMotor,
			this.driveTrain.backLeftMotor,
			this.driveTrain.backRightMotor,
			gamepad1,
		).schedule()
		
		this.status.set(Status.Active)
	}
	
	fun startLauncherControls(gamepad2: Gamepad): Unit {
		val commandGamepad = CommandGamepad(gamepad2)
		
		commandGamepad.dpadUp.onTrue(this.outtake.up())
		commandGamepad.dpadDown.onTrue(this.outtake.down())
		
		commandGamepad.b.onTrue(this.outtake.grab())
		commandGamepad.y.onFalse(this.outtake.drop())
		
		
	}
//	override val mechanisms = setOf(this.driveTrain);
}