package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.teamcode.BuckoBee

import dev.nextftc.robot.*
import dev.nextftc.robot.opmode.*
import org.firstinspires.ftc.teamcode.mechanisms.Status

import com.acmerobotics.dashboard.config.Config

@Config
object TestConstants {
	@JvmField
	var strafeCompensation: Double = 1.15
}

@NextTeleop(name = "My Teleop")
class MyTeleop(private val robot: BuckoBee = BuckoBee()) : NextOpMode(BuckoBee()) {
	override fun periodic() {
		Telemetry.log("Status", "Running")
	}
	
	override fun start() {
		this.robot.startDrive(this.gamepad1, TestConstants.strafeCompensation)
		this.robot.startLauncherControls(this.gamepad2)
	}
}