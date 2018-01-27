package org.usfirst.frc.team6002.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;


public class I2CArduino {
	public I2C Arduino = new I2C(I2C.Port.kOnboard, 84);
	public byte[] toSend = new byte[1];
	boolean ledOn = true;
	
	public void init() {
		Arduino = new I2C(I2C.Port.kOnboard, 84);
		toSend[0] = 13;
	}
	public void lightflash() {
		if(ledOn) {
			toSend[0] = 13;//on
		}
		else {
			toSend[0] = 14;//off
		}
		Arduino.write(84, toSend[0]);
		Timer.delay(.00005);
	}
	public void toggleOn() {
		ledOn = !ledOn;
	}
	public byte getByte() {
		return toSend[0];
		
	}
	public void transaction(byte[] b, int Length) {
		Arduino.transaction(b, Length, null, 0);
	}
	public boolean getLedOn() {
		return ledOn;
	}
	
	
}
