package uni.oldenburg.server.agent.message;

public class MessageType {
	public final static int REQUEST_RAMP_INFO 		= 0x00; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			= 0x01; // RampPlattform -> JobAgent
	public final static int START_AUCTION           = 0x02; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION         = 0x03; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	= 0x04; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				= 0x05;
}
