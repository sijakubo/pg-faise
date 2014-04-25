package uni.oldenburg.server.agent.message;

public class MessageType {
	public final static int REQUEST_RAMP_INFO 		= 0x00; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			= 0x01; // RampPlattform -> JobAgent
	public final static int START_AUCTION           = 0x02; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION         = 0x03; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	= 0x04; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				= 0x05; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			= 0x06;	// JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				= 0x07; // PackageAgent
	public final static int GET_PACKAGE_COUNT		= 0x08; // PackageAgent
	public final static int REMOVE_PACKAGE			= 0x09; // PackageAgent
	public final static int ASSIGN_JOB				= 0x0A; // JobAgent
}
