package uni.oldenburg.server.agent.message;

/**
 * @author Christopher Matthias
 */	
public class MessageType {
	public final static int REQUEST_RAMP_INFO 		   = 0; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			   = 1; // RampPlattform -> JobAgent
	public final static int START_AUCTION              = 2; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION            = 3; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	   = 4; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				   = 5; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			   = 6; // JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				   = 7; // PackageAgent
	public final static int GET_PACKAGE_COUNT	 	   = 8; // PackageAgent
	public final static int REMOVE_PACKAGE		 	   = 9; // PackageAgent
	public final static int ASSIGN_JOB				   = 10; // JobAgent
}
