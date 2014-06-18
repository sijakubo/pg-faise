package uni.oldenburg.server.agent.message;

/**
 * @author Christopher Matthias
 */	
public class MessageType {
	public final static int REQUEST_RAMP_INFO 		   				= 0; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			   				= 1; // RampPlattform -> JobAgent
	public final static int PACKAGE_SPACE_AVAILABLE	   				= 2; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				   				= 3; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			   				= 4; // JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				   				= 5; // PackageAgent
	public final static int GET_PACKAGE_COUNT	 	   				= 6; // PackageAgent
	public final static int ASSIGN_JOB				   				= 7; // JobAgent
	public final static int ENQUIRE_RAMPS_WITHOUT_ENTRANCE			= 8; // Entrance/Exit PackageAgent -> Own OrderAgent
	public final static int ENQUIRE_RAMPS_RELAY						= 9; // Own OrderAgent -> Storage/Exit OrderAgent
	public final static int ENQUIRE_RAMP_RESPONSE					= 10;
	public final static int DEMAND_PACKAGE							= 11;
	public final static int ENQUIRE_RAMPS_STORAGE					= 12;
	public final static int FIND_PACKAGE_IN_STORAGE					= 13;
	public final static int SET_DESTINATION							= 14;
	public final static int SET_PENDING_INCOMING_STATUS				= 15;
	public final static int GET_PENDING_JOB_STATUS					= 16;
	public final static int ADD_PACKAGE_COMPLETED					= 17;
	public final static int SET_JOB_FLAG_COMPLETED					= 18;
	public final static int SET_DESTINATION_COMPLETED				= 19;
	public final static int GET_INCOMING_JOB_STATUS					= 20;
	public final static int GET_OUTGOING_JOB_STATUS					= 21;
	public final static int AUCTION_START							= 22;
	public final static int AUCTION_END								= 23;
	
	public final static int ESTIMATION_REQUEST						= 24;
	public final static int ESTIMATION_RESPONSE						= 25;
	public final static int ASSIGN_JOB_TO_VEHICLE					= 26;
	public final static int GET_CURRENT_POSITION					= 27;
	public final static int DRIVING_START							= 28;	
	public final static int TRANSFER_PACKAGE						= 29;	
	public final static int TRANSFER_PACKAGE_COMPLETED				= 30;
}
