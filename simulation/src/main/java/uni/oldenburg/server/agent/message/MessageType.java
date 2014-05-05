package uni.oldenburg.server.agent.message;

/**
 * @author Christopher Matthias
 */	
public class MessageType {
	public final static int REQUEST_RAMP_INFO 		   = 0x00; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			   = 0x01; // RampPlattform -> JobAgent
	public final static int START_AUCTION              = 0x02; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION            = 0x03; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	   = 0x04; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				   = 0x05; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			   = 0x06; // JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				   = 0x07; // PackageAgent
	public final static int GET_PACKAGE_COUNT	 	   = 0x08; // PackageAgent
	public final static int REMOVE_PACKAGE		 	   = 0x09; // PackageAgent
	public final static int ASSIGN_JOB				   = 0x0A; // JobAgent
	public final static int ASSIGN_VEHICLE_FOR_PACKAGE = 0x0B; // RampRouting @ VehicleRouting
	public final static int SEARCH_FOR_PACKAGE = 0x0C; //PackageAgent (Exit) @ OrderAgent (EXIT)
	public final static int ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS = 0x0D; // OrderAgent(Exit) @ OrderAgent (Storage)
	public final static int CHECK_IF_PACKAGE_IS_STORED = 0xE; // OrderAgent(Storage) @ PackageAgent (Storage)
	public final static int ANSWER_IF_PACKAGE_IS_CONTAINED = 0xE; // PackageAgent (Storage)@ OrderAgent(Storage) 	
	public final static int GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT = 0xF; // OrderAgent (Storage) @ OrderAgent (Exit)
	public final static int SET_PACKAGE_RESERVED = 0x10; // Orderagent (Exit)@ PackageAgent (Exit)
	public final static int PACKAGE_RESERVATION = 0x11; //  PackageAgent (Exit)
	public final static int SET_PACKAGE_DESTINATION_STORAGE = 0x12; //  PackageAgent (Exit)
	public final static int INITIALIZE_START_AUCTION_BEHAVIOUR = 0x13; //  PackageAgent (Entry/Storage) @ Routingagent

   public static final int START_RAMP_SEARCH_FOR_PACKAGE = 0x14;
   public static final int ENQUIRE_OUTGOING_RAMP = 0x15;
   public static final int CHECK_IF_PACKAGE_IS_NEEDED = 0x16;
   public static final int PACKAGE_IS_NEEDED = 0x17;
   public static final int START_RAMP_EXIT_PACKAGE_ENQUIRE = 0x18;
   public static final int PACKAGE_IS_NEEDED_FROM_RAMP_EXIT = 0x19;
   public static final int RESERVE_PACKAGE_SLOT_ON_RAMP = 0x20;
   public static final int ASSIGN_PACKAGE_DESTINATION = 0x21;

}
