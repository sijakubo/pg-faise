package uni.oldenburg.server.agent;

import java.io.IOException;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";

	private Conveyor myConveyor;
	private Szenario mySzenario;
	
	private Logger logger = Logger.getLogger(RampOrderAgent.class);

	/**
	 * @author Matthias
	 */
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (Conveyor) args[1];
		}
		
		// am i am ramp?
		if (myConveyor instanceof ConveyorRamp) {
			ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
			
			// what ramp type am i?
			switch(myRampConveyor.getRampType()) {
				case ConveyorRamp.RAMP_ENTRANCE:
					addBehaviour(new SendEquirePackageRequestRelay(MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_RELAY)));
					break;
				case ConveyorRamp.RAMP_EXIT:
					addBehaviour(new SendEquirePackageRequestRelay(MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_RELAY)));
					addBehaviour(new GetEnquirePackageRequest(MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_WITHOUT_ENTRANCE)));
					break;
				case ConveyorRamp.RAMP_STOREAGE:
					addBehaviour(new GetEnquirePackageRequest(
							MessageTemplate.or(	MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_WITHOUT_ENTRANCE), 
												MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_STORAGE))));
					break;
			}
		}
		
		String nickname = AgentHelper.getUniqueNickname(RampOrderAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

		if (Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 * 		PackageAgent::SendEquirePackageRequest
	 * Send message:
	 * 		RampOrderAgent::GetEnquirePackageRequest		
	 * 
	 * enquires ramp(s) if they want/demand to get a package
	 * 
     * @author Matthias
     */
	private class SendEquirePackageRequestRelay extends CyclicReceiverBehaviour {
		protected SendEquirePackageRequestRelay(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			ACLMessage enquireRampsMsg = null;
			int requestingRampType = Integer.parseInt(msg.getUserDefinedParameter("requestingRampType"));
			int packageID = Integer.parseInt(msg.getUserDefinedParameter("packageID"));
			
			switch(requestingRampType) {
				case ConveyorRamp.RAMP_ENTRANCE:
					enquireRampsMsg = new ACLMessage(MessageType.ENQUIRE_RAMPS_WITHOUT_ENTRANCE);
					break;
				case ConveyorRamp.RAMP_EXIT:
					enquireRampsMsg =  new ACLMessage(MessageType.ENQUIRE_RAMPS_STORAGE);
					break;
				default:
					return;
			}
			
			enquireRampsMsg.addUserDefinedParameter("packageID", "" + packageID);
			enquireRampsMsg.addUserDefinedParameter("requestingRampType", "" + requestingRampType);
			
			AgentHelper.addReceivers(enquireRampsMsg, myAgent, mySzenario.getId());
			
			send(enquireRampsMsg);
		}
	}
	
	/**
	 * Got message:
	 * 		RamOrderAgent::SendEquirePackageRequestRelay
	 * 		PackageAgent::Job_Available
	 * 		PackageAgent::GetPackageCountBehaviour
	 * Send message:
	 * 		PackageAgent::Job_Available
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 
	 * determines if ramp has space to take a package
	 * and demands to get it when ramp has a job for it
	 * 
     * @author Matthias
     */
	private class GetEnquirePackageRequest extends CyclicReceiverBehaviour {
		protected GetEnquirePackageRequest(MessageTemplate mt) {
			super(mt);
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,IOException {
			int packageID = Integer.parseInt(msg.getUserDefinedParameter("packageID"));
			int requestingRampType = Integer.parseInt(msg.getUserDefinedParameter("requestingRampType"));
			boolean isSpaceAvailable = false;
			boolean doDemandPackage = false;
			
			ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
			
			// entrance ramp sent enquire -> send to exit and storage ramps
			if (requestingRampType == ConveyorRamp.RAMP_ENTRANCE) {				
				// send to exit ramps and ask, if it has an available job
				if (myRampConveyor.getRampType() == ConveyorRamp.RAMP_EXIT) {					
					// ask if job is available and force demanding if so
					ACLMessage msgDemandPackage = new ACLMessage(MessageType.DEMAND_PACKAGE);
					msgDemandPackage.addUserDefinedParameter("packageID", "" + packageID);
					AgentHelper.addReceiver(msgDemandPackage, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
					send(msgDemandPackage);
					
					ACLMessage msgResponseDemand = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.DEMAND_PACKAGE));
					doDemandPackage = msgResponseDemand.getUserDefinedParameter("demanding") == "1" ? true : false;
					// only tell that you have space, when you want something
					if (doDemandPackage)
						isSpaceAvailable = true;
				
				} // send to storage ramps and ask if they have space to take a package
				else if (myRampConveyor.getRampType() == ConveyorRamp.RAMP_STOREAGE) {	
					ACLMessage msgSizeRequest = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
					AgentHelper.addReceiver(msgSizeRequest, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
					send(msgSizeRequest);
					
					ACLMessage msgSizeResponse = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT));
					int size = Integer.parseInt(msgSizeResponse.getUserDefinedParameter("package_count"));
					
					isSpaceAvailable = (myRampConveyor.getPackageCountMax() - size) > 0 ? true : false;
				}
				else { // ignore the rest
					return;
				}
			
			} // exit ramp sent enquire -> send to storage ramps 
			else if (requestingRampType == ConveyorRamp.RAMP_EXIT) {
				ACLMessage msgDemandPackage = new ACLMessage(MessageType.FIND_PACKAGE_IN_STORAGE);
				msgDemandPackage.addUserDefinedParameter("packageID", "" + packageID);
				AgentHelper.addReceiver(msgDemandPackage, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgDemandPackage);
				
				ACLMessage msgResponseDemand = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.FIND_PACKAGE_IN_STORAGE));
				doDemandPackage = msgResponseDemand.getUserDefinedParameter("demanding") == "1" ? true : false;
				
			} // if someone/something else asks, ignore it
			else {
				return;
			}
			
			if (Debugging.showDebugMessages) {
				logger.log(Level.INFO, myAgent.getLocalName() + " - send Enquire [I am: " + 
								((myRampConveyor.getRampType() == ConveyorRamp.RAMP_EXIT) ? "Exit" : "Storage") + " - " +
								"Requester: " + ((requestingRampType == ConveyorRamp.RAMP_ENTRANCE) ? "Entrance" : "Exit") +
								"]: space: " + isSpaceAvailable + " (for entrance only) - demand package: " + doDemandPackage + " - packageID: " + packageID);	
			}
			
			ACLMessage msgEnquireResponse = new ACLMessage(MessageType.ENQUIRE_RAMPS_RESPONSE);
			// only has to be processed by entrance ramps when 
			msgEnquireResponse.addUserDefinedParameter("space_available", isSpaceAvailable == true ? "1" : "0");
			// needed for entrance and exit ramps in case a package with a specific id was found
			msgEnquireResponse.addUserDefinedParameter("demand_package", doDemandPackage == true ? "1" : "0");
			
		}
	}
}
