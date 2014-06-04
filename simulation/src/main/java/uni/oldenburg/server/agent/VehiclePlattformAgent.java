package uni.oldenburg.server.agent;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.Point;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class VehiclePlattformAgent extends Agent {
	public final static String NAME = "VehiclePlattformAgent";
	
	private ConveyorVehicle myConveyor;
	private Szenario mySzenario;
	
	private Logger logger = Logger.getLogger(VehiclePlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (ConveyorVehicle) args[1];
		}
		
		addBehaviour(new DrivePath());
		addBehaviour(new GetCurrentPosition(MessageType.GET_CURRENT_POSITION));
		
		String nickname = AgentHelper.getUniqueNickname(VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());		
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 *		VehicleRoutingAgent::HandleEstimationRequestAssignment	
	 * Send message:
	 * 		VehicleRoutingAgent::HandleEstimationRequestAssignment
	 * 
	 * sends the coordinates of the own conveyor
	 * 
     * @author Matthias
     */
	private class GetCurrentPosition extends CyclicReceiverBehaviour {
		protected GetCurrentPosition(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			ACLMessage msgResponse = new ACLMessage(MessageType.GET_CURRENT_POSITION);
			msgResponse.addReceiver(msg.getSender());
			
			msgResponse.addUserDefinedParameter("pos_x", "" + myConveyor.getX());
			msgResponse.addUserDefinedParameter("pos_y", "" + myConveyor.getY());
			
			send(msgResponse);
		}
	}
	
	/**
	 * Got message:
	 *		VehicleRoutingAgent::AssignJob				
	 * Send message:
	 * 		RampPlattformAgent::TransferPackageRelay
	 * 		PackageAgent::TransferPackage
	 * 
	 * - drives to the source ramp
	 * - gets the package from the source ramp
	 * - drives to the destination ramp
	 * - gives package to destination ramp
	 * 
     * @author Matthias
     */
	private class DrivePath extends CyclicBehaviour {
		int step = 0;
		Point srcPoint = null;
		Point dstPoint = null;
		
		int srcRampID = 0;
		int dstRampID = 0;
		
		public void action() {
			// start driving
			if (step == 0) {
				ACLMessage msgResponse = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.DRIVING_START));
				
				if (msgResponse != null) {					
					srcRampID = Integer.parseInt(msgResponse.getUserDefinedParameter("srcRampID"));
					dstRampID = Integer.parseInt(msgResponse.getUserDefinedParameter("dstRampID"));
					
					// get entry/exit positions of ramps
					for(Conveyor tmpConveyor: mySzenario.getConveyorList()) {
						if (tmpConveyor instanceof ConveyorRamp) {
							ConveyorRamp tmpRampConveyor = (ConveyorRamp)tmpConveyor;
							
							if (tmpRampConveyor.getID() == srcRampID) {
								srcPoint = tmpRampConveyor.getExitPosition();
							}
							else if (tmpRampConveyor.getID() == dstRampID) {
								dstPoint = tmpRampConveyor.getEntryPosition();
							}
						}
					}
					
					step = 1;
				}
				else
					block();
			}
			else if (step == 1) {
				// go to source ramp							
				myConveyor.setPosition(srcPoint.getX(), srcPoint.getY(), true);
				
				
				//logger.log(Level.INFO, "Transfer 01 --- " + srcRampID + "->" + myConveyor.getID());
				// take package from source ramp to this vehicle				
				ACLMessage msgTransferFromSource = new ACLMessage(MessageType.TRANSFER_PACKAGE);
				msgTransferFromSource.addUserDefinedParameter("dstConveyorID", "" + myConveyor.getID());
				AgentHelper.addReceiver(msgTransferFromSource, myAgent, RampPlattformAgent.NAME, srcRampID, mySzenario.getId());
				send(msgTransferFromSource);
				
				myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.TRANSFER_PACKAGE_COMPLETED));
				
				//logger.log(Level.INFO, "Conveyor " + myConveyor.getID() + ": 01 TRANSFER package completed");
				
				// go to destination ramp						
				myConveyor.setPosition(dstPoint.getX(), dstPoint.getY(), true);
				
				
				logger.log(Level.INFO, "Transfer 02 --- " + myConveyor.getID() + "->" + dstRampID);
				// give package to destination ramp				
				ACLMessage msgTransferToDestination = new ACLMessage(MessageType.TRANSFER_PACKAGE);
				msgTransferToDestination.addUserDefinedParameter("dstConveyorID", "" + dstRampID);
				// -------------------------------------------------------------------------------------------------------------------------------------------------
				//msgTransferToDestination.addUserDefinedParameter("transfering", "1");
				// -------------------------------------------------------------------------------------------------------------------------------------------------				
				AgentHelper.addReceiver(msgTransferToDestination, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgTransferToDestination);
				
				myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.TRANSFER_PACKAGE_COMPLETED));
				
				logger.log(Level.INFO, "Conveyor " + myConveyor.getID() + ": 02 TRANSFER package completed");
				
				// allow new drive request
				step = 0;
			}
		}
	}
}
