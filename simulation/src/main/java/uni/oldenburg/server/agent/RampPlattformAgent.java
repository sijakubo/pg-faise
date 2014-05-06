package uni.oldenburg.server.agent;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

@SuppressWarnings("serial")
public class RampPlattformAgent extends Agent {
	public final static String NAME = "RampPlattformAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	private int rampType = 0;
	private int packageCountMax = 0;
	
	private Logger logger = Logger.getLogger(RampPlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];
			
			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
			rampType = ((ConveyorRamp)myConveyor).getRampType();
			packageCountMax = myConveyor.getPackageCountMax();
		}
		
		addBehaviour(new SendRampInfoBehaviour());
		addBehaviour(new IsPackageSpaceAvailableBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 * 		JobAgent::RequestRampInfosBehaviour
	 * Send message: 
	 * 		JobAgent::ReceiveRampInfosBehaviour
	 * 
	 * send information about the ramp type
	 * 
     * @author Matthias
     */
	private class SendRampInfoBehaviour extends OneShotBehaviour {	
		public void action() {
			RampPlattformAgent currentAgent = (RampPlattformAgent)myAgent;
			
			// get ramp info request
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.REQUEST_RAMP_INFO);			
			ACLMessage msg = myAgent.blockingReceive(mt);
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- REQUEST_RAMP_INFO");
			
			// send ramp info data
			ACLMessage msgReply = new ACLMessage(MessageType.SEND_RAMP_INFO);
			msgReply.addUserDefinedParameter("rampType", "" + currentAgent.rampType);
			msgReply.addReceiver(msg.getSender());
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> SEND_RAMP_INFO");
			
			send(msgReply);
		}
	}

	/**
	 * Got message:
	 * 		JobAgent::DistributeJobBehaviour
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 		JobAgent::AssignDestinationRampBehaviour
	 * 			[after ramp was chosen]
	 * Send message:
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 			[ask ramps for item count]
	 * 		JobAgent::AssignDestinationRampBehaviour
	 * 		PackageAgent::AddPackageBehaviour
	 * 			[only if current ramp got the job]
	 * 
	 * send message if space is available for a package on this ramp
	 * and wait for a response if a new space should be "reserved" or not
	 * 
     * @author Matthias
     */
	private class IsPackageSpaceAvailableBehaviour extends CyclicBehaviour {
		int step = 0;
		int packageCount = 0;
		
		public void action() {
			RampPlattformAgent currentAgent = (RampPlattformAgent)myAgent;
			PackageData pendingPackage = null;
			
			if (step == 0) {
				// get ramp space request
				MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.PACKAGE_SPACE_AVAILABLE);
				ACLMessage msg = myAgent.receive(mt);
				
				if (msg != null) {
					if(Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " <- PACKAGE_SPACE_AVAILABLE");
					
					try {
						pendingPackage = (PackageData)msg.getContentObject();
						
						// get package count from packageagent
						// request
						ACLMessage msgPackageReq = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
						AgentHelper.addReceiver(msgPackageReq, myAgent, PackageAgent.NAME, currentAgent.conveyorID, currentAgent.szenarioID);
						
						if(Debugging.showPackageMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " -> GET_PACKAGE_COUNT");
						
						send(msgPackageReq);
						
						// response
						mt = MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT);
						ACLMessage msgPackageRes = myAgent.blockingReceive(mt);
						
						if(Debugging.showPackageMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " <- GET_PACKAGE_COUNT");
						
						packageCount = Integer.parseInt(msgPackageRes.getUserDefinedParameter("package_count"));
						int packageCountMax = currentAgent.packageCountMax;
						String isSpaceAvailable = packageCount < packageCountMax ? "1" : "0";
						
						// send ramp space info
						ACLMessage msgReply = new ACLMessage(MessageType.PACKAGE_SPACE_AVAILABLE);
						msgReply.addUserDefinedParameter("space_available", isSpaceAvailable);
						msgReply.addUserDefinedParameter("enquiring_ramp_conveyor_id",
                        msg.getUserDefinedParameter("enquiring_ramp_conveyor_id"));
                  msgReply.setContentObject(pendingPackage);
						msgReply.addReceiver(msg.getSender());
						
						if(Debugging.showInfoMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " -> PACKAGE_SPACE_AVAILABLE");
						
						++step;					
						
						send(msgReply);
					}
					catch (UnreadableException e) {
						e.printStackTrace();
					}
					catch (IOException e) {
						e.printStackTrace();
					}
				}
				else {
					block();
				}
			}
			else if (step == 1) {
				MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.RESERVE_SPACE);
				ACLMessage msg = myAgent.receive(mt);
				
				if (msg != null) {
					if(Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " <- RESERVE_SPACE");
					
					String target = msg.getUserDefinedParameter("RampAID");
					
					// reserve/add space in ramp if target matches 
					if (target.compareTo(myAgent.getAID().toString()) == 0) {
						ACLMessage msgAddPackage = new ACLMessage(MessageType.ADD_PACKAGE);
						AgentHelper.addReceiver(msgAddPackage, currentAgent, PackageAgent.NAME, conveyorID, szenarioID);
						
						++packageCount;
							
						try {
							msgAddPackage.setContentObject(msg.getContentObject());
							send(msgAddPackage);
						}
						catch (IOException e) {
							e.printStackTrace();
						}
						catch (UnreadableException e) {
							e.printStackTrace();
						}
						
						if(Debugging.showInfoMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " - space reserved: " + packageCount + "/" + currentAgent.packageCountMax);
					}
					
					step = 0;
				}
				else {
					block();
				}
			}
		}
	}
}
