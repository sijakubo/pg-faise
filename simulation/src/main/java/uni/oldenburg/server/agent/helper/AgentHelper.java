package uni.oldenburg.server.agent.helper;

import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.SzenarioInfo;
import jade.core.AID;
import jade.core.Agent;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

public class AgentHelper {	
	/**
	 * register agent for global identification
	 * 
	 * @author Matthias
	 *
	 */
	public static void registerAgent(int szenarioID, Agent myAgent, String serviceName) {
		DFAgentDescription dfd = new DFAgentDescription();
		dfd.setName(myAgent.getAID());
		ServiceDescription sd = new ServiceDescription();
		sd.setType("FAISE-" + szenarioID);
		sd.setName(serviceName);
		dfd.addServices(sd);
		
		try {
			DFService.register(myAgent, dfd);
		} catch (FIPAException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * unregister agent as global identifier
	 * 
	 * @author Matthias
	 */
	public static void unregister(Agent myAgent) {
		try {
			DFService.deregister(myAgent);
		} catch (FIPAException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * retrieve list of all currently registered agents
	 * 
	 * @author Matthias
	 *
	 */
	public static List<AID> getAgentList(int szenarioID, Agent myAgent) {
		List<AID> lstAgentName = new ArrayList<AID>();
		
		DFAgentDescription dfd = new DFAgentDescription();
		ServiceDescription sd = new ServiceDescription();
		sd.setType("FAISE-" + szenarioID);
		dfd.addServices(sd);
		
		try {
			DFAgentDescription[] result = DFService.search(myAgent, dfd);

			for (int i = 0; i < result.length; i++) {
				lstAgentName.add(result[i].getName());
			}
			
		} catch (FIPAException e) {
			e.printStackTrace();
		}
		
		return lstAgentName;		
	}
	
	/**
	 * add receiver with specific agent name / address
	 * 
     * @author Matthias
     */
	public static void addReceiver(ACLMessage msg, Agent myAgent, String targetAgentName, int conveyorID, int szenarioID) {
		List<AID> lstAID = getAgentList(szenarioID, myAgent);
		
		String nickname = AgentHelper.getUniqueNickname(targetAgentName, conveyorID, szenarioID);
		
		for (AID myAID : lstAID) {
			if (myAID.toString().contains(nickname)) {
				msg.addReceiver(myAID);
			}
		}
	}
	
	/**
	 * set list of AIDs as receivers
	 * 
	 * @author Matthias
	 */
	public static void addReceivers(ACLMessage msg, List<AID> lstAID) {
		for (AID myAID : lstAID) {	
			msg.addReceiver(myAID);
		}
	}

	/**
	 * set all agents in current szenario as receivers
	 * 
	 * @author Matthias
	 */
	public static void addReceivers(ACLMessage msg, Agent myAgent, int szenarioID) {
		List<AID> lstAID = getAgentList(szenarioID, myAgent);
		
		AgentHelper.addReceivers(msg, lstAID);
	}
	
	/**
	 * compute unique name for agents
	 * 
	 * @author Matthias
	 *
	 */	
	public static String getUniqueNickname(String nickname, int conveyorID, int szenarioID) {
		nickname += "-" + szenarioID;
		
		if (conveyorID > 0)
			nickname += "-" + conveyorID;
		
		return nickname;
	}
	
	/**
	 * retrieve infos (ramp type counts) about the current szenario
	 * 
	 * @author Matthias
	 */
	public static SzenarioInfo calculateRampCounts(Szenario mySzenario) {
		SzenarioInfo myInfo = new SzenarioInfo();
		
		for(Conveyor myConveyor : mySzenario.getConveyorList()) {
			if (myConveyor instanceof ConveyorRamp) {
				ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
				
				switch(myRampConveyor.getRampType()) {
					case ConveyorRamp.RAMP_ENTRANCE:
						++myInfo.EntryRampCount;
						break;
					case ConveyorRamp.RAMP_EXIT:
						++myInfo.ExitRampCount;
						break;
					case ConveyorRamp.RAMP_STOREAGE:
						++myInfo.StorageRampCount;
						break;
				}
			}
		}
		
		return myInfo;
	}
}
