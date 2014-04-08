package uni.oldenburg.agent.helper;

import java.util.ArrayList;
import java.util.List;

import jade.core.AID;
import jade.core.Agent;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;

public class AgentHelper {
	/**
	 * register agent for global identification
	 * 
	 * @author Matthias
	 *
	 */
	public static void registerAgent(Agent myAgent, String serviceName) {
		DFAgentDescription dfd = new DFAgentDescription();
		dfd.setName(myAgent.getAID());
		ServiceDescription sd = new ServiceDescription();
		sd.setType("FAISE");
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
	 *
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
	public static List<AID> getAgentList(Agent myAgent) {
		List<AID> lstAgentName = new ArrayList<AID>();
		
		DFAgentDescription dfd = new DFAgentDescription();
		ServiceDescription sd = new ServiceDescription();
		sd.setType("FAISE");
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
}
