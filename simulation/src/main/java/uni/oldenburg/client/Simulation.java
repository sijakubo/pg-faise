package uni.oldenburg.client;

import com.google.gwt.core.client.EntryPoint;
import com.google.gwt.core.client.GWT;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.RootPanel;

import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.client.service.SimulationService;

/**
 * Entry point classes define <code>onModuleLoad()</code>.
 */

public class Simulation implements EntryPoint {
	
	
	/**
	  * This is the entry point method.
	  */	
	 public void onModuleLoad() {
		 ServiceAsync rpcService = GWT.create(SimulationService.class);
		 HandlerManager eventBus = new HandlerManager(null);
		 AppController appViewer = new AppController(rpcService, eventBus);
		 appViewer.go(RootPanel.get("main"));
	  }
}
