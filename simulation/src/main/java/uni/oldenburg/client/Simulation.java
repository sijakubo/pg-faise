package uni.oldenburg.client;

import com.google.gwt.core.client.EntryPoint;
import com.google.gwt.core.client.GWT;
import com.google.gwt.event.shared.HandlerManager;

import com.google.gwt.user.client.ui.RootPanel;

import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.client.service.SimulationService;
import uni.oldenburg.client.util.LoggingUtil;

/**
 * Entry point Klasse. Beim starten der GWT-Applikation wird die onModuleLoad-Methode aufgerufen.
 * Es werden alle notwendigen Klassen fuer Viewwechsel und Client-Server Kommunikation initialisiert.
 * @author Raschid  
 */
public class Simulation implements EntryPoint {

	/**
	 * This is the entry point method.
	 */
	public void onModuleLoad() {
		GWT.setUncaughtExceptionHandler(new GWT.UncaughtExceptionHandler() {
			public void onUncaughtException(Throwable e) {
				LoggingUtil.logMessageToServer("Uncaught Exception on Client",
						e);
			}
		});

		try {
			ServiceAsync rpcService = GWT.create(SimulationService.class);

			HandlerManager eventBus = new HandlerManager(null);
			AppController appViewer = new AppController(rpcService, eventBus);
			appViewer.go(RootPanel.get("main"));
		} catch (RuntimeException ex) {
			GWT.getUncaughtExceptionHandler().onUncaughtException(ex);
		}
	}
}
