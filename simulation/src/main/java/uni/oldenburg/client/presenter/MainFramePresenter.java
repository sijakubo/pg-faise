package uni.oldenburg.client.presenter;


import uni.oldenburg.client.SimulationServiceAsync;

import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

public class MainFramePresenter implements Presenter {


    public interface Display {
    	Widget asWidget();
    }

    private final SimulationServiceAsync rpcService;
    private final HandlerManager eventBus;
    private final Display display;

    public MainFramePresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, Display view) {
        this.rpcService = rpcService;
        this.eventBus = eventBus;
        this.display = view;
    }

    public void bind() {

    }


    public void go(final HasWidgets container) {
      bind();
   	  container.clear();
   	  container.add(display.asWidget());
    }


}
