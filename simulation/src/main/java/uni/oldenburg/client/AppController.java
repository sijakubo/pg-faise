package uni.oldenburg.client;

import com.google.gwt.core.client.GWT;
import com.google.gwt.event.logical.shared.ValueChangeEvent;
import com.google.gwt.event.logical.shared.ValueChangeHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.History;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;
import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.client.presenter.Presenter;
import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.client.service.SimulationService;
import uni.oldenburg.client.service.SimulationServiceAsync;
import uni.oldenburg.client.view.MainFrameView;

/**
 * Klasse initialisiert das Hauptfenster und ermöglicht einen Viewwechsel. Falls weitere Views eingebaut werden sollen, kann in der onValueChange
 * Methode ein Viewwechsel eingebaut werden.
 * @author Raschid
 * 
 */
public class AppController extends Presenter implements ValueChangeHandler<String> {
   private HasWidgets container;

   public AppController(ServiceAsync rpcService, HandlerManager eventBus) {
      super(rpcService, eventBus);
      bind();
   }

   public Widget getDisplay() {
      return null;
   }

   public void bind() {
      History.addValueChangeHandler(this);
   }

   @Override
   public void go(final HasWidgets container) {
      this.container = container;

      if ("".equals(History.getToken())) {
         History.newItem("MainFrame");
      } else {
         History.fireCurrentHistoryState();
      }
   }

   public void onValueChange(ValueChangeEvent<String> event) {
      String token = event.getValue();

      if (token == null)
         return;

      SimulationServiceAsync identityService = GWT.create(SimulationService.class);
      Presenter presenter = new MainFramePresenter(identityService, eventBus, new MainFrameView());

      presenter.go(container);
   }
}

