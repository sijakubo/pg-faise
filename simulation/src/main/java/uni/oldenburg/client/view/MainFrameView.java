package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.VerticalPanel;

public class MainFrameView extends Composite implements
		MainFramePresenter.Display {

	private Button loginButton;

	public MainFrameView() {
		VerticalPanel absolutePanel = new VerticalPanel();

		initWidget(absolutePanel);

		loginButton = new Button("Hauptfenster");
		loginButton.setText("Login");
		loginButton.setSize("100px", "30px");
		absolutePanel.add(loginButton);
	}

}
