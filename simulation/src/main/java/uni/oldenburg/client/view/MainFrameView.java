package uni.oldenburg.client.view;

import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.PasswordTextBox;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;

import uni.oldenburg.client.presenter.MainFramePresenter;

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
