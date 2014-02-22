package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.RegistrationPresenter;

import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.PasswordTextBox;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;

public class RegistrationView extends Composite implements RegistrationPresenter.IDisplay {
	private TextBox txtUsername;
	private PasswordTextBox txtPassword;
	private Button btnRegister;
	
	public RegistrationView() {
		VerticalPanel vpLogin = new VerticalPanel();
		
		txtUsername = new TextBox();
		txtUsername.setText("Username");
		txtUsername.setPixelSize(200, 30);
		vpLogin.add(txtUsername);
		
		txtPassword = new PasswordTextBox();
		txtPassword.setText("Password");
		txtPassword.setPixelSize(200, 30);
		vpLogin.add(txtPassword);
		
		btnRegister = new Button("Registrieren");
		btnRegister.setText("Registrieren");
		btnRegister.setSize("100px", "30px");		
		vpLogin.add(btnRegister);
		
		initWidget(vpLogin);
    }

	public HasValue<String> getUsername() {
		return txtUsername;
	}

	public HasValue<String> getPassword() {
		return txtPassword;
	}

	public HasClickHandlers getRegisterButton() {
		return btnRegister;
	}
}
