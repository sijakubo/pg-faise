package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.LoginPresenter;

import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.PasswordTextBox;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;

public class LoginView extends Composite implements LoginPresenter.IDisplay {
	private TextBox txtUsername;
	private PasswordTextBox txtPassword;
	private Button btnLogin;
	private Button btnRegister;
  
	public LoginView() {
		VerticalPanel vpLogin = new VerticalPanel();
		
		txtUsername = new TextBox();
		txtUsername.setText("Username");
		txtUsername.setPixelSize(200, 30);
		vpLogin.add(txtUsername);
		
		txtPassword = new PasswordTextBox();
		txtPassword.setText("Password");
		txtPassword.setPixelSize(200, 30);
		vpLogin.add(txtPassword);
		
		HorizontalPanel hpButtons = new HorizontalPanel();
		btnLogin = new Button("Senden");
		btnLogin.setText("Login");
		btnLogin.setSize("100px", "30px");
		hpButtons.add(btnLogin);
		
		btnRegister = new Button("Registrieren");
		btnRegister.setText("Registrieren");
		btnRegister.setSize("100px", "30px");
		hpButtons.add(btnRegister);
		
		vpLogin.add(hpButtons);
		
		initWidget(vpLogin);
  }
  
  
  public HasValue<String> getUsername() {
	  return txtUsername;
  }
	
  public HasValue<String> getPassword() {
	  return txtPassword;
  }
	
  public HasClickHandlers getLoginButton() {
	  return btnLogin;
  }
	
  public HasClickHandlers getRegisterButton() {
	  return btnRegister;
  }
}
