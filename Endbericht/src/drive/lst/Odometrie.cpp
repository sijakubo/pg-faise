void TankSteering::odomCallback(const ros::TimerEvent& event) //Funktionsaufruf durch Timer
{

	double pos_delta[2], temp_pos[2];
	for (int i = left; i <= right; i++) { //left entspricht dem Wert 0, right 1
		temp_pos[i] = tankSettings.epos[i]->getAbsolutePosition() * tankSettings.wheelPerimeter; //Bestimmung der zurueckgelegten Strecke beider Raeder
	}

	for (int i = left; i <= right; i++) {
		pos_delta[i] = temp_pos[i] - pos.lastPosition[i]; //Unterschied von jetziger zur vorheriger Position
		pos.lastPosition[i] = temp_pos[i];
	}

	double polar_s = (pos_delta[right] + pos_delta[left]) * 0.5; //Berechnung der Weglaenge
	double polar_theta = (pos_delta[right] - pos_delta[left]) / tankSettings.axisLength; // Berechnung der Drehung

	pos.now.x = pos.last.x + polar_s * cos(pos.last.theta + polar_theta); //Berechnung und Aktualisierung der x-Position
	pos.now.y = pos.last.y + polar_s * sin(pos.last.theta + polar_theta); //Berechnung und Aktualisierung der y-Position

	pos.now.theta = pos.last.theta + polar_theta; //Aktualisierung der eingenommenen Ausrichtung

}