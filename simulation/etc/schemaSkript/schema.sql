CREATE TABLE joblist (	
	id integer primary key,
	name varchar(200)
);

CREATE TABLE job (
	id integer primary key,
	joblist_id integer references joblist(id),
	destination_id integer,
	time_delay integer
);

CREATE TABLE szenario (
	id integer primary key,
	title varchar(200),
	time_created date
);
	
CREATE TABLE szenario_conveyor (
	id integer primary key,
  type varchar(200),
	pos_x integer,
	pos_y integer,
	direction integer,
	szenario_id integer references szenario(id)
);