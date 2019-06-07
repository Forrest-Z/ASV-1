close all;
clear;


timeselect=[10 100]; % second
Path='20190606/test1/';
% read sql file
controllerdata=csvread(strcat(Path,'controller.csv'),1,0);
estimatordata=csvread(strcat(Path,'estimator.csv'),1,0);
plannerdata=csvread(strcat(Path,'planner.csv'),1,0);
gpsdata = readgps(strcat(Path,'gps.csv'));

timestamp0=min(controllerdata(1,2),estimatordata(1,2));
timestamp0=min(timestamp0,plannerdata(1,2));
timestamp0=min(timestamp0,gpsdata.DATETIME(1));

timestamp_controller=(controllerdata(:,2)-timestamp0)*86400.0;
index_select=find(timeselect(1)<=timestamp_controller & timestamp_controller<= timeselect(2));
controller.timestamp=timestamp_controller(index_select);
controller.tau=controllerdata(index_select,3:5);
controller.alpha=controllerdata(index_select,6:11);
controller.rpm=controllerdata(index_select,12:17);
controller.est=controllerdata(index_select,18:20);

timestamp_estimator=(estimatordata(:,2)-timestamp0)*86400.0;
index_select=find(timeselect(1)<=timestamp_estimator & timestamp_estimator<= timeselect(2));
estimator.timestamp=timestamp_controller(index_select);
estimator.measurement=estimatordata(index_select,3:8);
estimator.state=estimatordata(index_select,9:14);
estimator.perror=estimatordata(index_select,15:17);
estimator.verror=estimatordata(index_select,18:20);

timestamp_gps=(gpsdata.DATETIME-timestamp0)*86400.0;
index_select=find(timeselect(1)<=timestamp_gps & timestamp_gps<= timeselect(2));
gps.timestamp=timestamp_gps(index_select);
gps.heading=gpsdata.heading(index_select);
gps.pitch=gpsdata.pitch(index_select);
gps.roll=gpsdata.roll(index_select);
gps.u=gpsdata.Ve(index_select);
gps.v=gpsdata.Vn(index_select);
gps.x=gpsdata.UTM_x(index_select);
gps.x=gpsdata.UTM_y(index_select);