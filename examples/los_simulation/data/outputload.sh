#!/bin/bash
 sqlite3 ./dbsave.db <<!
.headers on
.mode csv
.output estimator.csv
select * from estimator;
.output planner.csv
select * from planner;
.output controller.csv
select * from controller;
.output GPS.csv
select * from GPS;
