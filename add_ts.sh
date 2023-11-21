#!/bin/bash

tail -f $1 | ts '%H:%M:%S' >> $1_ts.log 
