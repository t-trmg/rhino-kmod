#!/bin/bash
DIR=/sys/module/rcbfx/parameters

function dump_reg
{
	REG=$1
	echo Reg $REG
	echo $REG > $DIR/reg_addr
	sleep 2
	cat $DIR/reg_addr $DIR/reg_val
}

if [ -z "$1" ]
then
	echo "No register specified - Probing all possible register locations. This will take about 10 minutes. Sit back, relax, and enjoy some espresso."
	for REG in `seq 1 255`
	do
		dump_reg $REG
	done
	dump_reg 0
else
	dump_reg 0	# Clear
	dump_reg $1
fi
