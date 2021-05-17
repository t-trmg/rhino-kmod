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

echo "This will take about 2 minutes..."
for REG in `seq 1 45`
do
	dump_reg $REG
done
dump_reg 0
