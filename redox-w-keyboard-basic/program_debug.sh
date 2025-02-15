#!/bin/bash

MAKEDIR=$(dirname "$(readlink -f "$0")")/custom/armgcc/
HEX=_build/nrf51822_xxac-keyboard-debug.hex

echo '=============================== MAKING ================================'
make -C ${MAKEDIR}
if [[ $? -ne 0 ]] ; then
    exit 0
fi
sleep 0.1
HEX=`readlink -f $(dirname "$(readlink -f "$0")")/custom/armgcc/${HEX}`
du -b $HEX

echo
echo '============================= PROGRAMMING ============================='
{
	echo "program $HEX verify reset"
	sleep 2
} | telnet 127.0.0.1 4444

echo
echo '============================== FINISHED ==============================='
