#!/bin/sh

[ -e /etc/config/poemgr ] && exit 0

. /lib/functions/uci-defaults.sh

board=$(board_name)
case "$board" in
cudy,c200p)
    cp /usr/lib/poemgr/config/c200p.config /etc/config/poemgr
    ;;
plasmacloud,psx8|\
plasmacloud,psx10)
    cp /usr/lib/poemgr/config/psx10.config /etc/config/poemgr
    ;;
plasmacloud,psx28)
    cp /usr/lib/poemgr/config/psx28.config /etc/config/poemgr
    ;;
ubnt,usw-flex)
    cp /usr/lib/poemgr/config/usw-lite.config /etc/config/poemgr
    ;;
esac
