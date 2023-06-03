#! /bin/sh
### BEGIN INIT INFO
# Provides: lookat
# Required-Start: $syslog
# Required-Stop: $syslog
# Default-Start: 2 3 4 5
# Default-Stop: 0 1 6
# Short-Description: lookat server
# Description:
### END INIT INFO

case "$1" in
    start)
        echo "lookat wird gestartet"
        # Starte Programm
        sudo runuser -u pi -- /home/pi/c_source/lookat/lookat -n
        ;;
    stop)
        echo "lookat wird beendet"
        # Beende Programm
        killall -w lookat
        ;;
    *)
        echo "Benutzt: /etc/init.d/lookat {start|stop}"
        exit 1
        ;;
esac
 
exit 0
