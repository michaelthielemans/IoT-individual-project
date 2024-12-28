#!/bin/bash
### BEGIN INIT INFO
# Provides:          start_ir_keytable
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Description:       Set up IR keytable
### END INIT INFO

# Run ir-keytable command
sudo ir-keytable -p lirc,nec
