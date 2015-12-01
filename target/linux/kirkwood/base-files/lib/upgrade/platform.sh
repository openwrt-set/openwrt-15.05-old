#!/bin/sh

. /lib/functions/system.sh
. /lib/kirkwood.sh

platform_do_upgrade() {
	local board=$(kirkwood_board_name)
	
	case "$board" in
		*)
			default_do_upgrade "$ARGV"
			;;
	esac
}

platform_check_image() {
	local board=$(kirkwood_board_name)

	case "$board" in
		"irz_kw04")
			nand_do_platform_check $board $1
			return $?
			;;
	esac
	
	echo "Sysupgrade is not yet supported on $board"
	return 1
}

append sysupgrade_pre_upgrade
