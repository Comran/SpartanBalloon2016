until sudo mount /dev/sda2 /media/logger;
	do echo "failed to mount usb";
	echo "trying to mount again..." >> /home/comran/startup_log.txt;
	sleep 1;
done
