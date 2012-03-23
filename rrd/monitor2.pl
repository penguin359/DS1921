#!/usr/bin/perl

use 5.008_001;
use strict;
use warnings;

my $ALARM = "/Developer/Platforms/iPhoneSimulator.platform/Developer/SDKs/iPhoneSimulator3.2.sdk/System/Library/PrivateFrameworks/AOSNotification.framework/Resources/findme_alarm_1.aiff";

$|=1;

my $alarm = 0;
my $chip = "Unknown";

while(<>) {
	chomp;
	s/\r//;
	#print "$_\n";
	if(/Chip = ([A-Z0-9]+)/) {
		$chip = $1;
		$alarm = 0;
		next;
	}
	if(/ALARM/) {
		$alarm = 1;
		next;
	}
	if(/T=.*, ([-0-9.]*)F/) {
		my $alarmStr = "";
		$alarmStr = " (Alarm!)" if $alarm;
		print "$chip: $1 ºF$alarmStr\n";
		system("rrdtool","update",$chip.".rrd","N:".$1);
		system("echo \"$1\" >> \"$chip.csv\"");
		system("afplay",$ALARM) if $alarm;
	}
}
