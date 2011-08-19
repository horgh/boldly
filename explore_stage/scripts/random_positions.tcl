#!/usr/bin/tclsh8.5

proc frak {f} {
	#puts $f "[expr round( rand()*6 - 27.0)] [expr round( -55 - rand()*3)] [expr round( rand()*360 )]"
	puts $f "[expr round( rand()*13 - 53.0)] [expr round( -109 - rand()*7)] [expr round( rand()*360 )]"
}

set f [open f.txt w]

for {set x 0} {$x < 10} {incr x} {
	frak $f
}

close $f
