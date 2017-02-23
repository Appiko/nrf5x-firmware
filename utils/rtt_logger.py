#!/usr/bin/env python
# Copyright (c) 2017 Sampad Mohanty, Appiko
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import socket
import datetime
import subprocess,time,sys,os

if len(sys.argv)<2:
	print "Enter the first argument as either 'nrf51' or 'nrf52'"
	sys.exit(0)

device = sys.argv[1]

if (device != "nrf51") and (device != "nrf52"):
	print "Enter the first argument as either 'nrf51' or 'nrf52'"
	sys.exit(0)

#Start the JLinkExe so that the RTT server is started
print "Starting Server..."
subprocess.call("/opt/SEGGER/JLink/JLinkExe -If swd -device " + device + " -speed 4000 -AutoConnect 1 &",shell=True)
time.sleep(1)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('localhost', 19021)

print "Connecting to the server @",server_address
sock.connect(server_address)

time.sleep(1)

past = 0
base = 0

try:
	while True:
    	#read up to 1000 bytes from the socket (the max amount)
		rcv=sock.recv(1000)
    	#we didn't get any data.
		if rcv=="":
			continue

    	#Set base time as the time when first character is received
		if not base:
			base = time.time()

		current = time.time()
		elapsed = current - base
		delta = elapsed - past
		past = elapsed 
		
		current_str = datetime.datetime.now().strftime("%H:%M:%S.%f")
		print_msg = "[%s %2.6f] " % (current_str, delta)
		print (print_msg + rcv).strip()

#On Ctrl + C
except KeyboardInterrupt:
	print "Disconnecting socket and killing JLinkExe\n"
	del sock
	os.system("pkill JLinkExe")
	time.sleep(1)
	sys.exit(0)

#On any exception
except Exception as e:
	print "Exception: " + str(e) + "\nDisconnecting socket and killing JLinkExe\n"
	del sock
	os.system("pkill JLinkExe")
	time.sleep(1)
	sys.exit(0)
