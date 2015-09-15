
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the BSD License on
# github: https://github.com/Humarobotics/modbus_wrapper
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 
#
# Copyright (c) 2015, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#
# This wrapper package is based on the pymodbus library developed by:
# Galen Collins
# github: https://github.com/bashwork/pymodbus
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.

from post_threading import Post
from signals import Signal
import time
import sys

sys.path.append('/home/pi/pymodbus')

try:
    from pymodbus.server.sync import ModbusSocketFramer, ModbusTcpServer
    from pymodbus.device import ModbusDeviceIdentification
    from pymodbus.datastore import ModbusSequentialDataBlock
    from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
except Exception,e:
    print "-- INSTALL DEPENDENCIES -- "
    print "sudo apt-get install python-pymodbus"    
    print "or from source:"
    print "git clone git@github.com:bashwork/pymodbus.git"
    print "cd pymodbus"
    print "sudo python setup.py install"
    print "and"
    print "sudo apt-get install python-pyasn1 python-twisted-conch"
    print e
    exit()

ADDRESS_WRITE_START = 0
ADDRESS_READ_START = 0

class CustomHoldingRegister(ModbusSequentialDataBlock):
    def __init__(self,address,value):
        """
            Creates a custom holding register to add a publisher and subscriber to the modbus server
            
            :param address: The starting address of the holding register
            :type address: int
            :param values: The initial values for the holding registers
            :type values: list[int]
            :param sub_topic: ROS topic name for the subscriber that updates the modbus registers
            :type sub_topic: string
            :param pub_topic: ROS topic name for the publisher that publishes a message, once there is something written to the writeable modbus registers
            :type pub_topic: string
        """
        
        super(CustomHoldingRegister,self).__init__(address,value)
        self.reading_start = ADDRESS_READ_START
        self.writing_start = ADDRESS_WRITE_START        
        
    def setValues(self, address, value):
        """ 
            Sets the requested values to the holding registers
             

            :param address: The starting address
            :type address: int
            :param values: The new values to be set
            :type values: list[int]
        """
        #print "address",address,"value",value  
        ModbusSequentialDataBlock.setValues(self,address, value)
        
            
        
    def __updateWriteableRegisters(self,msg):
        if len(msg.data) > self.reading_start+1: # how does this one work?
            print "Message to long. Shorten it or it will be ignored"
        self.setValues(self.writing_start, list(msg.data))
        


class ModbusWrapperServer():
    def __init__(self,port=1234):
        """
            Creates a Modbus TCP Server object
            .. note:: The default port for modbus is 502. This modbus server uses port 1234 by default, otherwise superuser rights are required.
            
            .. note:: Use "startServer" to start the listener.
            
            :param port: Port for the modbus TCP server
            :type port: int
                        
        """
#         chr = CustomHoldingRegister(ADDRESS_WRITE_START, [16]*100)
#         cdi = CustomHoldingRegister(ADDRESS_WRITE_START, [16]*100)
        self.onShutdown=False;
        self.state_changed = Signal()
        cco = CustomHoldingRegister(ADDRESS_WRITE_START, [16]*100)
        self.store = ModbusSlaveContext(
            di = ModbusSequentialDataBlock(ADDRESS_WRITE_START, [16]*100),
            co = cco,
            hr = ModbusSequentialDataBlock(ADDRESS_WRITE_START, [16]*100), 
            ir = ModbusSequentialDataBlock(ADDRESS_WRITE_START, [16]*100))
        self.context = ModbusServerContext(slaves=self.store, single=True)

        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName  = 'Pymodbus'
        self.identity.ProductCode = 'PM'
        self.identity.VendorUrl   = 'http://github.com/bashwork/pymodbus/'
        self.identity.ProductName = 'Pymodbus Server'
        self.identity.ModelName   = 'Pymodbus Server'
        self.identity.MajorMinorRevision = '1.0'

        self.store.setValues(2,0,[0]*1)
        self.post = Post(self)
        framer = ModbusSocketFramer
        self.server = ModbusTcpServer(self.context, framer, self.identity, address=("0.0.0.0", port))
        
    
        
    def startServer(self):
        """
            Non blocking call to start the server
        """
        self.post._startServer()
        
    def _startServer(self):
        print "Modbus server started"
        self.server.serve_forever()
        
    def stopServer(self):
        """
            Closes the server
        """
        self.onShutdown = True
        self.server.server_close()
        self.server.shutdown()
        
    def waitForCoilOutput(self,address,timeout=2):
        now = time.time()
        while not self.onShutdown:
            values = self.store.getValues(1,address, 1)
            if values[0] is True:
                return True
            else:
                if timeout > 0 and now + timeout > time.time():
                    time.sleep(1/50)
                else:
                    return False
    
    def stateChangeListener(self,address):
        self.post.__stateChangeListener(address)
    
    def __stateChangeListener(self,address):
        old_state = 0;
        while not self.onShutdown:
            value = self.store.getValues(1,address, 1)[0]
            
            if old_state != value:
                old_state = value
                self.state_changed(address,value)
                #print "state has changed"
            time.sleep(1/50) 
    
    def setDigitalInput(self,address,values):
        if not values is list:
            values = [values]
        self.store.setValues(2,address,values)
        
    def triggerInput(self,address,timeout=0.5):
        self.setDigitalInput(address, True)
        self.post.resetInput(address, timeout)
        
    def resetInput(self,address,timeout):
        time.sleep(timeout)
        self.setDigitalInput(address, False)
        
