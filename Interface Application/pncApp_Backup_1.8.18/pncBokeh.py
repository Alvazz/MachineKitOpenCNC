### Bokeh Plotting for PocketNC ###

import socket
import numpy as np
import threading
import sys
import pickle

from bokeh.plotting import figure, curdoc
from bokeh.driving import linear
from bokeh.models import SingleIntervalTicker, LinearAxis
import random

# Globals
lastNdxReceived = 0
#socket_pnc = 0
pncCommHost = '0.0.0.0'
pncCommPort = 6666
pncIntf = 0

# Data Storage
#plotData = 0
#plotData = dict(Time=[], queueLengths=[], xPosns=[])
pncData = 0

class pncCommListener(threading.Thread):
    def __init__(self, conn):
        super(pncCommListener, self).__init__()
        #self.tcqPlotData = []
        self.needUpdate = 0
        self.conn = conn
        self.data = ""
        print('pncComm thread started')

    def run(self):
        while True:
            (bytes_received, rec_address) = self.conn.recvfrom(65536)
            #dataArray = pickle.loads(bytes_received)
            dataDict = pickle.loads(bytes_received)
            #print(repr(dataDict))
            self.data = dataDict
            self.needUpdate = 1
            #string_received = bytes_received.decode("utf-8")
            #print(string_received)
            #self.data = self.data + dataArray

            #if self.data.endswith(u"&"):
            #    print('Received complete string', string_received)
            #    #process_data_points(self.data)
            #    self.data = ""
            #    self.needUpdate = 1

    def initPlots(self):
        print('initializing plots')

    def send_msg(self,msg):
        self.conn.send(msg)

    def close(self):
        self.conn.close()

def init():
    global pncIntf#, pncSocket, pncCommHost, pncCommPort
    try:
        #establish reader from pncComm
        pncSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #pncSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        pncSocket.bind((pncCommHost,pncCommPort))
    except socket.error:
        print ('Failed to create feedback socket')
        #pncSocket.close()
        sys.exit()

    pncIntf = pncCommListener(pncSocket)
    pncIntf.start()

#update_lock = threading.Lock()
queueLength = figure(title="Buffer Fill Level", plot_width=800, plot_height=400)
r1 = queueLength.line([], [], color="firebrick", line_width=2)

threadTime = figure(title="Z-Axis Encoder", plot_width=800, plot_height=400)
r2 = threadTime.line([], [], color="firebrick", line_width=2)
ticker = SingleIntervalTicker(interval=5, num_minor_ticks=10)
xaxis = LinearAxis(ticker=ticker)
yaxis = LinearAxis(ticker=ticker)

ds1 = r1.data_source
ds2 = r2.data_source

@linear()
def update(step):
    #print(step)
    if pncIntf.needUpdate:
        #print('need to update plots')
        queue_data = pncIntf.data['TCQ']
        encoder_data = pncIntf.data['Encoder']
        #encoder_len = len(encoder_data)
       
        pncIntf.needUpdate = 0
        ds1.data = dict(x = np.arange(len(queue_data)), y = queue_data)
        ds1.trigger('data', ds1.data, ds1.data)
        ds2.data = dict(x = np.arange(len(encoder_data)), y = encoder_data)
        ds2.trigger('data', ds2.data, ds2.data)
        
    #ds1.trigger('data', ds1.data, ds1.data)
    #ds2.trigger('data', ds2.data, ds2.data)


curdoc().add_root(queueLength)
curdoc().add_root(threadTime)
init()

# Add a periodic callback to be run every 500 milliseconds
curdoc().add_periodic_callback(update, 100)
curdoc().title = "PocketNC Interface"
