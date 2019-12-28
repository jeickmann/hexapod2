from microWebSrv import MicroWebSrv
import struct

dirformat = '>iff'
dirformat_index = 0
logformat = '>i200p'
logformat_index = 1

def _recvBinaryCallback(webSocket, data):
    if(data[0] == dirformat_index):
        (type, x, y) = struct.unpack(dirformat, data)
        print("Received %f %f", x, y)
        logmsg = struct.pack(logformat, logformat_index, bytes('Hallo WS Welt', 'utf-8'))
        webSocket.SendBinary(logmsg)

def _acceptWebSocketCallback(webSocket, httpClient) :
  print("WS ACCEPT")
  webSocket.RecvBinaryCallback = _recvBinaryCallback
  webSocket.RecvTextCallback = _recvTextCallback

def startWebServer(path="/web"):
    mws = MicroWebSrv(port=8090, webPath=path)                                 
    mws.MaxWebSocketRecvLen     = 256                      # Default is set to 1024
    mws.AcceptWebSocketCallback = _acceptWebSocketCallback # Function to receive WebSockets
    mws.Start(threaded=True)                               # Starts server in a new thread