# Default connection parameters
def_feedback_listen_ip = '0.0.0.0'
def_feedback_listen_port = 514
def_control_client_ip = '129.1.15.5'
def_control_client_port = 5007

# Initialize control communication with PocketNC using TCP and feedback read
# communication with UDP.
def commInit(feedback_listen_ip = def_feedback_listen_ip,
             feedback_listen_port = def_feedback_listen_port,
             control_client_ip = def_control_client_ip,
             control_client_port = def_control_client_port):
    global control, feedback, bokehIntf, serialIntf
    try:
        socket_fb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socket_fb.bind((feedback_listen_ip, feedback_listen_port))
    except socket.error:
        print ('Failed to bind to feedback socket to listen on')
        sys.exit()

    try:
        socket_ctrl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_ctrl.connect((control_client_ip, control_client_port))
    except socket.error:
        print ('Failed to connect to client ip for giving it control')
        sys.exit()      

    print ('[+] Listening for feedback data on port', feedback_listen_port)
    print ('[+] Connection to control client (emcrsh) established at address',
                control_client_ip,' on port ', control_client_port)

    feedback = fb_server(socket_fb)
    feedback.start()

    control = control_client(socket_ctrl)
    control.start()

######### SCULPTPRINT INTEGRATION CODE #########

def start():
    #commInit()
    return True

def read():
    #return feedbackData[-1]
    return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]]

# Returns true if monitoring is currently happening.
def isMonitoring():
    #global feed_thread
    #return feedback.is_alive()
    return True

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop():
    #global feed_thread
    #if feedback.is_alive():
    #    print('Feed thread is still alive')
    #else:
    #    print('Feed thread is not alive')
    #feedback.deactivate()
    #feedback.join()
    #feedback.close()
    #print('Buffer file was closed.\n')
    return True;
