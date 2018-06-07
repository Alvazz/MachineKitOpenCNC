from multiprocessing import Process, Pipe, Queue
from multiprocessing.managers import BaseManager, NamespaceProxy

class testData():
    def __init__(self):
        self.a = 1
        self.b = 100

    def testfunc(self,a,b):
        return a+b

class testDataProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__', 'testfunc')
    def testfunc(self,a,b):
        callmethod = object.__getattribute__(self, '_callmethod')
        return callmethod('testfunc',args=(a,b))

class PNCAppManager(BaseManager):
    pass

PNCAppManager.register('MachineModel', testData, testDataProxy)

class TestProcess(Process):
    def __init__(self, machine):
        super(TestProcess, self).__init__()
        self.a = 1
        self.b = 2
        self.machine = machine

    def run(self):
        while True:
            print(self.machine.a)
            pass

def addValues(a,b):
    return a+b

if __name__ == "__main__":
    pncAppManager = PNCAppManager()
    pncAppManager.start()
    machine_model = pncAppManager.MachineModel()

    pnc = TestProcess(machine_model)
    pnc.start()
    #pnc.a = pncAppManager.value()

    #p = Process(target=addValues,args=(pnc.a,5))

    #print(pnc.a)
    while True:
        pass

