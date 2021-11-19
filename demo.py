import threading
import time


class SharedCounter:
    '''
    A counter object that can be shared by multiple threads.
    '''

    def __init__(self, initial_value=0):
        self._value = initial_value
        self._value_lock = threading.Lock()

    def incr(self, delta=1):
        '''
        Increment the counter with locking
        '''
        while True:
            time.sleep(0.5)
            self._value_lock.acquire()
            self._value += 2*delta
            # self._value_lock.release()
            print('########self._value ', self._value)

    def decr(self, delta=1):
        '''
        Decrement the counter with locking
        '''
        while True:
            time.sleep(1.5)
            self._value_lock.release()
            self._value -= delta
            self._value_lock.release()


if __name__ == '__main__':
    a = SharedCounter()
    t1 = threading.Thread(target=a.incr)
    t2 = threading.Thread(target=a.decr)
    t1.start()
    t2.start()
